/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_i2s

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/cache.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <soc.h>

#include <bflb_soc.h>
#include <glb_reg.h>
#include <bouffalolab/common/i2s_reg.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2s_bflb, CONFIG_I2S_LOG_LEVEL);

/* I2S hardware mode values */
#define BFLB_I2S_MODE_LEFT_JUSTIFIED  0U
#define BFLB_I2S_MODE_RIGHT_JUSTIFIED 1U
#define BFLB_I2S_MODE_DSP             2U

/* I2S hardware frame/data size values */
#define BFLB_I2S_SIZE_8  0U
#define BFLB_I2S_SIZE_16 1U
#define BFLB_I2S_SIZE_24 2U
#define BFLB_I2S_SIZE_32 3U

/* Clock enable register offsets and bits vary by SoC */
#if defined(CONFIG_SOC_SERIES_BL70X)
#define BFLB_I2S_CLK_REG_OFFSET GLB_CLK_CFG3_OFFSET
#define BFLB_I2S_CLK_EN_BIT     BIT(13)
#define BFLB_I2S_CLK_SEL_BIT    BIT(12)
#elif defined(CONFIG_SOC_SERIES_BL61X)
#define BFLB_I2S_CLK_REG_OFFSET GLB_I2S_CFG0_OFFSET
#define BFLB_I2S_CLK_EN_BIT     BIT(7)
#endif

struct i2s_bflb_queue_item {
	void *mem_block;
	size_t size;
};

struct i2s_bflb_stream {
	int32_t state;
	struct k_msgq queue;
	struct i2s_config cfg;
	const struct device *dma_dev;
	uint32_t dma_channel;
	uint32_t dma_trigger;
	struct dma_config dma_cfg;
	void *mem_block;
	bool last_block;
	bool tx_drain;
};

struct i2s_bflb_cfg {
	uint32_t base;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config)(const struct device *dev);
};

struct i2s_bflb_data {
	struct i2s_bflb_stream tx;
	struct i2s_bflb_stream rx;
};

static inline uint32_t i2s_bflb_reg_read(const struct i2s_bflb_cfg *cfg, uint32_t offset)
{
	return sys_read32(cfg->base + offset);
}

static inline void i2s_bflb_reg_write(const struct i2s_bflb_cfg *cfg, uint32_t offset, uint32_t val)
{
	sys_write32(val, cfg->base + offset);
}

static inline void i2s_bflb_reg_set_bits(const struct i2s_bflb_cfg *cfg, uint32_t offset,
					 uint32_t bits)
{
	uint32_t val = i2s_bflb_reg_read(cfg, offset);

	val |= bits;
	i2s_bflb_reg_write(cfg, offset, val);
}

static inline void i2s_bflb_reg_clr_bits(const struct i2s_bflb_cfg *cfg, uint32_t offset,
					 uint32_t bits)
{
	uint32_t val = i2s_bflb_reg_read(cfg, offset);

	val &= ~bits;
	i2s_bflb_reg_write(cfg, offset, val);
}

static bool queue_is_empty(struct k_msgq *q)
{
	return k_msgq_num_used_get(q) == 0U;
}

static k_timeout_t i2s_bflb_timeout(int32_t timeout)
{
	if (timeout == SYS_FOREVER_MS) {
		return K_FOREVER;
	}

	return SYS_TIMEOUT_MS(timeout);
}

static int queue_get(struct k_msgq *q, void **mem_block, size_t *size, int32_t timeout)
{
	struct i2s_bflb_queue_item item;
	int ret;

	ret = k_msgq_get(q, &item, i2s_bflb_timeout(timeout));
	if (ret == 0) {
		*mem_block = item.mem_block;
		*size = item.size;
	}

	return ret;
}

static int queue_put(struct k_msgq *q, void *mem_block, size_t size, int32_t timeout)
{
	struct i2s_bflb_queue_item item = {
		.mem_block = mem_block,
		.size = size,
	};

	return k_msgq_put(q, &item, i2s_bflb_timeout(timeout));
}

static void stream_queue_drop(struct i2s_bflb_stream *stream)
{
	size_t size;
	void *mem_block;

	while (queue_get(&stream->queue, &mem_block, &size, 0) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, mem_block);
	}
}

static void i2s_bflb_enable_clock(void)
{
	uint32_t val;

	val = sys_read32(GLB_BASE + BFLB_I2S_CLK_REG_OFFSET);
	val |= BFLB_I2S_CLK_EN_BIT;
	sys_write32(val, GLB_BASE + BFLB_I2S_CLK_REG_OFFSET);
}

static void i2s_bflb_disable_hw(const struct i2s_bflb_cfg *cfg)
{
	/* Disable master and slave modes */
	i2s_bflb_reg_clr_bits(cfg, I2S_CONFIG_OFFSET, I2S_CR_I2S_M_EN | I2S_CR_I2S_S_EN);

	/* Disable TX and RX data */
	i2s_bflb_reg_clr_bits(cfg, I2S_CONFIG_OFFSET, I2S_CR_I2S_TXD_EN | I2S_CR_I2S_RXD_EN);

	/* Disable DMA */
	i2s_bflb_reg_clr_bits(cfg, I2S_FIFO_CONFIG_0_OFFSET, I2S_DMA_TX_EN | I2S_DMA_RX_EN);
}

static int i2s_bflb_set_clock(const struct i2s_bflb_cfg *cfg, uint32_t bclk_freq)
{
	uint32_t peri_clk;
	uint32_t div;
	uint32_t div_l;
	uint32_t div_h;
	uint32_t val;

	/*
	 * Get the I2S peripheral clock frequency.
	 * On BL70x this is BCLK (system bus clock) by default.
	 * On BL61x this is the I2S reference clock (defaults to BCLK).
	 */
	peri_clk = sys_clock_hw_cycles_per_sec();

	if (bclk_freq == 0U) {
		return -EINVAL;
	}

	/* Calculate divider with rounding: div = round(peri_clk / bclk_freq) - 2 */
	div = (uint32_t)((((uint64_t)peri_clk * 10U) / bclk_freq + 5U) / 10U);
	if (div >= 2U) {
		div -= 2U;
	} else {
		div = 0U;
	}

	/* Clamp to 12-bit max */
	if (div > 0xFFFU) {
		div = 0xFFFU;
	}

	/* Split into high/low halves for 50% duty cycle */
	div_l = div / 2U;
	div_h = div - div_l;

	val = (div_l << I2S_CR_BCLK_DIV_L_SHIFT) & I2S_CR_BCLK_DIV_L_MASK;
	val |= (div_h << I2S_CR_BCLK_DIV_H_SHIFT) & I2S_CR_BCLK_DIV_H_MASK;
	i2s_bflb_reg_write(cfg, I2S_BCLK_CONFIG_OFFSET, val);

	LOG_DBG("peri_clk=%u bclk_freq=%u div=%u (L=%u H=%u)", peri_clk, bclk_freq, div, div_l,
		div_h);

	return 0;
}

static int i2s_bflb_word_size_to_hw(uint8_t word_size)
{
	switch (word_size) {
	case 8U:
		return BFLB_I2S_SIZE_8;
	case 16U:
		return BFLB_I2S_SIZE_16;
	case 24U:
		return BFLB_I2S_SIZE_24;
	case 32U:
		return BFLB_I2S_SIZE_32;
	default:
		return -EINVAL;
	}
}

static uint8_t i2s_bflb_dma_width(uint8_t word_size)
{
	if (word_size <= 8U) {
		return 1U;
	} else if (word_size <= 16U) {
		return 2U;
	} else {
		return 4U;
	}
}

static void dma_rx_callback(const struct device *dma_dev, void *user_data, uint32_t channel,
			    int status);
static void dma_tx_callback(const struct device *dma_dev, void *user_data, uint32_t channel,
			    int status);

static int i2s_bflb_configure(const struct device *dev, enum i2s_dir dir,
			      const struct i2s_config *i2s_cfg)
{
	const struct i2s_bflb_cfg *cfg = dev->config;
	struct i2s_bflb_data *data = dev->data;
	struct i2s_bflb_stream *stream;
	bool is_master;
	uint32_t config_val;
	uint32_t bclk_freq;
	uint32_t num_channels;
	uint32_t frame_bits;
	int hw_data_size;
	int hw_frame_size;
	int ret;

	if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else if (dir == I2S_DIR_BOTH) {
		return -ENOSYS;
	} else {
		LOG_ERR("Invalid direction");
		return -EINVAL;
	}

	if ((stream->state != I2S_STATE_NOT_READY) && (stream->state != I2S_STATE_READY)) {
		LOG_ERR("Invalid state %d for configure", stream->state);
		return -EINVAL;
	}

	/* Determine master/slave from options */
	is_master = true;
	if ((i2s_cfg->options & I2S_OPT_BIT_CLK_TARGET) != 0U ||
	    (i2s_cfg->options & I2S_OPT_FRAME_CLK_TARGET) != 0U) {
		is_master = false;
	}

	/* Setting frame_clk_freq to 0 resets the interface */
	if (i2s_cfg->frame_clk_freq == 0U) {
		stream_queue_drop(stream);
		(void)memset(&stream->cfg, 0, sizeof(struct i2s_config));
		stream->state = I2S_STATE_NOT_READY;
		return 0;
	}

	/* Validate and convert word size */
	hw_data_size = i2s_bflb_word_size_to_hw(i2s_cfg->word_size);
	if (hw_data_size < 0) {
		LOG_ERR("Unsupported word size %u", i2s_cfg->word_size);
		return -EINVAL;
	}

	/*
	 * Frame size (slot width) is at least as large as data size.
	 * For I2S standard format, frame size per channel is typically
	 * 16 or 32 bits regardless of actual data width.
	 */
	if (i2s_cfg->word_size <= 16U) {
		frame_bits = 16U;
	} else {
		frame_bits = 32U;
	}
	hw_frame_size = i2s_bflb_word_size_to_hw(frame_bits);

	/*
	 * For I2S standard format, number of channels is always 2.
	 * For other formats, use the configured channel count.
	 */
	if ((i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) == I2S_FMT_DATA_FORMAT_I2S) {
		num_channels = 2U;
	} else {
		num_channels = i2s_cfg->channels;
	}

	if ((num_channels < 1U) || (num_channels > 4U)) {
		LOG_ERR("Unsupported channel count %u", num_channels);
		return -EINVAL;
	}

	/* Store config */
	(void)memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));

	/* Build the I2S_CONFIG register value */
	config_val = i2s_bflb_reg_read(cfg, I2S_CONFIG_OFFSET);

	/* Clear all configurable bits */
	config_val &= ~(I2S_CR_I2S_M_EN | I2S_CR_I2S_S_EN | I2S_CR_I2S_TXD_EN | I2S_CR_I2S_RXD_EN |
			I2S_CR_MONO_MODE | I2S_CR_MUTE_MODE | I2S_CR_FS_1T_MODE |
			I2S_CR_FS_CH_CNT_MASK | I2S_CR_FRAME_SIZE_MASK | I2S_CR_DATA_SIZE_MASK |
			I2S_CR_I2S_MODE_MASK | I2S_CR_ENDIAN | I2S_CR_OFS_CNT_MASK | I2S_CR_OFS_EN);

	/* Set frame and data size */
	config_val |= ((uint32_t)hw_frame_size << I2S_CR_FRAME_SIZE_SHIFT) & I2S_CR_FRAME_SIZE_MASK;
	config_val |= ((uint32_t)hw_data_size << I2S_CR_DATA_SIZE_SHIFT) & I2S_CR_DATA_SIZE_MASK;

	/* Set channel count (hardware encoding: 0=1ch, 1=2ch, 2=3ch, 3=4ch) */
	if (num_channels >= 2U) {
		config_val |=
			((num_channels - 1U) << I2S_CR_FS_CH_CNT_SHIFT) & I2S_CR_FS_CH_CNT_MASK;
	}

	/* Mono mode for single channel */
	if (num_channels == 1U) {
		config_val |= I2S_CR_MONO_MODE;
	}

	/* Set data format */
	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		/*
		 * Standard I2S (Philips) is left-justified with 1-bit offset.
		 * BFLB hardware: mode=0 (left-justified) + OFS_EN + OFS_CNT=1.
		 */
		config_val |= (BFLB_I2S_MODE_LEFT_JUSTIFIED << I2S_CR_I2S_MODE_SHIFT) &
			      I2S_CR_I2S_MODE_MASK;
		config_val |= I2S_CR_OFS_EN;
		config_val |= (1U << I2S_CR_OFS_CNT_SHIFT) & I2S_CR_OFS_CNT_MASK;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		config_val |= (BFLB_I2S_MODE_DSP << I2S_CR_I2S_MODE_SHIFT) & I2S_CR_I2S_MODE_MASK;
		config_val |= I2S_CR_FS_1T_MODE;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		config_val |= (BFLB_I2S_MODE_DSP << I2S_CR_I2S_MODE_SHIFT) & I2S_CR_I2S_MODE_MASK;
		break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		config_val |= (BFLB_I2S_MODE_LEFT_JUSTIFIED << I2S_CR_I2S_MODE_SHIFT) &
			      I2S_CR_I2S_MODE_MASK;
		break;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		config_val |= (BFLB_I2S_MODE_RIGHT_JUSTIFIED << I2S_CR_I2S_MODE_SHIFT) &
			      I2S_CR_I2S_MODE_MASK;
		break;
	default:
		LOG_ERR("Unsupported data format");
		return -EINVAL;
	}

	/* Bit order */
	if ((i2s_cfg->format & I2S_FMT_DATA_ORDER_LSB) != 0U) {
		config_val |= I2S_CR_ENDIAN;
	}

	i2s_bflb_reg_write(cfg, I2S_CONFIG_OFFSET, config_val);

	/* Configure clock polarity in I2S_IO_CONFIG */
	{
		uint32_t io_val = i2s_bflb_reg_read(cfg, I2S_IO_CONFIG_OFFSET);

		io_val &= ~(I2S_CR_I2S_BCLK_INV | I2S_CR_I2S_FS_INV);

		if ((i2s_cfg->format & I2S_FMT_BIT_CLK_INV) != 0U) {
			io_val |= I2S_CR_I2S_BCLK_INV;
		}
		if ((i2s_cfg->format & I2S_FMT_FRAME_CLK_INV) != 0U) {
			io_val |= I2S_CR_I2S_FS_INV;
		}

		i2s_bflb_reg_write(cfg, I2S_IO_CONFIG_OFFSET, io_val);
	}

	/* Set BCLK frequency (only in master mode) */
	if (is_master) {
		bclk_freq = i2s_cfg->frame_clk_freq * frame_bits * num_channels;
		ret = i2s_bflb_set_clock(cfg, bclk_freq);
		if (ret < 0) {
			return ret;
		}
	}

	/* Configure FIFO: clear FIFOs, enable L/R merge for stereo <=16 bit */
	{
		uint32_t fifo0 = i2s_bflb_reg_read(cfg, I2S_FIFO_CONFIG_0_OFFSET);

		/* Clear both FIFOs */
		fifo0 |= I2S_TX_FIFO_CLR | I2S_RX_FIFO_CLR;

		/* L/R merge: pack both channels into one 32-bit FIFO entry */
		if ((num_channels == 2U) && (i2s_cfg->word_size <= 16U)) {
			fifo0 |= I2S_CR_FIFO_LR_MERGE;
		} else {
			fifo0 &= ~I2S_CR_FIFO_LR_MERGE;
		}

		fifo0 &= ~I2S_CR_FIFO_LR_EXCHG;
		fifo0 &= ~I2S_CR_FIFO_24B_LJ;

		i2s_bflb_reg_write(cfg, I2S_FIFO_CONFIG_0_OFFSET, fifo0);
	}

	/* Set FIFO thresholds */
	{
		uint32_t fifo1 = i2s_bflb_reg_read(cfg, I2S_FIFO_CONFIG_1_OFFSET);

		fifo1 &= ~(I2S_TX_FIFO_TH_MASK | I2S_RX_FIFO_TH_MASK);
		/* TX threshold: fire when TX FIFO has room for at least half */
		fifo1 |= (8U << I2S_TX_FIFO_TH_SHIFT) & I2S_TX_FIFO_TH_MASK;
		/* RX threshold: fire when RX FIFO has at least this many */
		fifo1 |= (8U << I2S_RX_FIFO_TH_SHIFT) & I2S_RX_FIFO_TH_MASK;
		i2s_bflb_reg_write(cfg, I2S_FIFO_CONFIG_1_OFFSET, fifo1);
	}

	/* Prepare DMA config */
	{
		uint8_t dma_width = i2s_bflb_dma_width(i2s_cfg->word_size);
		struct dma_config *dma = &stream->dma_cfg;

		(void)memset(dma, 0, sizeof(struct dma_config));

		/*
		 * With L/R merge enabled (stereo <=16 bit), both channels are
		 * packed into a 32-bit word, so DMA width must be 4 bytes.
		 */
		if ((num_channels == 2U) && (i2s_cfg->word_size <= 16U)) {
			dma_width = 4U;
		}

		dma->source_data_size = dma_width;
		dma->dest_data_size = dma_width;
		dma->source_burst_length = 4U;
		dma->dest_burst_length = 4U;
		dma->dma_slot = stream->dma_trigger;
		dma->block_count = 1U;
		dma->user_data = (void *)dev;

		if (dir == I2S_DIR_TX) {
			dma->channel_direction = MEMORY_TO_PERIPHERAL;
			dma->dma_callback = dma_tx_callback;
		} else {
			dma->channel_direction = PERIPHERAL_TO_MEMORY;
			dma->dma_callback = dma_rx_callback;
		}
	}

	stream->state = I2S_STATE_READY;

	return 0;
}

static const struct i2s_config *i2s_bflb_config_get(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_bflb_data *data = dev->data;
	struct i2s_bflb_stream *stream;

	if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else {
		return NULL;
	}

	if (stream->state == I2S_STATE_NOT_READY) {
		return NULL;
	}

	return &stream->cfg;
}

static int i2s_bflb_start_dma(struct i2s_bflb_stream *stream, const struct i2s_bflb_cfg *cfg,
			      void *buf, size_t buf_size, enum i2s_dir dir)
{
	struct dma_block_config blk_cfg;
	int ret;

	(void)memset(&blk_cfg, 0, sizeof(blk_cfg));
	blk_cfg.block_size = buf_size;

	if (dir == I2S_DIR_TX) {
		blk_cfg.source_address = (uint32_t)buf;
		blk_cfg.dest_address = cfg->base + I2S_FIFO_WDATA_OFFSET;
		blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		blk_cfg.source_address = cfg->base + I2S_FIFO_RDATA_OFFSET;
		blk_cfg.dest_address = (uint32_t)buf;
		blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}

	stream->dma_cfg.head_block = &blk_cfg;

	ret = dma_config(stream->dma_dev, stream->dma_channel, &stream->dma_cfg);
	if (ret < 0) {
		return ret;
	}

	ret = dma_start(stream->dma_dev, stream->dma_channel);

	return ret;
}

static int i2s_bflb_reload_dma(struct i2s_bflb_stream *stream, const struct i2s_bflb_cfg *cfg,
			       void *buf, size_t buf_size, enum i2s_dir dir)
{
	uint32_t src;
	uint32_t dst;
	int ret;

	if (dir == I2S_DIR_TX) {
		src = (uint32_t)buf;
		dst = cfg->base + I2S_FIFO_WDATA_OFFSET;
	} else {
		src = cfg->base + I2S_FIFO_RDATA_OFFSET;
		dst = (uint32_t)buf;
	}

	ret = dma_reload(stream->dma_dev, stream->dma_channel, src, dst, buf_size);
	if (ret < 0) {
		return ret;
	}

	return dma_start(stream->dma_dev, stream->dma_channel);
}

static int rx_stream_start(struct i2s_bflb_stream *stream, const struct device *dev)
{
	const struct i2s_bflb_cfg *cfg = dev->config;
	bool is_master;
	uint32_t config_val;
	int ret;

	/* Allocate initial RX buffer */
	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("Failed to allocate RX buffer");
		return ret;
	}

	/* Start DMA for RX */
	ret = i2s_bflb_start_dma(stream, cfg, stream->mem_block, stream->cfg.block_size,
				 I2S_DIR_RX);
	if (ret < 0) {
		LOG_ERR("Failed to start RX DMA: %d", ret);
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
		return ret;
	}

	/* Enable RX DMA in I2S */
	i2s_bflb_reg_set_bits(cfg, I2S_FIFO_CONFIG_0_OFFSET, I2S_DMA_RX_EN);

	/* Enable RX data path */
	i2s_bflb_reg_set_bits(cfg, I2S_CONFIG_OFFSET, I2S_CR_I2S_RXD_EN);

	/* Enable master or slave */
	is_master = (stream->cfg.options & I2S_OPT_BIT_CLK_TARGET) == 0U;
	config_val = i2s_bflb_reg_read(cfg, I2S_CONFIG_OFFSET);
	if (is_master) {
		config_val |= I2S_CR_I2S_M_EN;
	} else {
		config_val |= I2S_CR_I2S_S_EN;
	}
	i2s_bflb_reg_write(cfg, I2S_CONFIG_OFFSET, config_val);

	return 0;
}

static int tx_stream_start(struct i2s_bflb_stream *stream, const struct device *dev)
{
	const struct i2s_bflb_cfg *cfg = dev->config;
	bool is_master;
	uint32_t config_val;
	size_t mem_block_size;
	int ret;

	/* Get first TX buffer from queue */
	ret = queue_get(&stream->queue, &stream->mem_block, &mem_block_size, 0);
	if (ret < 0) {
		LOG_ERR("No TX data queued");
		return ret;
	}

	/* Flush cache before DMA reads from memory */
	sys_cache_data_flush_range(stream->mem_block, mem_block_size);

	/* Start DMA for TX */
	ret = i2s_bflb_start_dma(stream, cfg, stream->mem_block, mem_block_size, I2S_DIR_TX);
	if (ret < 0) {
		LOG_ERR("Failed to start TX DMA: %d", ret);
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
		return ret;
	}

	/* Enable TX DMA in I2S */
	i2s_bflb_reg_set_bits(cfg, I2S_FIFO_CONFIG_0_OFFSET, I2S_DMA_TX_EN);

	/* Enable TX data path */
	i2s_bflb_reg_set_bits(cfg, I2S_CONFIG_OFFSET, I2S_CR_I2S_TXD_EN);

	/* Enable master or slave */
	is_master = (stream->cfg.options & I2S_OPT_BIT_CLK_TARGET) == 0U;
	config_val = i2s_bflb_reg_read(cfg, I2S_CONFIG_OFFSET);
	if (is_master) {
		config_val |= I2S_CR_I2S_M_EN;
	} else {
		config_val |= I2S_CR_I2S_S_EN;
	}
	i2s_bflb_reg_write(cfg, I2S_CONFIG_OFFSET, config_val);

	return 0;
}

static void rx_stream_disable(struct i2s_bflb_stream *stream, const struct device *dev)
{
	const struct i2s_bflb_cfg *cfg = dev->config;

	/* Stop DMA */
	dma_stop(stream->dma_dev, stream->dma_channel);

	/* Disable RX DMA and data path */
	i2s_bflb_reg_clr_bits(cfg, I2S_FIFO_CONFIG_0_OFFSET, I2S_DMA_RX_EN);
	i2s_bflb_reg_clr_bits(cfg, I2S_CONFIG_OFFSET, I2S_CR_I2S_RXD_EN);

	/* Free active buffer */
	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}

	/* Disable I2S if TX is also not running */
	{
		uint32_t config_val = i2s_bflb_reg_read(cfg, I2S_CONFIG_OFFSET);

		if ((config_val & I2S_CR_I2S_TXD_EN) == 0U) {
			config_val &= ~(I2S_CR_I2S_M_EN | I2S_CR_I2S_S_EN);
			i2s_bflb_reg_write(cfg, I2S_CONFIG_OFFSET, config_val);
		}
	}
}

static void tx_stream_disable(struct i2s_bflb_stream *stream, const struct device *dev)
{
	const struct i2s_bflb_cfg *cfg = dev->config;

	/* Stop DMA */
	dma_stop(stream->dma_dev, stream->dma_channel);

	/* Disable TX DMA and data path */
	i2s_bflb_reg_clr_bits(cfg, I2S_FIFO_CONFIG_0_OFFSET, I2S_DMA_TX_EN);
	i2s_bflb_reg_clr_bits(cfg, I2S_CONFIG_OFFSET, I2S_CR_I2S_TXD_EN);

	/* Free active buffer */
	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}

	/* Disable I2S if RX is also not running */
	{
		uint32_t config_val = i2s_bflb_reg_read(cfg, I2S_CONFIG_OFFSET);

		if ((config_val & I2S_CR_I2S_RXD_EN) == 0U) {
			config_val &= ~(I2S_CR_I2S_M_EN | I2S_CR_I2S_S_EN);
			i2s_bflb_reg_write(cfg, I2S_CONFIG_OFFSET, config_val);
		}
	}
}

static void dma_rx_callback(const struct device *dma_dev, void *user_data, uint32_t channel,
			    int status)
{
	const struct device *dev = user_data;
	const struct i2s_bflb_cfg *cfg = dev->config;
	struct i2s_bflb_data *data = dev->data;
	struct i2s_bflb_stream *stream = &data->rx;
	void *completed_block;
	int ret;

	if (status < 0) {
		stream->state = I2S_STATE_ERROR;
		rx_stream_disable(stream, dev);
		return;
	}

	if (stream->state == I2S_STATE_ERROR) {
		rx_stream_disable(stream, dev);
		return;
	}

	/* Save completed block */
	completed_block = stream->mem_block;

	/* Allocate next RX buffer */
	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block, K_NO_WAIT);
	if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		rx_stream_disable(stream, dev);
		return;
	}

	/* Reload DMA with new buffer */
	ret = i2s_bflb_reload_dma(stream, cfg, stream->mem_block, stream->cfg.block_size,
				  I2S_DIR_RX);
	if (ret < 0) {
		LOG_ERR("Failed to reload RX DMA: %d", ret);
		k_mem_slab_free(stream->cfg.mem_slab, completed_block);
		stream->state = I2S_STATE_ERROR;
		rx_stream_disable(stream, dev);
		return;
	}

	/* Invalidate cache for completed buffer (DMA wrote to it) */
	sys_cache_data_invd_range(completed_block, stream->cfg.block_size);

	/* Queue completed block for application to read */
	ret = queue_put(&stream->queue, completed_block, stream->cfg.block_size, 0);
	if (ret < 0) {
		k_mem_slab_free(stream->cfg.mem_slab, completed_block);
		stream->state = I2S_STATE_ERROR;
		rx_stream_disable(stream, dev);
		return;
	}

	/* Check if stopping was requested */
	if (stream->state == I2S_STATE_STOPPING) {
		stream->state = I2S_STATE_READY;
		rx_stream_disable(stream, dev);
	}
}

static void dma_tx_callback(const struct device *dma_dev, void *user_data, uint32_t channel,
			    int status)
{
	const struct device *dev = user_data;
	const struct i2s_bflb_cfg *cfg = dev->config;
	struct i2s_bflb_data *data = dev->data;
	struct i2s_bflb_stream *stream = &data->tx;
	size_t mem_block_size;
	int ret;

	if (status < 0) {
		stream->state = I2S_STATE_ERROR;
		tx_stream_disable(stream, dev);
		return;
	}

	/* Free completed TX block */
	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}

	if (stream->state == I2S_STATE_ERROR) {
		tx_stream_disable(stream, dev);
		return;
	}

	/* Handle STOPPING state */
	if (stream->state == I2S_STATE_STOPPING) {
		if (!stream->tx_drain) {
			/* STOP: discard remaining data immediately */
			stream_queue_drop(stream);
			stream->state = I2S_STATE_READY;
			tx_stream_disable(stream, dev);
			return;
		}
		/* DRAIN: stop only when queue is empty */
		if (queue_is_empty(&stream->queue)) {
			stream->state = I2S_STATE_READY;
			tx_stream_disable(stream, dev);
			return;
		}
		/* DRAIN with data remaining: fall through to send next block */
	}

	/* Check for last block */
	if (stream->last_block) {
		stream->state = I2S_STATE_READY;
		tx_stream_disable(stream, dev);
		return;
	}

	/* Get next TX block from queue */
	ret = queue_get(&stream->queue, &stream->mem_block, &mem_block_size, 0);
	if (ret < 0) {
		if (stream->state == I2S_STATE_STOPPING) {
			stream->state = I2S_STATE_READY;
		} else {
			stream->state = I2S_STATE_ERROR;
		}
		tx_stream_disable(stream, dev);
		return;
	}

	/* Flush cache before DMA reads */
	sys_cache_data_flush_range(stream->mem_block, mem_block_size);

	/* Reload DMA with next buffer */
	ret = i2s_bflb_reload_dma(stream, cfg, stream->mem_block, mem_block_size, I2S_DIR_TX);
	if (ret < 0) {
		LOG_ERR("Failed to reload TX DMA: %d", ret);
		stream->state = I2S_STATE_ERROR;
		tx_stream_disable(stream, dev);
	}
}

static int i2s_bflb_trigger(const struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	struct i2s_bflb_data *data = dev->data;
	struct i2s_bflb_stream *stream;
	unsigned int key;
	int ret;

	if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else if (dir == I2S_DIR_BOTH) {
		return -ENOSYS;
	} else {
		LOG_ERR("Invalid direction");
		return -EINVAL;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			LOG_ERR("START: invalid state %d", stream->state);
			return -EIO;
		}

		stream->last_block = false;

		if (dir == I2S_DIR_TX) {
			ret = tx_stream_start(stream, dev);
		} else {
			ret = rx_stream_start(stream, dev);
		}

		if (ret < 0) {
			LOG_ERR("START failed: %d", ret);
			return ret;
		}

		stream->state = I2S_STATE_RUNNING;
		break;

	case I2S_TRIGGER_STOP:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("STOP: invalid state %d", stream->state);
			return -EIO;
		}
		stream->state = I2S_STATE_STOPPING;
		stream->tx_drain = false;
		irq_unlock(key);
		break;

	case I2S_TRIGGER_DRAIN:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("DRAIN: invalid state %d", stream->state);
			return -EIO;
		}

		if (dir == I2S_DIR_TX) {
			if (queue_is_empty(&stream->queue)) {
				stream->state = I2S_STATE_READY;
				tx_stream_disable(stream, dev);
			} else {
				stream->state = I2S_STATE_STOPPING;
				stream->tx_drain = true;
			}
		} else {
			/* RX DRAIN behaves like STOP */
			stream->state = I2S_STATE_STOPPING;
			stream->tx_drain = false;
		}
		irq_unlock(key);
		break;

	case I2S_TRIGGER_DROP:
		key = irq_lock();
		if (stream->state == I2S_STATE_NOT_READY) {
			irq_unlock(key);
			LOG_ERR("DROP: invalid state");
			return -EIO;
		}

		if (dir == I2S_DIR_TX) {
			tx_stream_disable(stream, dev);
		} else {
			rx_stream_disable(stream, dev);
		}
		stream_queue_drop(stream);
		stream->state = I2S_STATE_READY;
		irq_unlock(key);
		break;

	case I2S_TRIGGER_PREPARE:
		if (stream->state != I2S_STATE_ERROR) {
			LOG_ERR("PREPARE: invalid state %d", stream->state);
			return -EIO;
		}
		stream->state = I2S_STATE_READY;
		stream_queue_drop(stream);
		break;

	default:
		LOG_ERR("Unsupported trigger command %d", cmd);
		return -EINVAL;
	}

	return 0;
}

static int i2s_bflb_api_read(const struct device *dev, void **mem_block, size_t *size)
{
	struct i2s_bflb_data *data = dev->data;
	struct i2s_bflb_stream *stream = &data->rx;

	if (stream->state == I2S_STATE_NOT_READY) {
		LOG_ERR("RX not configured");
		return -EIO;
	}

	return queue_get(&stream->queue, mem_block, size, stream->cfg.timeout);
}

static int i2s_bflb_api_write(const struct device *dev, void *mem_block, size_t size)
{
	struct i2s_bflb_data *data = dev->data;
	struct i2s_bflb_stream *stream = &data->tx;

	if ((stream->state != I2S_STATE_RUNNING) && (stream->state != I2S_STATE_READY)) {
		LOG_ERR("TX not ready");
		return -EIO;
	}

	return queue_put(&stream->queue, mem_block, size, stream->cfg.timeout);
}

static void i2s_bflb_isr(const struct device *dev)
{
	const struct i2s_bflb_cfg *cfg = dev->config;
	uint32_t fifo_status;

	/* Check for FIFO errors */
	fifo_status = i2s_bflb_reg_read(cfg, I2S_FIFO_CONFIG_0_OFFSET);

	if ((fifo_status & (I2S_TX_FIFO_OVERFLOW | I2S_TX_FIFO_UNDERFLOW)) != 0U) {
		LOG_WRN("TX FIFO error: 0x%08x", fifo_status);
		/* Clear TX FIFO */
		i2s_bflb_reg_set_bits(cfg, I2S_FIFO_CONFIG_0_OFFSET, I2S_TX_FIFO_CLR);
	}

	if ((fifo_status & (I2S_RX_FIFO_OVERFLOW | I2S_RX_FIFO_UNDERFLOW)) != 0U) {
		LOG_WRN("RX FIFO error: 0x%08x", fifo_status);
		/* Clear RX FIFO */
		i2s_bflb_reg_set_bits(cfg, I2S_FIFO_CONFIG_0_OFFSET, I2S_RX_FIFO_CLR);
	}

	/* Clear interrupt status */
	{
		uint32_t int_sts = i2s_bflb_reg_read(cfg, I2S_INT_STS_OFFSET);

		/* Mask all fired interrupts */
		int_sts |= I2S_CR_I2S_TXF_MASK | I2S_CR_I2S_RXF_MASK | I2S_CR_I2S_FER_MASK;
		i2s_bflb_reg_write(cfg, I2S_INT_STS_OFFSET, int_sts);
	}
}

static int i2s_bflb_init(const struct device *dev)
{
	const struct i2s_bflb_cfg *cfg = dev->config;
	struct i2s_bflb_data *data = dev->data;
	int ret;

	/* Enable I2S peripheral clock */
	i2s_bflb_enable_clock();

	/* Configure pinctrl */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Pinctrl setup failed: %d", ret);
		return ret;
	}

	/* Disable I2S hardware */
	i2s_bflb_disable_hw(cfg);

	/* Clear FIFOs */
	i2s_bflb_reg_set_bits(cfg, I2S_FIFO_CONFIG_0_OFFSET, I2S_TX_FIFO_CLR | I2S_RX_FIFO_CLR);

	/* Mask all interrupts initially */
	{
		uint32_t int_sts = i2s_bflb_reg_read(cfg, I2S_INT_STS_OFFSET);

		int_sts |= I2S_CR_I2S_TXF_MASK | I2S_CR_I2S_RXF_MASK | I2S_CR_I2S_FER_MASK;
		i2s_bflb_reg_write(cfg, I2S_INT_STS_OFFSET, int_sts);
	}

	/* Configure IRQ */
	cfg->irq_config(dev);

	/* Verify DMA devices are ready */
	if (!device_is_ready(data->tx.dma_dev)) {
		LOG_ERR("TX DMA device not ready");
		return -ENODEV;
	}

	if (!device_is_ready(data->rx.dma_dev)) {
		LOG_ERR("RX DMA device not ready");
		return -ENODEV;
	}

	/* Initialize stream states */
	data->tx.state = I2S_STATE_NOT_READY;
	data->tx.mem_block = NULL;
	data->rx.state = I2S_STATE_NOT_READY;
	data->rx.mem_block = NULL;

	LOG_INF("I2S @0x%08x initialized", cfg->base);

	return 0;
}

static DEVICE_API(i2s, i2s_bflb_driver_api) = {
	.configure = i2s_bflb_configure,
	.config_get = i2s_bflb_config_get,
	.read = i2s_bflb_api_read,
	.write = i2s_bflb_api_write,
	.trigger = i2s_bflb_trigger,
};

#define I2S_BFLB_INIT(index)                                                                       \
                                                                                                   \
	static void i2s_bflb_irq_config_##index(const struct device *dev);                         \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static const struct i2s_bflb_cfg i2s_bflb_cfg_##index = {                                  \
		.base = DT_INST_REG_ADDR(index),                                                   \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		.irq_config = i2s_bflb_irq_config_##index,                                         \
	};                                                                                         \
                                                                                                   \
	static struct i2s_bflb_queue_item                                                          \
		i2s_bflb_rx_queue_items_##index[CONFIG_I2S_BFLB_RX_BLOCK_COUNT];                   \
	static struct i2s_bflb_queue_item                                                          \
		i2s_bflb_tx_queue_items_##index[CONFIG_I2S_BFLB_TX_BLOCK_COUNT];                   \
                                                                                                   \
	static struct i2s_bflb_data i2s_bflb_data_##index = {                                      \
		.tx =                                                                              \
			{                                                                          \
				.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(index, tx)),    \
				.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, tx, channel),      \
				.dma_trigger = DT_INST_DMAS_CELL_BY_NAME(index, tx, trigsrc),      \
			},                                                                         \
		.rx =                                                                              \
			{                                                                          \
				.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(index, rx)),    \
				.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, rx, channel),      \
				.dma_trigger = DT_INST_DMAS_CELL_BY_NAME(index, rx, trigsrc),      \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	static int i2s_bflb_init_queues_##index(const struct device *dev)                          \
	{                                                                                          \
		struct i2s_bflb_data *data = dev->data;                                            \
                                                                                                   \
		k_msgq_init(&data->tx.queue, (char *)i2s_bflb_tx_queue_items_##index,              \
			    sizeof(struct i2s_bflb_queue_item), CONFIG_I2S_BFLB_TX_BLOCK_COUNT);   \
		k_msgq_init(&data->rx.queue, (char *)i2s_bflb_rx_queue_items_##index,              \
			    sizeof(struct i2s_bflb_queue_item), CONFIG_I2S_BFLB_RX_BLOCK_COUNT);   \
                                                                                                   \
		return i2s_bflb_init(dev);                                                         \
	}                                                                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, i2s_bflb_init_queues_##index, NULL, &i2s_bflb_data_##index,   \
			      &i2s_bflb_cfg_##index, POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,        \
			      &i2s_bflb_driver_api);                                               \
                                                                                                   \
	static void i2s_bflb_irq_config_##index(const struct device *dev)                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), i2s_bflb_isr,       \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}

DT_INST_FOREACH_STATUS_OKAY(I2S_BFLB_INIT)
