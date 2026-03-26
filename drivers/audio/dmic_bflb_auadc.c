/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_auadc

#include <zephyr/audio/dmic.h>
#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <soc.h>

#include <bflb_soc.h>
#include <bouffalolab/common/auadc_reg.h>

#if defined(CONFIG_SOC_SERIES_BL61X)
#include <glb_reg.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dmic_bflb_auadc, CONFIG_AUDIO_DMIC_LOG_LEVEL);

/* Input mode values */
#define AUADC_INPUT_MODE_ADC      0U
#define AUADC_INPUT_MODE_PDM_LEFT 1U
#define AUADC_INPUT_MODE_PDM_RIGHT 2U

/* Sample rate encoding for AUADC_ADC_RATE field */
#define AUADC_RATE_8K  0U
#define AUADC_RATE_16K 1U
#define AUADC_RATE_24K 2U
#define AUADC_RATE_32K 3U
#define AUADC_RATE_48K 4U

/* RX FIFO data resolution */
#define AUADC_RX_DATA_RES_16BIT 2U
#define AUADC_RX_DATA_RES_20BIT 0U
#define AUADC_RX_DATA_RES_24BIT 1U
#define AUADC_RX_DATA_RES_32BIT 3U

/* Default RX FIFO trigger level */
#define AUADC_RX_TRG_LEVEL_DEFAULT 3U

/* Default PGA settings for analog ADC mode */
#define AUADC_PGA_GAIN_DEFAULT   0U
#define AUADC_PGA_MODE_AC        0U
#define AUADC_CHANNEL_SELP_AUDIO 1U
#define AUADC_CHANNEL_SELN_GND   0U
#define AUADC_CHANNEL_EN_POS     1U

/* Default analog config (from SDK bflb_auadc.c defaults) */
#define AUADC_ANA_CFG1_DEFAULT 0x01050510U
#define AUADC_ANA_CFG2_DEFAULT 0x00000140U

struct dmic_bflb_auadc_cfg {
	uint32_t base;
	const struct device *dma_dev;
	uint32_t dma_channel;
	uint32_t dma_trigger;
	uint8_t input_mode;
};

struct dmic_bflb_auadc_data {
	enum dmic_state state;
	struct k_mem_slab *mem_slab;
	size_t block_size;
	struct k_msgq rx_queue;
	void *active_block;
	struct dma_config dma_cfg;
	uint32_t pcm_rate;
};

/* Register access helpers */

static inline uint32_t auadc_read(const struct dmic_bflb_auadc_cfg *cfg, uint32_t offset)
{
	return sys_read32(cfg->base + offset);
}

static inline void auadc_write(const struct dmic_bflb_auadc_cfg *cfg, uint32_t offset,
				uint32_t val)
{
	sys_write32(val, cfg->base + offset);
}

static inline void auadc_set_bits(const struct dmic_bflb_auadc_cfg *cfg, uint32_t offset,
				   uint32_t bits)
{
	uint32_t val = auadc_read(cfg, offset);

	val |= bits;
	auadc_write(cfg, offset, val);
}

static inline void auadc_clr_bits(const struct dmic_bflb_auadc_cfg *cfg, uint32_t offset,
				   uint32_t bits)
{
	uint32_t val = auadc_read(cfg, offset);

	val &= ~bits;
	auadc_write(cfg, offset, val);
}

static inline void auadc_modify(const struct dmic_bflb_auadc_cfg *cfg, uint32_t offset,
				 uint32_t mask, uint32_t val)
{
	uint32_t reg = auadc_read(cfg, offset);

	reg &= ~mask;
	reg |= (val & mask);
	auadc_write(cfg, offset, reg);
}

/* Map PCM sample rate to hardware encoding */
static int auadc_rate_to_hw(uint32_t pcm_rate, uint32_t *hw_rate)
{
	switch (pcm_rate) {
	case 8000U:
		*hw_rate = AUADC_RATE_8K;
		return 0;
	case 16000U:
		*hw_rate = AUADC_RATE_16K;
		return 0;
	case 24000U:
		*hw_rate = AUADC_RATE_24K;
		return 0;
	case 32000U:
		*hw_rate = AUADC_RATE_32K;
		return 0;
	case 48000U:
		*hw_rate = AUADC_RATE_48K;
		return 0;
	default:
		return -EINVAL;
	}
}

/* Map PCM width to RX FIFO data resolution */
static int auadc_width_to_res(uint8_t pcm_width, uint32_t *data_res)
{
	switch (pcm_width) {
	case 16U:
		*data_res = AUADC_RX_DATA_RES_16BIT;
		return 0;
	case 20U:
		*data_res = AUADC_RX_DATA_RES_20BIT;
		return 0;
	case 24U:
		*data_res = AUADC_RX_DATA_RES_24BIT;
		return 0;
	case 32U:
		*data_res = AUADC_RX_DATA_RES_32BIT;
		return 0;
	default:
		return -EINVAL;
	}
}

/* DMA width in bytes for a given PCM width */
static uint8_t auadc_dma_width(uint8_t pcm_width)
{
	if (pcm_width <= 8U) {
		return 1U;
	} else if (pcm_width <= 16U) {
		return 2U;
	} else {
		return 4U;
	}
}

#if defined(CONFIG_SOC_SERIES_BL61X)
static void auadc_enable_clock(void)
{
	/* BL61x: GLB_BASE + GLB_AUDIO_CFG0_OFFSET, bit 15 = audio ADC clock enable */
	sys_set_bits(GLB_BASE + GLB_AUDIO_CFG0_OFFSET, BIT(15));
}
#else
static void auadc_enable_clock(void)
{
	/* BL70xL: no dedicated audio clock gate */
}
#endif

static void auadc_hw_configure(const struct dmic_bflb_auadc_cfg *cfg, uint32_t hw_rate,
				uint32_t data_res)
{
	uint32_t val;

	/* 1. Enable audio clock gate and set sample rate */
	val = AUADC_AUDIO_CKG_EN;
	val |= (hw_rate << AUADC_ADC_RATE_SHIFT) & AUADC_ADC_RATE_MASK;
	auadc_write(cfg, AUADC_AUDPDM_TOP_OFFSET, val);

	/* 2. AUDPDM_ITF: clear ADC_0_EN, set ADC_ITF_EN */
	val = AUADC_ADC_ITF_EN;
	auadc_write(cfg, AUADC_AUDPDM_ITF_OFFSET, val);

	/* 3. Set input source in PDM_DAC_0 */
	if (cfg->input_mode == AUADC_INPUT_MODE_ADC) {
		/* ADC_0_SRC = 0 (analog) */
		auadc_clr_bits(cfg, AUADC_PDM_DAC_0_OFFSET, AUADC_ADC_0_SRC);
	} else {
		/* ADC_0_SRC = 1 (PDM) */
		auadc_set_bits(cfg, AUADC_PDM_DAC_0_OFFSET, AUADC_ADC_0_SRC);
	}

	/* 4. If PDM mode: enable PDM and select L/R channel */
	if (cfg->input_mode != AUADC_INPUT_MODE_ADC) {
		uint32_t pdm_sel;

		pdm_sel = (cfg->input_mode == AUADC_INPUT_MODE_PDM_LEFT) ? 0U : 1U;

		val = AUADC_PDM_0_EN;
		val |= (pdm_sel << AUADC_ADC_0_PDM_SEL_SHIFT) & AUADC_ADC_0_PDM_SEL_MASK;
		auadc_write(cfg, AUADC_PDM_PDM_0_OFFSET, val);
	} else {
		auadc_write(cfg, AUADC_PDM_PDM_0_OFFSET, 0U);
	}

	/* 5. AUDADC_CMD: OSR selection based on rate and mode.
	 *    For rates <= 16kHz use OSR=1 (high OSR), for higher rates use OSR=0.
	 *    PDM mode always uses OSR=0.
	 */
	val = auadc_read(cfg, AUADC_AUDADC_CMD_OFFSET);
	val &= ~AUADC_AUDADC_AUDIO_OSR_SEL;
	if ((cfg->input_mode == AUADC_INPUT_MODE_ADC) && (hw_rate <= AUADC_RATE_16K)) {
		val |= AUADC_AUDADC_AUDIO_OSR_SEL;
	}
	auadc_write(cfg, AUADC_AUDADC_CMD_OFFSET, val);

	/* 6. RX FIFO control: set data resolution, trigger level, disable DRQ + ints, flush */
	val = AUADC_RX_FIFO_FLUSH;
	val |= (data_res << AUADC_RX_DATA_RES_SHIFT) & AUADC_RX_DATA_RES_MASK;
	val |= (AUADC_RX_TRG_LEVEL_DEFAULT << AUADC_RX_TRG_LEVEL_SHIFT)
		& AUADC_RX_TRG_LEVEL_MASK;
	auadc_write(cfg, AUADC_AUDADC_RX_FIFO_CTRL_OFFSET, val);

	/* Clear the flush bit */
	auadc_clr_bits(cfg, AUADC_AUDADC_RX_FIFO_CTRL_OFFSET, AUADC_RX_FIFO_FLUSH);

	/* 7. Enable ADC_0 in AUDPDM_ITF */
	auadc_set_bits(cfg, AUADC_AUDPDM_ITF_OFFSET, AUADC_ADC_0_EN);

	/* 8. If analog ADC mode: configure analog front-end */
	if (cfg->input_mode == AUADC_INPUT_MODE_ADC) {
		/* Set analog config registers to SDK defaults */
		auadc_write(cfg, AUADC_AUDADC_ANA_CFG1_OFFSET, AUADC_ANA_CFG1_DEFAULT);
		auadc_write(cfg, AUADC_AUDADC_ANA_CFG2_OFFSET, AUADC_ANA_CFG2_DEFAULT);

		/* Power up PGA + SDM, set channel, gain, mode */
		val = auadc_read(cfg, AUADC_AUDADC_CMD_OFFSET);
		val |= AUADC_AUDADC_PGA_PU | AUADC_AUDADC_SDM_PU | AUADC_AUDADC_CONV;
		val &= ~AUADC_AUDADC_PGA_GAIN_MASK;
		val |= (AUADC_PGA_GAIN_DEFAULT << AUADC_AUDADC_PGA_GAIN_SHIFT)
			& AUADC_AUDADC_PGA_GAIN_MASK;
		val &= ~AUADC_AUDADC_PGA_MODE_MASK;
		val |= (AUADC_PGA_MODE_AC << AUADC_AUDADC_PGA_MODE_SHIFT)
			& AUADC_AUDADC_PGA_MODE_MASK;
		val &= ~AUADC_AUDADC_CHANNEL_SELP_MASK;
		val |= (AUADC_CHANNEL_SELP_AUDIO << AUADC_AUDADC_CHANNEL_SELP_SHIFT)
			& AUADC_AUDADC_CHANNEL_SELP_MASK;
		val &= ~AUADC_AUDADC_CHANNEL_SELN_MASK;
		val |= (AUADC_CHANNEL_SELN_GND << AUADC_AUDADC_CHANNEL_SELN_SHIFT)
			& AUADC_AUDADC_CHANNEL_SELN_MASK;
		val &= ~AUADC_AUDADC_CHANNEL_EN_MASK;
		val |= (AUADC_CHANNEL_EN_POS << AUADC_AUDADC_CHANNEL_EN_SHIFT)
			& AUADC_AUDADC_CHANNEL_EN_MASK;
		auadc_write(cfg, AUADC_AUDADC_CMD_OFFSET, val);
	}
}

/* DMA callback -- runs in ISR context */
static void dmic_bflb_auadc_dma_callback(const struct device *dma_dev, void *user_data,
					   uint32_t channel, int status)
{
	const struct device *dev = user_data;
	const struct dmic_bflb_auadc_cfg *cfg = dev->config;
	struct dmic_bflb_auadc_data *data = dev->data;
	void *completed_block;
	int ret;

	if (status < 0) {
		LOG_ERR("DMA error: %d", status);
		data->state = DMIC_STATE_ERROR;
		goto disable;
	}

	if (data->state != DMIC_STATE_ACTIVE) {
		goto disable;
	}

	/* Save the completed block */
	completed_block = data->active_block;

	/* Allocate next buffer for DMA */
	ret = k_mem_slab_alloc(data->mem_slab, &data->active_block, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("Failed to allocate RX buffer");
		data->state = DMIC_STATE_ERROR;
		data->active_block = NULL;
		goto disable;
	}

	/* Reload DMA with the new buffer */
	ret = dma_reload(dma_dev, channel,
			 (uint32_t)(cfg->base + AUADC_AUDADC_RX_FIFO_DATA_OFFSET),
			 (uint32_t)data->active_block,
			 data->block_size);
	if (ret < 0) {
		LOG_ERR("Failed to reload DMA: %d", ret);
		k_mem_slab_free(data->mem_slab, data->active_block);
		data->active_block = NULL;
		data->state = DMIC_STATE_ERROR;
		goto disable;
	}

	ret = dma_start(dma_dev, channel);
	if (ret < 0) {
		LOG_ERR("Failed to restart DMA: %d", ret);
		k_mem_slab_free(data->mem_slab, data->active_block);
		data->active_block = NULL;
		data->state = DMIC_STATE_ERROR;
		goto disable;
	}

	/* Invalidate cache for the completed buffer */
	sys_cache_data_invd_range(completed_block, data->block_size);

	/* Enqueue completed buffer for the application */
	ret = k_msgq_put(&data->rx_queue, &completed_block, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("RX queue full, dropping buffer");
		k_mem_slab_free(data->mem_slab, completed_block);
	}

	return;

disable:
	dma_stop(dma_dev, channel);
	auadc_clr_bits(cfg, AUADC_AUDADC_RX_FIFO_CTRL_OFFSET,
		       AUADC_RX_CH_EN | AUADC_RX_DRQ_EN);
	if (data->active_block != NULL) {
		k_mem_slab_free(data->mem_slab, data->active_block);
		data->active_block = NULL;
	}
}

static int dmic_bflb_auadc_configure(const struct device *dev, struct dmic_cfg *config)
{
	const struct dmic_bflb_auadc_cfg *cfg = dev->config;
	struct dmic_bflb_auadc_data *data = dev->data;
	struct pcm_stream_cfg *stream;
	uint32_t hw_rate;
	uint32_t data_res;
	uint8_t pcm_width;
	unsigned int key;
	int ret;

	key = irq_lock();

	if (data->state == DMIC_STATE_ACTIVE) {
		LOG_ERR("Cannot configure while active");
		irq_unlock(key);
		return -EBUSY;
	}

	irq_unlock(key);

	if (config->channel.req_num_streams < 1U) {
		LOG_ERR("At least one stream required");
		return -EINVAL;
	}

	stream = &config->streams[0];

	/* If rate or width is 0, disable the stream */
	if (stream->pcm_rate == 0U || stream->pcm_width == 0U) {
		key = irq_lock();
		data->state = DMIC_STATE_INITIALIZED;
		irq_unlock(key);
		return 0;
	}

	ret = auadc_rate_to_hw(stream->pcm_rate, &hw_rate);
	if (ret < 0) {
		LOG_ERR("Unsupported sample rate: %u", stream->pcm_rate);
		return -EINVAL;
	}

	pcm_width = stream->pcm_width;
	ret = auadc_width_to_res(pcm_width, &data_res);
	if (ret < 0) {
		LOG_ERR("Unsupported PCM width: %u", pcm_width);
		return -EINVAL;
	}

	if (stream->mem_slab == NULL) {
		LOG_ERR("mem_slab is required");
		return -EINVAL;
	}

	if (stream->block_size == 0U) {
		LOG_ERR("block_size must be non-zero");
		return -EINVAL;
	}

	/* Fill in actual channel info -- this is a mono device */
	config->channel.act_num_chan = 1U;
	config->channel.act_num_streams = 1U;
	config->channel.act_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	config->channel.act_chan_map_hi = 0U;

	/* Enable the audio peripheral clock */
	auadc_enable_clock();

	/* Configure AUADC hardware */
	auadc_hw_configure(cfg, hw_rate, data_res);

	/* Prepare DMA config (stored, applied on START) */
	{
		uint8_t dma_w = auadc_dma_width(pcm_width);

		(void)memset(&data->dma_cfg, 0, sizeof(data->dma_cfg));
		data->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		data->dma_cfg.source_data_size = dma_w;
		data->dma_cfg.dest_data_size = dma_w;
		data->dma_cfg.source_burst_length = 4U;
		data->dma_cfg.dest_burst_length = 4U;
		data->dma_cfg.dma_slot = cfg->dma_trigger;
		data->dma_cfg.block_count = 1U;
		data->dma_cfg.dma_callback = dmic_bflb_auadc_dma_callback;
		data->dma_cfg.user_data = (void *)dev;
	}

	/* Store stream config */
	data->mem_slab = stream->mem_slab;
	data->block_size = stream->block_size;
	data->pcm_rate = stream->pcm_rate;

	/* Purge any stale buffers from the RX queue */
	{
		void *stale;

		while (k_msgq_get(&data->rx_queue, &stale, K_NO_WAIT) == 0) {
			k_mem_slab_free(data->mem_slab, stale);
		}
	}

	key = irq_lock();
	data->state = DMIC_STATE_CONFIGURED;
	irq_unlock(key);

	LOG_DBG("Configured: rate=%u width=%u block_size=%u mode=%u",
		stream->pcm_rate, pcm_width, stream->block_size, cfg->input_mode);

	return 0;
}

static int dmic_bflb_auadc_start(const struct device *dev)
{
	const struct dmic_bflb_auadc_cfg *cfg = dev->config;
	struct dmic_bflb_auadc_data *data = dev->data;
	struct dma_block_config blk_cfg;
	int ret;

	/* Allocate the initial DMA target buffer */
	ret = k_mem_slab_alloc(data->mem_slab, &data->active_block, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("Failed to allocate initial RX buffer");
		return -ENOMEM;
	}

	/* Flush the RX FIFO before starting */
	auadc_set_bits(cfg, AUADC_AUDADC_RX_FIFO_CTRL_OFFSET, AUADC_RX_FIFO_FLUSH);
	auadc_clr_bits(cfg, AUADC_AUDADC_RX_FIFO_CTRL_OFFSET, AUADC_RX_FIFO_FLUSH);

	/* Configure DMA block */
	(void)memset(&blk_cfg, 0, sizeof(blk_cfg));
	blk_cfg.source_address = cfg->base + AUADC_AUDADC_RX_FIFO_DATA_OFFSET;
	blk_cfg.dest_address = (uint32_t)data->active_block;
	blk_cfg.block_size = data->block_size;
	blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;

	data->dma_cfg.head_block = &blk_cfg;

	ret = dma_config(cfg->dma_dev, cfg->dma_channel, &data->dma_cfg);
	if (ret < 0) {
		LOG_ERR("DMA config failed: %d", ret);
		k_mem_slab_free(data->mem_slab, data->active_block);
		data->active_block = NULL;
		return ret;
	}

	ret = dma_start(cfg->dma_dev, cfg->dma_channel);
	if (ret < 0) {
		LOG_ERR("DMA start failed: %d", ret);
		k_mem_slab_free(data->mem_slab, data->active_block);
		data->active_block = NULL;
		return ret;
	}

	/* Enable RX channel and DMA request */
	auadc_set_bits(cfg, AUADC_AUDADC_RX_FIFO_CTRL_OFFSET,
		       AUADC_RX_CH_EN | AUADC_RX_DRQ_EN);

	data->state = DMIC_STATE_ACTIVE;

	LOG_DBG("Started");

	return 0;
}

static int dmic_bflb_auadc_stop(const struct device *dev)
{
	const struct dmic_bflb_auadc_cfg *cfg = dev->config;
	struct dmic_bflb_auadc_data *data = dev->data;
	unsigned int key;

	/* Disable RX channel and DMA request */
	auadc_clr_bits(cfg, AUADC_AUDADC_RX_FIFO_CTRL_OFFSET,
		       AUADC_RX_CH_EN | AUADC_RX_DRQ_EN);

	/* Stop DMA */
	dma_stop(cfg->dma_dev, cfg->dma_channel);

	key = irq_lock();

	/* Free active DMA buffer if present */
	if (data->active_block != NULL) {
		k_mem_slab_free(data->mem_slab, data->active_block);
		data->active_block = NULL;
	}

	data->state = DMIC_STATE_CONFIGURED;

	irq_unlock(key);

	LOG_DBG("Stopped");

	return 0;
}

static int dmic_bflb_auadc_trigger(const struct device *dev, enum dmic_trigger cmd)
{
	struct dmic_bflb_auadc_data *data = dev->data;
	unsigned int key;
	int ret = 0;

	key = irq_lock();

	switch (cmd) {
	case DMIC_TRIGGER_START:
	case DMIC_TRIGGER_RELEASE:
		if (data->state == DMIC_STATE_CONFIGURED ||
		    data->state == DMIC_STATE_PAUSED) {
			irq_unlock(key);
			ret = dmic_bflb_auadc_start(dev);
			return ret;
		}
		if (data->state == DMIC_STATE_ACTIVE) {
			/* Already running */
			irq_unlock(key);
			return 0;
		}
		LOG_ERR("Cannot start: state=%d", data->state);
		irq_unlock(key);
		return -EIO;

	case DMIC_TRIGGER_STOP:
	case DMIC_TRIGGER_PAUSE:
		if (data->state == DMIC_STATE_ACTIVE) {
			irq_unlock(key);
			ret = dmic_bflb_auadc_stop(dev);
			if (ret == 0 && cmd == DMIC_TRIGGER_PAUSE) {
				key = irq_lock();
				data->state = DMIC_STATE_PAUSED;
				irq_unlock(key);
			}
			return ret;
		}
		/* Not active, nothing to do */
		irq_unlock(key);
		return 0;

	case DMIC_TRIGGER_RESET:
		if (data->state == DMIC_STATE_ACTIVE) {
			irq_unlock(key);
			(void)dmic_bflb_auadc_stop(dev);
			key = irq_lock();
		}
		/* Drain any queued buffers */
		{
			void *buf;

			while (k_msgq_get(&data->rx_queue, &buf, K_NO_WAIT) == 0) {
				k_mem_slab_free(data->mem_slab, buf);
			}
		}
		data->state = DMIC_STATE_INITIALIZED;
		irq_unlock(key);
		return 0;

	default:
		LOG_ERR("Invalid trigger command: %d", cmd);
		irq_unlock(key);
		return -EINVAL;
	}
}

static int dmic_bflb_auadc_read(const struct device *dev, uint8_t stream,
				  void **buffer, size_t *size, int32_t timeout)
{
	struct dmic_bflb_auadc_data *data = dev->data;
	int ret;

	ARG_UNUSED(stream);

	if (data->state != DMIC_STATE_ACTIVE &&
	    data->state != DMIC_STATE_CONFIGURED) {
		LOG_ERR("Device not ready for read");
		return -EIO;
	}

	ret = k_msgq_get(&data->rx_queue, buffer, SYS_TIMEOUT_MS(timeout));
	if (ret != 0) {
		LOG_DBG("No audio data available (timeout=%d)", timeout);
		return ret;
	}

	*size = data->block_size;

	return 0;
}

static const struct _dmic_ops dmic_bflb_auadc_api = {
	.configure = dmic_bflb_auadc_configure,
	.trigger = dmic_bflb_auadc_trigger,
	.read = dmic_bflb_auadc_read,
};

static int dmic_bflb_auadc_init(const struct device *dev)
{
	const struct dmic_bflb_auadc_cfg *cfg = dev->config;
	struct dmic_bflb_auadc_data *data = dev->data;

	if (!device_is_ready(cfg->dma_dev)) {
		LOG_ERR("DMA device not ready");
		return -ENODEV;
	}

	data->state = DMIC_STATE_INITIALIZED;
	data->active_block = NULL;

	LOG_DBG("AUADC initialized (base=0x%08x, mode=%u)", cfg->base, cfg->input_mode);

	return 0;
}

/* Map DT input-mode string to numeric constant */
#define AUADC_INPUT_MODE_FROM_DT(inst)						\
	COND_CODE_1(DT_INST_ENUM_HAS_VALUE(inst, input_mode, pdm_left),	\
		(AUADC_INPUT_MODE_PDM_LEFT),					\
		(COND_CODE_1(DT_INST_ENUM_HAS_VALUE(inst, input_mode, pdm_right), \
			(AUADC_INPUT_MODE_PDM_RIGHT),				\
			(AUADC_INPUT_MODE_ADC))))

#ifndef CONFIG_DMIC_BFLB_AUADC_QUEUE_SIZE
#define CONFIG_DMIC_BFLB_AUADC_QUEUE_SIZE 4
#endif

#define DMIC_BFLB_AUADC_INIT(inst)						\
									\
	static void *dmic_bflb_auadc_rx_msgs_##inst				\
		[CONFIG_DMIC_BFLB_AUADC_QUEUE_SIZE];				\
									\
	static const struct dmic_bflb_auadc_cfg dmic_bflb_auadc_cfg_##inst = {	\
		.base = DT_INST_REG_ADDR(inst),					\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, rx)),	\
		.dma_channel = DT_INST_DMAS_CELL_BY_NAME(inst, rx, channel),	\
		.dma_trigger = DT_INST_DMAS_CELL_BY_NAME(inst, rx, trigsrc),	\
		.input_mode = AUADC_INPUT_MODE_FROM_DT(inst),			\
	};									\
									\
	static struct dmic_bflb_auadc_data dmic_bflb_auadc_data_##inst;		\
									\
	static int dmic_bflb_auadc_init_##inst(const struct device *dev)	\
	{									\
		struct dmic_bflb_auadc_data *data = dev->data;			\
									\
		k_msgq_init(&data->rx_queue,					\
			    (char *)dmic_bflb_auadc_rx_msgs_##inst,		\
			    sizeof(void *),					\
			    ARRAY_SIZE(dmic_bflb_auadc_rx_msgs_##inst));	\
									\
		return dmic_bflb_auadc_init(dev);				\
	}									\
									\
	DEVICE_DT_INST_DEFINE(inst,						\
			      dmic_bflb_auadc_init_##inst,			\
			      NULL,						\
			      &dmic_bflb_auadc_data_##inst,			\
			      &dmic_bflb_auadc_cfg_##inst,			\
			      POST_KERNEL,					\
			      CONFIG_AUDIO_DMIC_INIT_PRIORITY,			\
			      &dmic_bflb_auadc_api);

DT_INST_FOREACH_STATUS_OKAY(DMIC_BFLB_AUADC_INIT)
