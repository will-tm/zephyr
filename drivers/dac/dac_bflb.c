/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_gpdac

#include <zephyr/device.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <soc.h>

#include <bflb_soc.h>
#include <glb_reg.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dac_bflb, CONFIG_DAC_LOG_LEVEL);

/* Maximum number of DAC channels */
#define BFLB_DAC_NUM_CHANNELS 2U

/* GPIP register offsets (relative to GPIP base, from DT reg[0]) */
#define GPIP_GPDAC_CONFIG_OFFSET     0x40U
#define GPIP_GPDAC_DMA_CONFIG_OFFSET 0x44U

/* GPIP_GPDAC_CONFIG bits */
#define GPIP_GPDAC_EN             BIT(0)
#define GPIP_GPDAC_MODE_SHIFT     8U
#define GPIP_GPDAC_MODE_MASK      (0x7U << GPIP_GPDAC_MODE_SHIFT)
#define GPIP_GPDAC_CH_A_SEL_SHIFT 16U
#define GPIP_GPDAC_CH_A_SEL_MASK  (0xFU << GPIP_GPDAC_CH_A_SEL_SHIFT)
#define GPIP_GPDAC_CH_B_SEL_SHIFT 20U
#define GPIP_GPDAC_CH_B_SEL_MASK  (0xFU << GPIP_GPDAC_CH_B_SEL_SHIFT)

/* Clock divider modes */
#define GPDAC_CLK_DIV_16 0U
#define GPDAC_CLK_DIV_32 1U
#define GPDAC_CLK_DIV_1  4U

/*
 * GLB GPDAC register offsets vary by SoC family.
 * BL70x/BL70xL/BL60x: 0x308/0x30C/0x310/0x314
 * BL61x: 0x120/0x124/0x128/0x12C (already defined as GLB_DAC_CFGx_OFFSET)
 *
 * We use the SoC-specific defines from glb_reg.h.
 */
#if defined(CONFIG_SOC_SERIES_BL70X) || defined(CONFIG_SOC_SERIES_BL70XL) || \
	defined(CONFIG_SOC_SERIES_BL60X)
#define BFLB_GLB_GPDAC_CTRL_OFF  0x308U
#define BFLB_GLB_GPDAC_ACTRL_OFF 0x30CU
#define BFLB_GLB_GPDAC_BCTRL_OFF 0x310U
#define BFLB_GLB_GPDAC_DATA_OFF  0x314U
#define BFLB_DAC_RESOLUTION      10U
#define BFLB_DAC_DATA_MASK       0x3FFU
#elif defined(CONFIG_SOC_SERIES_BL61X)
#define BFLB_GLB_GPDAC_CTRL_OFF  GLB_DAC_CFG0_OFFSET
#define BFLB_GLB_GPDAC_ACTRL_OFF GLB_DAC_CFG1_OFFSET
#define BFLB_GLB_GPDAC_BCTRL_OFF GLB_DAC_CFG2_OFFSET
#define BFLB_GLB_GPDAC_DATA_OFF  0x12CU
#define BFLB_DAC_RESOLUTION      10U
#define BFLB_DAC_DATA_MASK       0x3FFU
#else
#error "Unsupported SoC series for BFLB GPDAC"
#endif

/* Data register channel positions */
#define GPDAC_A_DATA_SHIFT 16U
#define GPDAC_B_DATA_SHIFT 0U

/*
 * GPDAC control register bit definitions.
 * Defined locally because not all SoC HAL headers include them.
 */
#define BFLB_GPDACA_RSTN_ANA BIT(0)
#define BFLB_GPDACB_RSTN_ANA BIT(1)
#define BFLB_GPDAC_REF_SEL   BIT(8)
#define BFLB_GPDAC_A_EN      BIT(0)
#define BFLB_GPDAC_IOA_EN    BIT(1)
#define BFLB_GPDAC_B_EN      BIT(0)
#define BFLB_GPDAC_IOB_EN    BIT(1)

struct dac_bflb_cfg {
	uint32_t gpip_base;
	uint32_t glb_base;
};

struct dac_bflb_data {
	bool ch_enabled[BFLB_DAC_NUM_CHANNELS];
};

static inline uint32_t gpip_read(const struct dac_bflb_cfg *cfg, uint32_t off)
{
	return sys_read32(cfg->gpip_base + off);
}

static inline void gpip_write(const struct dac_bflb_cfg *cfg, uint32_t off,
			      uint32_t val)
{
	sys_write32(val, cfg->gpip_base + off);
}

static inline uint32_t glb_read(const struct dac_bflb_cfg *cfg, uint32_t off)
{
	return sys_read32(cfg->glb_base + off);
}

static inline void glb_write(const struct dac_bflb_cfg *cfg, uint32_t off,
			     uint32_t val)
{
	sys_write32(val, cfg->glb_base + off);
}

static void dac_bflb_reset_analog(const struct dac_bflb_cfg *cfg)
{
	uint32_t val;

	/* Pulse reset for both channels */
	val = glb_read(cfg, BFLB_GLB_GPDAC_CTRL_OFF);
	val &= ~(BFLB_GPDACA_RSTN_ANA | BFLB_GPDACB_RSTN_ANA);
	glb_write(cfg, BFLB_GLB_GPDAC_CTRL_OFF, val);

	/* Brief delay for reset */
	k_busy_wait(1);

	val |= BFLB_GPDACA_RSTN_ANA | BFLB_GPDACB_RSTN_ANA;
	glb_write(cfg, BFLB_GLB_GPDAC_CTRL_OFF, val);
}

static void dac_bflb_set_ref_internal(const struct dac_bflb_cfg *cfg)
{
	uint32_t val;

	val = glb_read(cfg, BFLB_GLB_GPDAC_CTRL_OFF);
	val &= ~BFLB_GPDAC_REF_SEL;
	glb_write(cfg, BFLB_GLB_GPDAC_CTRL_OFF, val);
}

static int dac_bflb_channel_setup(const struct device *dev,
				  const struct dac_channel_cfg *channel_cfg)
{
	const struct dac_bflb_cfg *cfg = dev->config;
	struct dac_bflb_data *data = dev->data;
	uint8_t ch = channel_cfg->channel_id;
	uint32_t val;

	if (ch >= BFLB_DAC_NUM_CHANNELS) {
		LOG_ERR("Invalid channel %u (max %u)", ch,
			BFLB_DAC_NUM_CHANNELS - 1U);
		return -EINVAL;
	}

	if (channel_cfg->resolution != BFLB_DAC_RESOLUTION) {
		LOG_ERR("Only %u-bit resolution supported, got %u",
			BFLB_DAC_RESOLUTION, channel_cfg->resolution);
		return -ENOTSUP;
	}

	/* Enable the channel analog output */
	if (ch == 0U) {
		val = glb_read(cfg, BFLB_GLB_GPDAC_ACTRL_OFF);
		val |= BFLB_GPDAC_A_EN | BFLB_GPDAC_IOA_EN;
		glb_write(cfg, BFLB_GLB_GPDAC_ACTRL_OFF, val);
	} else {
		val = glb_read(cfg, BFLB_GLB_GPDAC_BCTRL_OFF);
		val |= BFLB_GPDAC_B_EN | BFLB_GPDAC_IOB_EN;
		glb_write(cfg, BFLB_GLB_GPDAC_BCTRL_OFF, val);
	}

	data->ch_enabled[ch] = true;

	LOG_DBG("Channel %u configured (%u-bit)", ch, BFLB_DAC_RESOLUTION);

	return 0;
}

static int dac_bflb_write_value(const struct device *dev, uint8_t channel,
				uint32_t value)
{
	const struct dac_bflb_cfg *cfg = dev->config;
	const struct dac_bflb_data *data = dev->data;
	uint32_t val;

	if (channel >= BFLB_DAC_NUM_CHANNELS) {
		return -EINVAL;
	}

	if (!data->ch_enabled[channel]) {
		LOG_ERR("Channel %u not configured", channel);
		return -EINVAL;
	}

	if (value > BFLB_DAC_DATA_MASK) {
		LOG_ERR("Value %u exceeds %u-bit range", value,
			BFLB_DAC_RESOLUTION);
		return -EINVAL;
	}

	val = glb_read(cfg, BFLB_GLB_GPDAC_DATA_OFF);

	if (channel == 0U) {
		val &= ~(BFLB_DAC_DATA_MASK << GPDAC_A_DATA_SHIFT);
		val |= (value & BFLB_DAC_DATA_MASK) << GPDAC_A_DATA_SHIFT;
	} else {
		val &= ~(BFLB_DAC_DATA_MASK << GPDAC_B_DATA_SHIFT);
		val |= (value & BFLB_DAC_DATA_MASK) << GPDAC_B_DATA_SHIFT;
	}

	glb_write(cfg, BFLB_GLB_GPDAC_DATA_OFF, val);

	return 0;
}

static int dac_bflb_init(const struct device *dev)
{
	const struct dac_bflb_cfg *cfg = dev->config;
	uint32_t val;

	/* Reset analog blocks */
	dac_bflb_reset_analog(cfg);

	/* Use internal reference */
	dac_bflb_set_ref_internal(cfg);

	/* Enable DAC engine with default clock divider (DIV_32) */
	val = gpip_read(cfg, GPIP_GPDAC_CONFIG_OFFSET);
	val |= GPIP_GPDAC_EN;
	val &= ~GPIP_GPDAC_MODE_MASK;
	val |= (GPDAC_CLK_DIV_32 << GPIP_GPDAC_MODE_SHIFT)
	       & GPIP_GPDAC_MODE_MASK;
	gpip_write(cfg, GPIP_GPDAC_CONFIG_OFFSET, val);

	/* Disable DMA (direct write mode) */
	val = gpip_read(cfg, GPIP_GPDAC_DMA_CONFIG_OFFSET);
	val &= ~BIT(0);
	gpip_write(cfg, GPIP_GPDAC_DMA_CONFIG_OFFSET, val);

	LOG_INF("GPDAC initialized (gpip=0x%08x)", cfg->gpip_base);

	return 0;
}

static DEVICE_API(dac, dac_bflb_driver_api) = {
	.channel_setup = dac_bflb_channel_setup,
	.write_value = dac_bflb_write_value,
};

#define DAC_BFLB_INIT(inst)						\
								\
	static const struct dac_bflb_cfg dac_bflb_cfg_##inst = {	\
		.gpip_base = DT_INST_REG_ADDR(inst),		\
		.glb_base = GLB_BASE,				\
	};							\
								\
	static struct dac_bflb_data dac_bflb_data_##inst;	\
								\
	DEVICE_DT_INST_DEFINE(inst, dac_bflb_init, NULL,	\
			      &dac_bflb_data_##inst,		\
			      &dac_bflb_cfg_##inst,		\
			      POST_KERNEL,			\
			      CONFIG_DAC_INIT_PRIORITY,		\
			      &dac_bflb_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DAC_BFLB_INIT)
