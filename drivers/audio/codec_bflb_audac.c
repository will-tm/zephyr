/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_audac

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/audio/codec.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>

#include <soc.h>
#include <bflb_soc.h>
#include <glb_reg.h>
#include <bouffalolab/common/audac_reg.h>

LOG_MODULE_REGISTER(codec_bflb_audac, CONFIG_AUDIO_CODEC_LOG_LEVEL);

/* Audio clock enable bit in GLB_AUDIO_CFG0 */
#define GLB_AUDIO_CLK_EN BIT(15)

/* Output mode enumeration */
#define AUDAC_OUTPUT_PWM      0
#define AUDAC_OUTPUT_GPDAC_A  1
#define AUDAC_OUTPUT_GPDAC_B  2
#define AUDAC_OUTPUT_GPDAC_AB 3

/* AU_PWM_MODE sample rate codes (PWM mode: direct, GPDAC mode: +8) */
#define AUDAC_RATE_CODE_8K     0
#define AUDAC_RATE_CODE_16K    1
#define AUDAC_RATE_CODE_32K    2
#define AUDAC_RATE_CODE_24K    3
#define AUDAC_RATE_CODE_48K    4
#define AUDAC_RATE_CODE_22P05K 5
#define AUDAC_RATE_CODE_44P1K  6

/* GPDAC mode offset added to rate code */
#define AUDAC_GPDAC_MODE_OFFSET 8

/* Default volume: 0dB = 0x100 (9-bit, 0.5dB steps, center) */
#define AUDAC_DEFAULT_VOLUME 0x100

/* FIFO trigger level */
#define AUDAC_DEFAULT_TRG_LEVEL 4

/* Zero-detect time default */
#define AUDAC_DEFAULT_ZD_TIME 512

/* AUDAC_1 defaults: DSM_SCALING_MODE=3, DSM_ORDER=1, MIX_SEL=0 */
#define AUDAC_1_DEFAULT \
	((3U << AUDAC_DAC_DSM_SCALING_MODE_SHIFT) | \
	 (1U << AUDAC_DAC_DSM_ORDER_SHIFT) | \
	 (0U << AUDAC_DAC_MIX_SEL_SHIFT))

struct codec_bflb_audac_cfg {
	uint32_t base;
	uint32_t glb_base;
	const struct pinctrl_dev_config *pcfg;
	uint8_t output_mode;
};

struct codec_bflb_audac_data {
	bool playing;
	uint8_t sample_rate_code;
	uint8_t data_mode;
	int32_t volume;
	bool muted;
};

static inline uint32_t audac_read(const struct codec_bflb_audac_cfg *cfg,
				  uint32_t offset)
{
	return sys_read32(cfg->base + offset);
}

static inline void audac_write(const struct codec_bflb_audac_cfg *cfg,
			       uint32_t offset, uint32_t val)
{
	sys_write32(val, cfg->base + offset);
}

static inline uint32_t glb_read(const struct codec_bflb_audac_cfg *cfg,
				uint32_t offset)
{
	return sys_read32(cfg->glb_base + offset);
}

static inline void glb_write(const struct codec_bflb_audac_cfg *cfg,
			     uint32_t offset, uint32_t val)
{
	sys_write32(val, cfg->glb_base + offset);
}

static int audac_rate_to_code(audio_pcm_rate_t rate, uint8_t *code)
{
	switch (rate) {
	case AUDIO_PCM_RATE_8K:
		*code = AUDAC_RATE_CODE_8K;
		break;
	case AUDIO_PCM_RATE_16K:
		*code = AUDAC_RATE_CODE_16K;
		break;
	case AUDIO_PCM_RATE_32K:
		*code = AUDAC_RATE_CODE_32K;
		break;
	case AUDIO_PCM_RATE_24K:
		*code = AUDAC_RATE_CODE_24K;
		break;
	case AUDIO_PCM_RATE_48K:
		*code = AUDAC_RATE_CODE_48K;
		break;
	case AUDIO_PCM_RATE_22P05K:
		*code = AUDAC_RATE_CODE_22P05K;
		break;
	case AUDIO_PCM_RATE_44P1K:
		*code = AUDAC_RATE_CODE_44P1K;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int audac_width_to_data_mode(audio_pcm_width_t width, uint8_t *mode)
{
	switch (width) {
	case AUDIO_PCM_WIDTH_16_BITS:
		*mode = 3U;
		break;
	case AUDIO_PCM_WIDTH_20_BITS:
		*mode = 2U;
		break;
	case AUDIO_PCM_WIDTH_24_BITS:
		*mode = 1U;
		break;
	case AUDIO_PCM_WIDTH_32_BITS:
		*mode = 0U;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void audac_enable_audio_clock(const struct codec_bflb_audac_cfg *cfg)
{
	uint32_t val;

	val = glb_read(cfg, GLB_AUDIO_CFG0_OFFSET);
	val |= GLB_AUDIO_CLK_EN;
	glb_write(cfg, GLB_AUDIO_CFG0_OFFSET, val);
}

static void audac_setup_gpdac(const struct codec_bflb_audac_cfg *cfg)
{
	uint32_t val;

	/* Reset and enable GPDAC analog blocks */
	val = glb_read(cfg, GLB_DAC_CFG0_OFFSET);
	val |= GLB_GPDACA_RSTN_ANA_MSK | GLB_GPDACB_RSTN_ANA_MSK;
	glb_write(cfg, GLB_DAC_CFG0_OFFSET, val);

	if (cfg->output_mode == AUDAC_OUTPUT_GPDAC_A ||
	    cfg->output_mode == AUDAC_OUTPUT_GPDAC_AB) {
		val = glb_read(cfg, GLB_DAC_CFG1_OFFSET);
		val |= GLB_GPDAC_A_EN_MSK | GLB_GPDAC_IOA_EN_MSK;
		glb_write(cfg, GLB_DAC_CFG1_OFFSET, val);
	}

	if (cfg->output_mode == AUDAC_OUTPUT_GPDAC_B ||
	    cfg->output_mode == AUDAC_OUTPUT_GPDAC_AB) {
		val = glb_read(cfg, GLB_DAC_CFG2_OFFSET);
		val |= GLB_GPDAC_B_EN_MSK | GLB_GPDAC_IOB_EN_MSK;
		glb_write(cfg, GLB_DAC_CFG2_OFFSET, val);
	}
}

static void audac_set_pwm_mode(const struct codec_bflb_audac_cfg *cfg,
				uint8_t rate_code)
{
	uint32_t val;
	uint32_t pwm_mode;

	if (cfg->output_mode == AUDAC_OUTPUT_PWM) {
		pwm_mode = (uint32_t)rate_code;
	} else {
		pwm_mode = (uint32_t)rate_code + AUDAC_GPDAC_MODE_OFFSET;
	}

	val = audac_read(cfg, AUDAC_0_OFFSET);
	val &= ~AUDAC_AU_PWM_MODE_MASK;
	val |= (pwm_mode << AUDAC_AU_PWM_MODE_SHIFT) & AUDAC_AU_PWM_MODE_MASK;
	val |= AUDAC_CKG_ENA | AUDAC_DAC_ITF_EN | AUDAC_DAC_0_EN;
	audac_write(cfg, AUDAC_0_OFFSET, val);
}

static void audac_set_volume_hw(const struct codec_bflb_audac_cfg *cfg,
				int32_t volume)
{
	uint32_t val;
	uint32_t vol_raw;

	/*
	 * Volume is in 0.5dB steps, 9-bit field (0-511).
	 * 0x100 = 0dB. The API provides an integer volume
	 * which we clamp to the 9-bit range.
	 */
	if (volume < 0) {
		vol_raw = 0U;
	} else if (volume > 0x1FF) {
		vol_raw = 0x1FFU;
	} else {
		vol_raw = (uint32_t)volume;
	}

	val = audac_read(cfg, AUDAC_S0_OFFSET);
	val &= ~AUDAC_DAC_S0_VOLUME_MASK;
	val |= (vol_raw << AUDAC_DAC_S0_VOLUME_SHIFT) & AUDAC_DAC_S0_VOLUME_MASK;
	val |= AUDAC_DAC_S0_VOLUME_UPDATE;
	val |= AUDAC_DAC_S0_MUTE_SOFTMODE;
	audac_write(cfg, AUDAC_S0_OFFSET, val);
}

static void audac_set_mute_hw(const struct codec_bflb_audac_cfg *cfg, bool mute)
{
	uint32_t val;

	val = audac_read(cfg, AUDAC_S0_OFFSET);
	if (mute) {
		val |= AUDAC_DAC_S0_MUTE;
	} else {
		val &= ~AUDAC_DAC_S0_MUTE;
	}
	val |= AUDAC_DAC_S0_MUTE_SOFTMODE;
	audac_write(cfg, AUDAC_S0_OFFSET, val);
}

static int codec_bflb_audac_configure(const struct device *dev,
				      struct audio_codec_cfg *cfg)
{
	const struct codec_bflb_audac_cfg *devcfg = dev->config;
	struct codec_bflb_audac_data *data = dev->data;
	uint8_t rate_code;
	uint8_t data_mode;
	uint32_t val;
	int ret;

	if (cfg->dai_type != AUDIO_DAI_TYPE_PCM) {
		LOG_ERR("Only AUDIO_DAI_TYPE_PCM supported");
		return -EINVAL;
	}

	ret = audac_rate_to_code(cfg->dai_cfg.pcm.samplerate, &rate_code);
	if (ret != 0) {
		LOG_ERR("Unsupported sample rate: %d", cfg->dai_cfg.pcm.samplerate);
		return ret;
	}

	ret = audac_width_to_data_mode(cfg->dai_cfg.pcm.pcm_width, &data_mode);
	if (ret != 0) {
		LOG_ERR("Unsupported PCM width: %d", cfg->dai_cfg.pcm.pcm_width);
		return ret;
	}

	data->sample_rate_code = rate_code;
	data->data_mode = data_mode;

	/* Step 1: Enable audio clock */
	audac_enable_audio_clock(devcfg);

	/* Step 2: Set AUDAC_0 - enable DAC, interface, clock gate, sample rate */
	audac_set_pwm_mode(devcfg, rate_code);

	/* Step 3: Set AUDAC_1 - DSM config */
	audac_write(devcfg, AUDAC_1_OFFSET, AUDAC_1_DEFAULT);

	/* Step 4: Set AUDAC_FIFO_CTRL - data mode, trigger level, no DMA yet */
	val = ((uint32_t)data_mode << AUDAC_TX_DATA_MODE_SHIFT) & AUDAC_TX_DATA_MODE_MASK;
	val |= ((uint32_t)AUDAC_DEFAULT_TRG_LEVEL << AUDAC_TX_TRG_LEVEL_SHIFT) &
	       AUDAC_TX_TRG_LEVEL_MASK;
	audac_write(devcfg, AUDAC_FIFO_CTRL_OFFSET, val);

	/* Step 5: Set AUDAC_ZD_0 - zero detect enable + time */
	val = AUDAC_ZD_EN |
	      ((uint32_t)AUDAC_DEFAULT_ZD_TIME & AUDAC_ZD_TIME_MASK);
	audac_write(devcfg, AUDAC_ZD_0_OFFSET, val);

	/* Step 6: Clear interrupt */
	val = audac_read(devcfg, AUDAC_STATUS_OFFSET);
	val |= AUDAC_DAC_S0_INT_CLR;
	audac_write(devcfg, AUDAC_STATUS_OFFSET, val);

	/* Step 7: Configure GPDAC if needed */
	if (devcfg->output_mode != AUDAC_OUTPUT_PWM) {
		audac_setup_gpdac(devcfg);
	}

	/* Step 8: Set initial volume */
	audac_set_volume_hw(devcfg, data->volume);
	audac_set_mute_hw(devcfg, data->muted);

	LOG_DBG("Configured: rate_code=%u data_mode=%u output_mode=%u",
		rate_code, data_mode, devcfg->output_mode);

	return 0;
}

static void codec_bflb_audac_start_output(const struct device *dev)
{
	const struct codec_bflb_audac_cfg *cfg = dev->config;
	struct codec_bflb_audac_data *data = dev->data;
	unsigned int key;
	uint32_t val;

	key = irq_lock();

	/* Enable TX channels (both L+R) and DMA request */
	val = audac_read(cfg, AUDAC_FIFO_CTRL_OFFSET);
	val |= (0x3U << AUDAC_TX_CH_EN_SHIFT);
	val |= AUDAC_TX_DRQ_EN;
	audac_write(cfg, AUDAC_FIFO_CTRL_OFFSET, val);

	data->playing = true;

	irq_unlock(key);

	LOG_DBG("Output started");
}

static void codec_bflb_audac_stop_output(const struct device *dev)
{
	const struct codec_bflb_audac_cfg *cfg = dev->config;
	struct codec_bflb_audac_data *data = dev->data;
	unsigned int key;
	uint32_t val;

	key = irq_lock();

	/* Disable TX channels and DMA request */
	val = audac_read(cfg, AUDAC_FIFO_CTRL_OFFSET);
	val &= ~AUDAC_TX_CH_EN_MASK;
	val &= ~AUDAC_TX_DRQ_EN;
	audac_write(cfg, AUDAC_FIFO_CTRL_OFFSET, val);

	/* Flush FIFO */
	val = audac_read(cfg, AUDAC_FIFO_CTRL_OFFSET);
	val |= AUDAC_TX_FIFO_FLUSH;
	audac_write(cfg, AUDAC_FIFO_CTRL_OFFSET, val);
	val &= ~AUDAC_TX_FIFO_FLUSH;
	audac_write(cfg, AUDAC_FIFO_CTRL_OFFSET, val);

	data->playing = false;

	irq_unlock(key);

	LOG_DBG("Output stopped");
}

static int codec_bflb_audac_set_property(const struct device *dev,
					 audio_property_t property,
					 audio_channel_t channel,
					 audio_property_value_t val)
{
	struct codec_bflb_audac_data *data = dev->data;

	ARG_UNUSED(channel);

	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_VOLUME:
		data->volume = val.vol;
		break;
	case AUDIO_PROPERTY_OUTPUT_MUTE:
		data->muted = val.mute;
		break;
	default:
		LOG_ERR("Unsupported property: %d", property);
		return -EINVAL;
	}

	return 0;
}

static int codec_bflb_audac_apply_properties(const struct device *dev)
{
	const struct codec_bflb_audac_cfg *cfg = dev->config;
	struct codec_bflb_audac_data *data = dev->data;

	audac_set_volume_hw(cfg, data->volume);
	audac_set_mute_hw(cfg, data->muted);

	return 0;
}

static const struct audio_codec_api codec_bflb_audac_api = {
	.configure = codec_bflb_audac_configure,
	.start_output = codec_bflb_audac_start_output,
	.stop_output = codec_bflb_audac_stop_output,
	.set_property = codec_bflb_audac_set_property,
	.apply_properties = codec_bflb_audac_apply_properties,
};

static int codec_bflb_audac_init(const struct device *dev)
{
	const struct codec_bflb_audac_cfg *cfg = dev->config;
	int ret;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("Failed to apply pinctrl: %d", ret);
		return ret;
	}

	LOG_INF("AUDAC initialized (output_mode=%u)", cfg->output_mode);

	return 0;
}

#define AUDAC_OUTPUT_MODE_IDX(inst) \
	DT_INST_ENUM_IDX_OR(inst, output_mode, 0)

#define CODEC_BFLB_AUDAC_DEFINE(inst)                                          \
	PINCTRL_DT_INST_DEFINE(inst);                                          \
                                                                               \
	static const struct codec_bflb_audac_cfg codec_bflb_audac_cfg_##inst = { \
		.base = DT_INST_REG_ADDR(inst),                                \
		.glb_base = GLB_BASE,                                          \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                  \
		.output_mode = AUDAC_OUTPUT_MODE_IDX(inst),                    \
	};                                                                     \
                                                                               \
	static struct codec_bflb_audac_data codec_bflb_audac_data_##inst = {   \
		.playing = false,                                              \
		.sample_rate_code = AUDAC_RATE_CODE_48K,                       \
		.data_mode = 3U,                                               \
		.volume = AUDAC_DEFAULT_VOLUME,                                \
		.muted = false,                                                \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(inst, codec_bflb_audac_init, NULL,               \
			      &codec_bflb_audac_data_##inst,                   \
			      &codec_bflb_audac_cfg_##inst,                    \
			      POST_KERNEL, CONFIG_AUDIO_CODEC_INIT_PRIORITY,   \
			      &codec_bflb_audac_api);

DT_INST_FOREACH_STATUS_OKAY(CODEC_BFLB_AUDAC_DEFINE)
