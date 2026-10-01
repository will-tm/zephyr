/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_camfront

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/video.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/video/video.h>

#include <mm_misc_reg.h>

#include "video_common.h"

LOG_MODULE_REGISTER(video_bflb_camfront, CONFIG_VIDEO_LOG_LEVEL);

/* The parallel bus of the front end is eight bits wide */
#define CAMFRONT_BUS_WIDTH 8

/* Time after a format change for the sensor to send pixel clock edges in the new mode */
#define CAMFRONT_JPEG_SYNC_MS 250

/* Time for a running sensor to send pixel clock edges, a frame at the slowest rate */
#define CAMFRONT_JPEG_RESYNC_MS 100

/* Pin state taking the frame sync pin away from the front end, for a compressed stream */
#define PINCTRL_STATE_COMPRESSED PINCTRL_STATE_PRIV_START

struct video_bflb_camfront_config {
	uintptr_t base;
	const struct pinctrl_dev_config *pcfg;
	const struct device *source_dev;
	const struct device *clock_dev;
	clock_control_subsys_t clock_core;
	clock_control_subsys_t clock_ref;
	uint32_t mclk_frequency;
	uint32_t pclk_frequency;
	uint8_t hsync_active;
	uint8_t vsync_active;
};

struct video_bflb_camfront_data {
	struct video_format fmt;
	k_timepoint_t fmt_settled;
};

/*
 * Pixel clock of the bus, from the pixel rate the sensor reports, or from the devicetree for a
 * sensor that does not. A compressed stream is clocked out two bytes per pixel like a raw one.
 */
static uint32_t video_bflb_camfront_pclk(const struct device *dev)
{
	const struct video_bflb_camfront_config *cfg = dev->config;
	struct video_bflb_camfront_data *data = dev->data;
	struct video_control ctrl = {.id = VIDEO_CID_PIXEL_RATE};
	uint8_t bpp = video_bits_per_pixel(data->fmt.pixelformat);

	if (video_get_ctrl(cfg->source_dev, &ctrl) < 0 || ctrl.val64 <= 0) {
		return cfg->pclk_frequency;
	}

	return ctrl.val64 * (bpp != 0 ? bpp : 16) / CAMFRONT_BUS_WIDTH;
}

/* Number of pixels buffered by the front end before it drains to the capture interface */
static uint32_t video_bflb_camfront_fifo_threshold(const struct device *dev)
{
	const struct video_bflb_camfront_config *cfg = dev->config;
	struct video_bflb_camfront_data *data = dev->data;
	uint32_t width = data->fmt.width;
	uint32_t pclk = video_bflb_camfront_pclk(dev);
	uint32_t threshold;
	uint32_t core_rate;

	if (clock_control_get_rate(cfg->clock_dev, cfg->clock_core, &core_rate) < 0 ||
	    core_rate == 0) {
		LOG_ERR("Cannot read the capture clock rate");
		return 2;
	}

	threshold = width - (width * (pclk / MHZ(1)) / (core_rate / MHZ(1))) / 2 + 10;

	if (threshold > (width - 1)) {
		threshold = width - 1;
	}

	return CLAMP(threshold, 2, 1024);
}

/*
 * Set the frame sync input inversion. The syncs reach the capture interface active high, and a
 * compressed stream, whose frame sync pin is taken away and reads low, is started by inverting it.
 */
static void video_bflb_camfront_frame_sync(const struct device *dev, bool active)
{
	const struct video_bflb_camfront_config *cfg = dev->config;
	struct video_bflb_camfront_data *data = dev->data;
	uint32_t regval = sys_read32(cfg->base + MM_MISC_CONFIG_OFFSET);
	bool invert;

	if (data->fmt.pitch == 0) {
		invert = active;
	} else {
		invert = (cfg->vsync_active == 0);
	}

	if (invert) {
		regval |= MM_MISC_RG_DVPAS_VS_INV_MSK;
	} else {
		regval &= ~MM_MISC_RG_DVPAS_VS_INV_MSK;
	}
	sys_write32(regval, cfg->base + MM_MISC_CONFIG_OFFSET);
}

static void video_bflb_camfront_configure(const struct device *dev)
{
	const struct video_bflb_camfront_config *cfg = dev->config;
	uint32_t regval;

	regval = sys_read32(cfg->base + MM_MISC_CONFIG_OFFSET);
	regval &= ~(MM_MISC_RG_DVPAS_FIFO_TH_MSK | MM_MISC_RG_DVPAS_HS_INV_MSK);
	regval |= video_bflb_camfront_fifo_threshold(dev) << MM_MISC_RG_DVPAS_FIFO_TH_POS;
	if (cfg->hsync_active == 0) {
		regval |= MM_MISC_RG_DVPAS_HS_INV_MSK;
	}
	regval |= MM_MISC_RG_DVPAS_ENABLE_MSK;
	sys_write32(regval, cfg->base + MM_MISC_CONFIG_OFFSET);

	video_bflb_camfront_frame_sync(dev, false);

	/* Feed the capture interface straight from the front end */
	sys_write32(0, cfg->base + MM_MISC_DVP2BUS_SRC_SEL_1_OFFSET);
}

static int video_bflb_camfront_set_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct video_bflb_camfront_config *cfg = dev->config;
	struct video_bflb_camfront_data *data = dev->data;
	const struct pinctrl_state *state;
	int ret;

	if (fmt->pixelformat == VIDEO_PIX_FMT_JPEG &&
	    pinctrl_lookup_state(cfg->pcfg, PINCTRL_STATE_COMPRESSED, &state) < 0) {
		LOG_ERR("A compressed stream needs the \"compressed\" pin state");
		return -ENOTSUP;
	}

	ret = video_set_format(cfg->source_dev, fmt);
	if (ret < 0) {
		return ret;
	}

	data->fmt = *fmt;
	data->fmt_settled = sys_timepoint_calc(K_MSEC(CAMFRONT_JPEG_SYNC_MS));

	/* The pins are switched ahead of any capture, so that no frame sees the swap */
	ret = pinctrl_apply_state(cfg->pcfg, fmt->pitch == 0 ? PINCTRL_STATE_COMPRESSED
							     : PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	video_bflb_camfront_configure(dev);

	return 0;
}

static int video_bflb_camfront_get_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct video_bflb_camfront_config *cfg = dev->config;

	return video_get_format(cfg->source_dev, fmt);
}

static int video_bflb_camfront_get_caps(const struct device *dev, struct video_caps *caps)
{
	const struct video_bflb_camfront_config *cfg = dev->config;

	return video_get_caps(cfg->source_dev, caps);
}

static int video_bflb_camfront_set_stream(const struct device *dev, bool enable,
					  enum video_buf_type type)
{
	const struct video_bflb_camfront_config *cfg = dev->config;
	struct video_bflb_camfront_data *data = dev->data;
	k_timepoint_t resync;
	int ret;

	if (!enable) {
		ret = video_stream_stop(cfg->source_dev, type);
		video_bflb_camfront_frame_sync(dev, false);
		return ret;
	}

	ret = video_stream_start(cfg->source_dev, type);
	if (ret < 0) {
		return ret;
	}

	/* A compressed stream starts once the front end has latched the inactive frame sync */
	if (data->fmt.pitch == 0) {
		resync = sys_timepoint_calc(K_MSEC(CAMFRONT_JPEG_RESYNC_MS));

		while (!sys_timepoint_expired(data->fmt_settled) ||
		       !sys_timepoint_expired(resync)) {
			k_sleep(K_MSEC(1));
		}

		video_bflb_camfront_frame_sync(dev, true);
	}

	return 0;
}

static int video_bflb_camfront_set_frmival(const struct device *dev,
					   struct video_frmival *frmival)
{
	const struct video_bflb_camfront_config *cfg = dev->config;

	return video_set_frmival(cfg->source_dev, frmival);
}

static int video_bflb_camfront_get_frmival(const struct device *dev,
					   struct video_frmival *frmival)
{
	const struct video_bflb_camfront_config *cfg = dev->config;

	return video_get_frmival(cfg->source_dev, frmival);
}

static int video_bflb_camfront_enum_frmival(const struct device *dev,
					    struct video_frmival_enum *fie)
{
	const struct video_bflb_camfront_config *cfg = dev->config;

	return video_enum_frmival(cfg->source_dev, fie);
}

static DEVICE_API(video, video_bflb_camfront_driver_api) = {
	.set_format = video_bflb_camfront_set_fmt,
	.get_format = video_bflb_camfront_get_fmt,
	.get_caps = video_bflb_camfront_get_caps,
	.set_stream = video_bflb_camfront_set_stream,
	.set_frmival = video_bflb_camfront_set_frmival,
	.get_frmival = video_bflb_camfront_get_frmival,
	.enum_frmival = video_bflb_camfront_enum_frmival,
};

#define SOURCE_DEV(n) DEVICE_DT_GET(DT_INST_PHANDLE(n, source))

#define CAMFRONT_ENDPOINT_IN(n) DT_INST_ENDPOINT_BY_ID(n, 0, 0)

/*
 * The front end reads eight data lines on the rising edge of the pixel clock and cannot be told
 * otherwise, so a bus described as anything else would be captured wrongly rather than not at all.
 */
#define VIDEO_BFLB_CAMFRONT_CHECK_BUS(n)                                                           \
	BUILD_ASSERT(DT_PROP_OR(CAMFRONT_ENDPOINT_IN(n), bus_width, CAMFRONT_BUS_WIDTH) ==         \
			     CAMFRONT_BUS_WIDTH,                                                   \
		     "bflb,camfront only reads an eight bit parallel bus");                        \
	BUILD_ASSERT(DT_PROP_OR(CAMFRONT_ENDPOINT_IN(n), pclk_sample, 1) == 1,                     \
		     "bflb,camfront only samples on the rising pixel clock edge");                 \
	BUILD_ASSERT(DT_PROP_OR(CAMFRONT_ENDPOINT_IN(n), data_shift, 0) == 0,                      \
		     "bflb,camfront reads the data lines unshifted");

#define VIDEO_BFLB_CAMFRONT_INIT(n)                                                                \
	VIDEO_BFLB_CAMFRONT_CHECK_BUS(n)                                                           \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static const struct video_bflb_camfront_config video_bflb_camfront_config_##n = {          \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.source_dev = SOURCE_DEV(n),                                                       \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_core = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, core, id),    \
		.clock_ref = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, ref, id),      \
		.mclk_frequency = DT_INST_PROP(n, mclk_frequency),                                 \
		.pclk_frequency = DT_INST_PROP(n, pclk_frequency),                                 \
		.hsync_active = DT_PROP_OR(CAMFRONT_ENDPOINT_IN(n), hsync_active, 1),              \
		.vsync_active = DT_PROP_OR(CAMFRONT_ENDPOINT_IN(n), vsync_active, 1),              \
	};                                                                                         \
                                                                                                   \
	static struct video_bflb_camfront_data video_bflb_camfront_data_##n;                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, &video_bflb_camfront_data_##n,                        \
			      &video_bflb_camfront_config_##n, POST_KERNEL,                        \
			      CONFIG_VIDEO_INIT_PRIORITY, &video_bflb_camfront_driver_api);        \
                                                                                                   \
	VIDEO_DEVICE_DEFINE(video_bflb_camfront_##n, DEVICE_DT_INST_GET(n), SOURCE_DEV(n));

DT_INST_FOREACH_STATUS_OKAY(VIDEO_BFLB_CAMFRONT_INIT)

/*
 * The sensor is clocked by CAM_REF on a chip clock output pin and needs it before its own
 * initialization can talk to it over I2C, so the clock is requested and the pins are muxed
 * ahead of every video device.
 */
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
static int video_bflb_camfront_clock_init(void)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	const struct video_bflb_camfront_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("Clock controller is not ready");
		return -ENODEV;
	}

	ret = clock_control_set_rate(cfg->clock_dev, cfg->clock_ref,
				     (clock_control_subsys_rate_t)(uintptr_t)cfg->mclk_frequency);
	if (ret < 0) {
		LOG_ERR("Cannot produce a %u Hz sensor reference clock (%d)", cfg->mclk_frequency,
			ret);
		return ret;
	}

	return pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
}

SYS_INIT(video_bflb_camfront_clock_init, POST_KERNEL,
	 CONFIG_VIDEO_BFLB_CAMFRONT_CLOCK_INIT_PRIORITY);
#endif
