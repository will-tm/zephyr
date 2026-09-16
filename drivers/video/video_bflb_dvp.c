/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_dvp

#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/video.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/video/formats.h>
#include <zephyr/video/video.h>

#include <bflb_soc.h>
#include <cam_reg.h>
#include <mm_misc_reg.h>

#include "video_common.h"

LOG_MODULE_REGISTER(video_bflb_dvp, CONFIG_VIDEO_LOG_LEVEL);

/* The AXI write port is 64 bits wide and the driver always uses INCR16 bursts */
#define DVP_BURST_BYTES 128U

/*
 * Data mode, which selects how many bytes the controller stores per pixel. The
 * capture is pass-through, so it follows the pixel size of the format. The
 * controller also has modes for three and four byte pixels, left out because
 * there is no sensor here emitting them to check them against.
 */
#define DVP_DATA_MODE_1_OR_2_BYTE 0U

/* The parallel bus of this controller is eight bits wide */
#define DVP_BUS_WIDTH 8

/* XLEN encoding for INCR16 bursts */
#define DVP_BURST_INCR16 3U


/*
 * The controller owns a ring of frames. It keeps filling the ring while the
 * frame it just completed is copied out, and reports which frame that is
 * through its frame FIFO, so the ring needs room for two frames.
 */
#define DVP_RING_FRAMES 2

#ifdef CONFIG_VIDEO_BFLB_DVP_RING_ZEPHYR_REGION
#define DVP_RING_SECTION Z_GENERIC_SECTION(CONFIG_VIDEO_BFLB_DVP_RING_ZEPHYR_REGION_NAME)
#else
#define DVP_RING_SECTION __noinit
#endif

struct video_bflb_dvp_config {
	uint32_t base;
	const struct device *source_dev;
	const struct device *clock_dev;
	clock_control_subsys_t clock_core;
	clock_control_subsys_t clock_ref;
	uint32_t mclk_frequency;
	uint8_t *ring;
	size_t ring_size;
	k_thread_stack_t *stack;
	size_t stack_size;
	uint32_t pclk_frequency;
	uint8_t hsync_active;
	uint8_t vsync_active;
};

struct video_bflb_dvp_data {
	struct video_format fmt;
	uint32_t frame_size;
	uint32_t ring_bytes;
	uint8_t data_mode;
	bool streaming;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	struct k_sem frame_ready;
	struct k_thread thread;
#ifdef CONFIG_POLL
	struct k_poll_signal *signal_out;
#endif
};

static void video_bflb_dvp_raise_signal(struct video_bflb_dvp_data *data, int result)
{
#ifdef CONFIG_POLL
	if (data->signal_out != NULL) {
		k_poll_signal_raise(data->signal_out, result);
	}
#else
	ARG_UNUSED(data);
	ARG_UNUSED(result);
#endif
}

/* The controller stores whole bytes, so only byte sized pixels can pass through */
static int video_bflb_dvp_data_mode(uint32_t pixelformat)
{
	switch (video_bits_per_pixel(pixelformat)) {
	case 8:
	case 16:
		return DVP_DATA_MODE_1_OR_2_BYTE;
	default:
		return -ENOTSUP;
	}
}

/* Number of pixels buffered by the sensor front end before it drains to AXI */
static uint32_t video_bflb_dvp_fifo_threshold(const struct device *dev, uint32_t width)
{
	const struct video_bflb_dvp_config *cfg = dev->config;
	uint32_t threshold;
	uint32_t core_rate;

	if (clock_control_get_rate(cfg->clock_dev, cfg->clock_core, &core_rate) < 0 ||
	    core_rate == 0) {
		LOG_ERR("Cannot read the capture clock rate");
		return 2;
	}

	threshold = width - (width * (cfg->pclk_frequency / MHZ(1)) /
			     (core_rate / MHZ(1))) / 2 + 10;

	if (threshold > (width - 1)) {
		threshold = width - 1;
	}

	return CLAMP(threshold, 2, 1024);
}

static void video_bflb_dvp_configure(const struct device *dev)
{
	const struct video_bflb_dvp_config *cfg = dev->config;
	struct video_bflb_dvp_data *data = dev->data;
	uint32_t width = data->fmt.width;
	uint32_t height = data->fmt.height;
	uint32_t regval;

	/* No cropping: the active window is the whole frame */
	sys_write32(width, cfg->base + CAM_DVP2AXI_HSYNC_CROP_OFFSET);
	sys_write32(height, cfg->base + CAM_DVP2AXI_VSYNC_CROP_OFFSET);
	sys_write32(height << CAM_REG_TOTAL_VCNT_SHIFT | width,
		    cfg->base + CAM_DVP2AXI_FRAM_EXM_OFFSET);
	sys_write32(0, cfg->base + CAM_DVP_DEBUG_OFFSET);

	sys_write32((uint32_t)cfg->ring, cfg->base + CAM_DVP2AXI_ADDR_START_OFFSET);
	sys_write32(data->frame_size, cfg->base + CAM_DVP2AXI_FRAME_BCNT_OFFSET);
	sys_write32(data->ring_bytes / DVP_BURST_BYTES, cfg->base + CAM_DVP2AXI_MEM_BCNT_OFFSET);

	regval = sys_read32(cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	regval &= ~(CAM_REG_DVP_ENABLE | CAM_REG_DROP_EN | CAM_REG_DROP_EVEN |
		    CAM_REG_DVP_DATA_MODE_MASK | CAM_REG_DVP_DATA_BSEL |
		    CAM_REG_V_SUBSAMPLE_EN | CAM_REG_V_SUBSAMPLE_POL | CAM_REG_XLEN_MASK |
		    CAM_REG_FRAM_VLD_POL | CAM_REG_LINE_VLD_POL);
	/* Software mode: the frame addresses are pushed on the frame FIFO */
	regval |= CAM_REG_SW_MODE;
	/* Restart each frame at the buffer start, so the image is not rolled */
	regval |= CAM_REG_HW_MODE_FWRAP;
	regval |= data->data_mode << CAM_REG_DVP_DATA_MODE_SHIFT;
	regval |= DVP_BURST_INCR16 << CAM_REG_XLEN_SHIFT;
	if (cfg->vsync_active != 0) {
		regval |= CAM_REG_FRAM_VLD_POL;
	}
	if (cfg->hsync_active != 0) {
		regval |= CAM_REG_LINE_VLD_POL;
	}
	sys_write32(regval, cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET);

	/* Interrupt on every frame, all error interrupts left masked */
	regval = sys_read32(cfg->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);
	regval &= ~(CAM_REG_FRAME_CNT_TRGR_INT_MASK | CAM_REG_INT_MEM_EN | CAM_REG_INT_FRAME_EN |
		    CAM_REG_INT_FIFO_EN | CAM_REG_INT_HCNT_EN | CAM_REG_INT_VCNT_EN);
	regval |= 1U << CAM_REG_FRAME_CNT_TRGR_INT_SHIFT;
	regval |= CAM_REG_INT_NORMAL_EN;
	sys_write32(regval, cfg->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);

	regval = sys_read32(MM_MISC_BASE + MM_MISC_CONFIG_OFFSET);
	regval &= ~MM_MISC_RG_DVPAS_FIFO_TH_MSK;
	regval |= video_bflb_dvp_fifo_threshold(dev, width) << MM_MISC_RG_DVPAS_FIFO_TH_POS;
	regval |= MM_MISC_RG_DVPAS_ENABLE_MSK;
	sys_write32(regval, MM_MISC_BASE + MM_MISC_CONFIG_OFFSET);

	/* Take the pixel stream straight from the sensor front end */
	sys_write32(0, MM_MISC_BASE + MM_MISC_DVP2BUS_SRC_SEL_1_OFFSET);

	LOG_DBG("%ux%u, %u bytes/frame, config 0x%08x", width, height, data->frame_size,
		sys_read32(cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET));
}

static void video_bflb_dvp_isr(const struct device *dev)
{
	const struct video_bflb_dvp_config *cfg = dev->config;
	struct video_bflb_dvp_data *data = dev->data;

	if ((sys_read32(cfg->base + CAM_DVP_STATUS_AND_ERROR_OFFSET) & CAM_STS_NORMAL_INT) == 0) {
		return;
	}

	sys_write32(CAM_REG_INT_NORMAL_CLR, cfg->base + CAM_DVP_FRAME_FIFO_POP_OFFSET);
	k_sem_give(&data->frame_ready);
}

static void video_bflb_dvp_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct video_bflb_dvp_config *cfg = dev->config;
	struct video_bflb_dvp_data *data = dev->data;
	struct video_buffer *vbuf;
	uint8_t *frame;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_sem_take(&data->frame_ready, K_FOREVER);

		while ((sys_read32(cfg->base + CAM_DVP_STATUS_AND_ERROR_OFFSET) &
			CAM_FRAME_VALID_CNT_MASK) != 0) {
			/* The frame FIFO says which ring frame was completed */
			frame = (uint8_t *)sys_read32(cfg->base + CAM_FRAME_START_ADDR0_OFFSET);

			vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);
			if (vbuf == NULL) {
				video_bflb_dvp_raise_signal(data, VIDEO_BUF_ERROR);
			} else {
				sys_cache_data_invd_range(frame, data->frame_size);
				memcpy(vbuf->buffer, frame, data->frame_size);

				vbuf->bytesused = data->frame_size;
				vbuf->line_offset = 0;
				vbuf->timestamp = k_uptime_get_32();
				k_fifo_put(&data->fifo_out, vbuf);
				video_bflb_dvp_raise_signal(data, VIDEO_BUF_DONE);
			}

			sys_write32(CAM_RFIFO_POP, cfg->base + CAM_DVP_FRAME_FIFO_POP_OFFSET);
		}
	}
}

static int video_bflb_dvp_set_stream(const struct device *dev, bool enable,
				     enum video_buf_type type)
{
	const struct video_bflb_dvp_config *cfg = dev->config;
	struct video_bflb_dvp_data *data = dev->data;
	uint32_t regval;
	int ret;

	if (!enable) {
		if (!data->streaming) {
			return 0;
		}

		regval = sys_read32(cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
		regval &= ~CAM_REG_DVP_ENABLE;
		sys_write32(regval, cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
		data->streaming = false;

		return video_stream_stop(cfg->source_dev, type);
	}

	if (data->streaming) {
		return -EBUSY;
	}

	video_bflb_dvp_configure(dev);
	sys_write32(CAM_REG_INT_NORMAL_CLR | CAM_RFIFO_POP,
		    cfg->base + CAM_DVP_FRAME_FIFO_POP_OFFSET);

	regval = sys_read32(cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	regval |= CAM_REG_DVP_ENABLE;
	sys_write32(regval, cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	data->streaming = true;

	ret = video_stream_start(cfg->source_dev, type);
	if (ret < 0) {
		video_bflb_dvp_set_stream(dev, false, type);
		return ret;
	}

	return 0;
}

static int video_bflb_dvp_get_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct video_bflb_dvp_config *cfg = dev->config;
	int ret;

	ret = video_get_format(cfg->source_dev, fmt);
	if (ret < 0) {
		return ret;
	}

	return video_estimate_fmt_size(fmt);
}

static int video_bflb_dvp_set_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct video_bflb_dvp_config *cfg = dev->config;
	struct video_bflb_dvp_data *data = dev->data;
	uint32_t frame_size;
	int ret;

	if (data->streaming) {
		return -EBUSY;
	}

	ret = video_set_format(cfg->source_dev, fmt);
	if (ret < 0) {
		return ret;
	}

	ret = video_estimate_fmt_size(fmt);
	if (ret < 0) {
		return ret;
	}

	ret = video_bflb_dvp_data_mode(fmt->pixelformat);
	if (ret < 0) {
		LOG_ERR("Format %s cannot be stored by the capture interface",
			VIDEO_FOURCC_TO_STR(fmt->pixelformat));
		return ret;
	}
	data->data_mode = (uint8_t)ret;

	frame_size = fmt->pitch * fmt->height;
	if ((frame_size % DVP_BURST_BYTES) != 0) {
		LOG_ERR("Frame size %u is not a multiple of %u", frame_size, DVP_BURST_BYTES);
		return -EINVAL;
	}

	if ((frame_size * DVP_RING_FRAMES) > cfg->ring_size) {
		LOG_ERR("Capture ring of %zu bytes holds less than %u frames of %u bytes",
			cfg->ring_size, DVP_RING_FRAMES, frame_size);
		return -ENOMEM;
	}

	data->fmt = *fmt;
	data->frame_size = frame_size;
	data->ring_bytes = frame_size * DVP_RING_FRAMES;

	return 0;
}

static int video_bflb_dvp_get_caps(const struct device *dev, struct video_caps *caps)
{
	const struct video_bflb_dvp_config *cfg = dev->config;

	/* One buffer holds the frame being captured, the other is handed to the application */
	caps->min_vbuf_count = 2;

	return video_get_caps(cfg->source_dev, caps);
}

static int video_bflb_dvp_enqueue(const struct device *dev, struct video_buffer *vbuf)
{
	struct video_bflb_dvp_data *data = dev->data;

	if (vbuf->size < data->frame_size) {
		LOG_ERR("Buffer of %u bytes is too small for a %u byte frame", vbuf->size,
			data->frame_size);
		return -EINVAL;
	}

	/* The controller writes through AXI, so no dirty line may be left behind */
	sys_cache_data_flush_and_invd_range(vbuf->buffer, data->frame_size);

	vbuf->bytesused = 0;
	k_fifo_put(&data->fifo_in, vbuf);

	return 0;
}

static int video_bflb_dvp_dequeue(const struct device *dev, struct video_buffer **vbuf,
				  k_timeout_t timeout)
{
	struct video_bflb_dvp_data *data = dev->data;

	*vbuf = k_fifo_get(&data->fifo_out, timeout);
	if (*vbuf == NULL) {
		return -EAGAIN;
	}

	return 0;
}

static int video_bflb_dvp_flush(const struct device *dev, bool cancel)
{
	struct video_bflb_dvp_data *data = dev->data;
	struct video_buffer *vbuf;

	if (!cancel) {
		while (!k_fifo_is_empty(&data->fifo_in)) {
			k_sleep(K_MSEC(1));
		}
		return 0;
	}

	while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT)) != NULL) {
		k_fifo_put(&data->fifo_out, vbuf);
		video_bflb_dvp_raise_signal(data, VIDEO_BUF_ABORTED);
	}

	return 0;
}

static int video_bflb_dvp_set_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct video_bflb_dvp_config *cfg = dev->config;

	return video_set_frmival(cfg->source_dev, frmival);
}

static int video_bflb_dvp_get_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct video_bflb_dvp_config *cfg = dev->config;

	return video_get_frmival(cfg->source_dev, frmival);
}

static int video_bflb_dvp_enum_frmival(const struct device *dev, struct video_frmival_enum *fie)
{
	const struct video_bflb_dvp_config *cfg = dev->config;

	return video_enum_frmival(cfg->source_dev, fie);
}

#ifdef CONFIG_POLL
static int video_bflb_dvp_set_signal(const struct device *dev, struct k_poll_signal *sig)
{
	struct video_bflb_dvp_data *data = dev->data;

	data->signal_out = sig;

	return 0;
}
#endif

static DEVICE_API(video, video_bflb_dvp_driver_api) = {
	.set_format = video_bflb_dvp_set_fmt,
	.get_format = video_bflb_dvp_get_fmt,
	.set_stream = video_bflb_dvp_set_stream,
	.get_caps = video_bflb_dvp_get_caps,
	.enqueue = video_bflb_dvp_enqueue,
	.dequeue = video_bflb_dvp_dequeue,
	.flush = video_bflb_dvp_flush,
	.set_frmival = video_bflb_dvp_set_frmival,
	.get_frmival = video_bflb_dvp_get_frmival,
	.enum_frmival = video_bflb_dvp_enum_frmival,
#ifdef CONFIG_POLL
	.set_signal = video_bflb_dvp_set_signal,
#endif
};

static int video_bflb_dvp_init(const struct device *dev)
{
	const struct video_bflb_dvp_config *cfg = dev->config;
	struct video_bflb_dvp_data *data = dev->data;
	struct video_format fmt = {.type = VIDEO_BUF_TYPE_OUTPUT};
	int ret;

	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);
	k_sem_init(&data->frame_ready, 0, 1);

	k_thread_create(&data->thread, cfg->stack, cfg->stack_size, video_bflb_dvp_thread,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_VIDEO_BFLB_DVP_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&data->thread, dev->name);

	ret = video_bflb_dvp_get_fmt(dev, &fmt);
	if (ret < 0) {
		LOG_ERR("Failed to get the source format (%d)", ret);
		return ret;
	}

	return video_bflb_dvp_set_fmt(dev, &fmt);
}

#define SOURCE_DEV(n) DEVICE_DT_GET(DT_INST_PHANDLE(n, source))

#define DVP_ENDPOINT(n) DT_INST_ENDPOINT_BY_ID(n, 0, 0)

/*
 * The controller reads eight data lines on the rising edge of the pixel clock
 * and cannot be told otherwise, so a bus described as anything else would be
 * captured wrongly rather than not at all.
 */
#define VIDEO_BFLB_DVP_CHECK_BUS(n)                                                                \
	BUILD_ASSERT(DT_PROP_OR(DVP_ENDPOINT(n), bus_width, DVP_BUS_WIDTH) == DVP_BUS_WIDTH,       \
		     "bflb,dvp only drives an eight bit parallel bus");                            \
	BUILD_ASSERT(DT_PROP_OR(DVP_ENDPOINT(n), pclk_sample, 1) == 1,                             \
		     "bflb,dvp only samples on the rising pixel clock edge");                      \
	BUILD_ASSERT(DT_PROP_OR(DVP_ENDPOINT(n), data_shift, 0) == 0,                              \
		     "bflb,dvp reads the data lines unshifted");

#define VIDEO_BFLB_DVP_INIT(n)                                                                     \
	VIDEO_BFLB_DVP_CHECK_BUS(n)                                                                \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static uint8_t DVP_RING_SECTION __aligned(32)                                              \
		video_bflb_dvp_ring_##n[CONFIG_VIDEO_BFLB_DVP_RING_SIZE];                          \
	static K_THREAD_STACK_DEFINE(video_bflb_dvp_stack_##n,                                     \
				     CONFIG_VIDEO_BFLB_DVP_THREAD_STACK_SIZE);                     \
                                                                                                   \
	static const struct video_bflb_dvp_config video_bflb_dvp_config_##n = {                    \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.source_dev = SOURCE_DEV(n),                                                       \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_core = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, core, id),    \
		.clock_ref = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, ref, id),      \
		.mclk_frequency = DT_INST_PROP(n, mclk_frequency),                                 \
		.ring = video_bflb_dvp_ring_##n,                                                   \
		.ring_size = sizeof(video_bflb_dvp_ring_##n),                                      \
		.stack = video_bflb_dvp_stack_##n,                                                 \
		.stack_size = K_THREAD_STACK_SIZEOF(video_bflb_dvp_stack_##n),                     \
		.pclk_frequency = DT_INST_PROP(n, pclk_frequency),                                 \
		.hsync_active = DT_PROP_OR(DVP_ENDPOINT(n), hsync_active, 1),                      \
		.vsync_active = DT_PROP_OR(DVP_ENDPOINT(n), vsync_active, 1),                      \
	};                                                                                         \
                                                                                                   \
	static struct video_bflb_dvp_data video_bflb_dvp_data_##n;                                 \
                                                                                                   \
	static int video_bflb_dvp_init_##n(const struct device *dev)                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), video_bflb_dvp_isr,         \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
                                                                                                   \
		return video_bflb_dvp_init(dev);                                                   \
	}                                                                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, video_bflb_dvp_init_##n, NULL, &video_bflb_dvp_data_##n,          \
			      &video_bflb_dvp_config_##n, POST_KERNEL,                             \
			      CONFIG_VIDEO_INIT_PRIORITY, &video_bflb_dvp_driver_api);             \
                                                                                                   \
	VIDEO_DEVICE_DEFINE(video_bflb_dvp_##n, DEVICE_DT_INST_GET(n), SOURCE_DEV(n));

DT_INST_FOREACH_STATUS_OKAY(VIDEO_BFLB_DVP_INIT)

/*
 * The sensor is clocked by CAM_REF on a chip clock output pin and needs it before
 * its own initialization can talk to it over I2C, so the clock is requested and
 * the pins are muxed ahead of every video device.
 */
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
static int video_bflb_dvp_clock_init(void)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	const struct video_bflb_dvp_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("Clock controller is not ready");
		return -ENODEV;
	}

	ret = clock_control_set_rate(cfg->clock_dev, cfg->clock_ref,
				     (clock_control_subsys_rate_t)(uintptr_t)cfg->mclk_frequency);
	if (ret < 0) {
		LOG_ERR("Cannot produce a %u Hz sensor reference clock (%d)",
			cfg->mclk_frequency, ret);
		return ret;
	}

	return pinctrl_apply_state(PINCTRL_DT_INST_DEV_CONFIG_GET(0), PINCTRL_STATE_DEFAULT);
}

SYS_INIT(video_bflb_dvp_clock_init, POST_KERNEL, CONFIG_VIDEO_BFLB_DVP_CLOCK_INIT_PRIORITY);
#endif
