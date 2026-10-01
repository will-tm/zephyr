/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_dvp2axi

#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/video/formats.h>
#include <zephyr/video/video.h>

#include <bflb_soc.h>
#include <cam_reg.h>
#include <glb_reg.h>

#include "video_common.h"

LOG_MODULE_REGISTER(video_bflb_dvp2axi, CONFIG_VIDEO_LOG_LEVEL);

/* The AXI write port is 64 bits wide and the driver always uses INCR16 bursts */
#define DVP_BURST_BYTES 128U

/*
 * Data mode, which selects how many bytes the controller stores per pixel. The
 * capture is pass-through, so it follows the pixel size of the format. The
 * controller also has modes for three and four byte pixels, left out because
 * there is no sensor here emitting them to check them against.
 */
#define DVP_DATA_MODE_1_OR_2_BYTE 0U

/* XLEN encoding for INCR16 bursts */
#define DVP_BURST_INCR16 3U


/*
 * The controller owns a ring of frames. It reports each frame it completes
 * through its frame FIFO and keeps filling the ring, while the completed frame
 * is lent to a buffer and sent from where it lies. The ring needs room for the
 * frame being written, the one being sent and one more, so that a frame is
 * not overwritten before it has gone out.
 */
#define DVP_RING_FRAMES 3

/*
 * A compressed stream has no frame length the controller could count, so it
 * runs around the whole ring and reports every chunk of this many bytes, which
 * the driver scans for the start and end of image markers.
 *
 * A frame running past its byte count wedges the controller at the next frame
 * end it sees, until the controller is reset. A sensor sending a compressed
 * stream stops its pixel clock between frames, so only some frame ends are
 * seen, at random. The front end therefore takes the frame sync away for the
 * whole stream, and the stream is one frame that never ends.
 */
#define DVP_JPEG_CHUNK 4096U

/*
 * A frame the sensor sends in the previous format can be larger than the byte count of the
 * new one, which wedges the controller, so capture waits for a couple of frames to go by.
 */
#define DVP_FMT_SETTLE_MS 150

/*
 * The frame a compressed stream is captured in never ends, so its line count runs up to the
 * end of the crop window, past which nothing is stored. The frame is restarted before that,
 * counting a line as a width of bytes, less than a compressed line holds.
 */
#define DVP_JPEG_LINE_BUDGET 49152U

#ifdef CONFIG_VIDEO_BFLB_DVP2AXI_RING_ZEPHYR_REGION
#define DVP_RING_SECTION Z_GENERIC_SECTION(CONFIG_VIDEO_BFLB_DVP2AXI_RING_ZEPHYR_REGION_NAME)
#else
#define DVP_RING_SECTION __noinit
#endif

struct video_bflb_dvp2axi_config {
	uint32_t base;
	const struct device *source_dev;
	uint8_t *ring;
	size_t ring_size;
	k_thread_stack_t *stack;
	size_t stack_size;
};

struct video_bflb_dvp2axi_data {
	struct video_format fmt;
	k_timepoint_t fmt_settled;
	uint32_t frame_size;
	uint32_t ring_bytes;
	uint8_t data_mode;
	bool streaming;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	/* Memory of each buffer while it is lent a frame of the capture ring */
	uint8_t *own_mem[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX];
	struct k_sem frame_ready;
	size_t rd;
	size_t soi;
	bool last_ff;
	size_t jpeg_bytes;
	bool jpeg_restart;
	struct k_thread thread;
#ifdef CONFIG_POLL
	struct k_poll_signal *signal_out;
#endif
};

static void video_bflb_dvp2axi_raise_signal(struct video_bflb_dvp2axi_data *data, int result)
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

/* Give a buffer back its own memory if it was lent a frame of the capture ring */
static void video_bflb_dvp2axi_return_frame(const struct device *dev, struct video_buffer *vbuf)
{
	const struct video_bflb_dvp2axi_config *cfg = dev->config;
	struct video_bflb_dvp2axi_data *data = dev->data;

	if (vbuf->buffer >= cfg->ring && vbuf->buffer < cfg->ring + cfg->ring_size) {
		vbuf->buffer = data->own_mem[vbuf->index];
	}
}

/* The controller stores whole bytes, so only byte sized pixels can pass through */
static int video_bflb_dvp2axi_data_mode(uint32_t pixelformat)
{
	/* A compressed stream is stored as it arrives on the bus */
	if (pixelformat == VIDEO_PIX_FMT_JPEG) {
		return DVP_DATA_MODE_1_OR_2_BYTE;
	}

	switch (video_bits_per_pixel(pixelformat)) {
	case 8:
	case 16:
		return DVP_DATA_MODE_1_OR_2_BYTE;
	default:
		return -ENOTSUP;
	}
}

static void video_bflb_dvp2axi_configure(const struct device *dev)
{
	const struct video_bflb_dvp2axi_config *cfg = dev->config;
	struct video_bflb_dvp2axi_data *data = dev->data;
	uint32_t width = data->fmt.width;
	uint32_t height = data->fmt.height;
	uint32_t regval;

	if (data->fmt.pitch == 0) {
		/* A compressed frame has no geometry on the bus, so nothing is cropped */
		width = 0xffff;
		height = 0xffff;
	}
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
	if (data->fmt.pitch != 0) {
		/* Software mode: the frame addresses are pushed on the frame FIFO */
		regval |= CAM_REG_SW_MODE;
		/* Restart each frame at the buffer start, so the image is not rolled */
		regval |= CAM_REG_HW_MODE_FWRAP;
	} else {
		/* A compressed stream is a run of chunks going around the whole ring, which
		 * only the hardware mode keeps writing while a single frame lasts
		 */
		regval &= ~(CAM_REG_SW_MODE | CAM_REG_HW_MODE_FWRAP);
	}
	regval |= data->data_mode << CAM_REG_DVP_DATA_MODE_SHIFT;
	regval |= DVP_BURST_INCR16 << CAM_REG_XLEN_SHIFT;
	/* The front end hands the syncs over active high */
	regval |= CAM_REG_FRAM_VLD_POL | CAM_REG_LINE_VLD_POL;
	sys_write32(regval, cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET);

	/* Interrupt on every frame, all error interrupts left masked */
	regval = sys_read32(cfg->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);
	regval &= ~(CAM_REG_FRAME_CNT_TRGR_INT_MASK | CAM_REG_INT_MEM_EN | CAM_REG_INT_FRAME_EN |
		    CAM_REG_INT_FIFO_EN | CAM_REG_INT_HCNT_EN | CAM_REG_INT_VCNT_EN);
	regval |= 1U << CAM_REG_FRAME_CNT_TRGR_INT_SHIFT;
	regval |= CAM_REG_INT_NORMAL_EN;
	sys_write32(regval, cfg->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);

	LOG_DBG("%ux%u, %u bytes/frame, config 0x%08x", width, height, data->frame_size,
		sys_read32(cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET));
}

static void video_bflb_dvp2axi_isr(const struct device *dev)
{
	const struct video_bflb_dvp2axi_config *cfg = dev->config;
	struct video_bflb_dvp2axi_data *data = dev->data;

	if ((sys_read32(cfg->base + CAM_DVP_STATUS_AND_ERROR_OFFSET) & CAM_STS_NORMAL_INT) == 0) {
		return;
	}

	sys_write32(CAM_REG_INT_NORMAL_CLR, cfg->base + CAM_DVP_FRAME_FIFO_POP_OFFSET);
	k_sem_give(&data->frame_ready);
}

/* Copy a frame out of the ring, which may wrap around its end */
static void video_bflb_dvp2axi_ring_copy(const struct device *dev, uint8_t *dst, size_t from,
				     size_t len)
{
	const struct video_bflb_dvp2axi_config *cfg = dev->config;
	struct video_bflb_dvp2axi_data *data = dev->data;
	size_t first = MIN(len, data->ring_bytes - from);

	memcpy(dst, cfg->ring + from, first);
	memcpy(dst + first, cfg->ring, len - first);
}

static void video_bflb_dvp2axi_jpeg_done(const struct device *dev, size_t end)
{
	struct video_bflb_dvp2axi_data *data = dev->data;
	struct video_buffer *vbuf;
	size_t len = (end + data->ring_bytes - data->soi) % data->ring_bytes;

	vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);
	if (vbuf == NULL) {
		video_bflb_dvp2axi_raise_signal(data, VIDEO_BUF_ERROR);
		return;
	}

	if (len > vbuf->size) {
		k_fifo_put(&data->fifo_in, vbuf);
		video_bflb_dvp2axi_raise_signal(data, VIDEO_BUF_ERROR);
		return;
	}

	video_bflb_dvp2axi_ring_copy(dev, vbuf->buffer, data->soi, len);

	vbuf->bytesused = len;
	vbuf->line_offset = 0;
	vbuf->timestamp = k_uptime_get_32();
	k_fifo_put(&data->fifo_out, vbuf);
	video_bflb_dvp2axi_raise_signal(data, VIDEO_BUF_DONE);
}

/* Scan the chunk the controller just wrote for the markers that bound each frame */
static void video_bflb_dvp2axi_jpeg_parse(const struct device *dev, size_t chunk)
{
	const struct video_bflb_dvp2axi_config *cfg = dev->config;
	struct video_bflb_dvp2axi_data *data = dev->data;
	size_t i;

	/* A chunk was lost, so whatever frame was in progress is incomplete */
	if (chunk != data->rd) {
		data->soi = SIZE_MAX;
		data->last_ff = false;
		video_bflb_dvp2axi_raise_signal(data, VIDEO_BUF_ERROR);
	}

	sys_cache_data_invd_range(cfg->ring + chunk, DVP_JPEG_CHUNK);
	data->jpeg_bytes += DVP_JPEG_CHUNK;

	for (i = chunk; i < chunk + DVP_JPEG_CHUNK; i++) {
		uint8_t byte = cfg->ring[i];

		if (data->last_ff && byte == 0xd8U) {
			data->soi = (i + data->ring_bytes - 1) % data->ring_bytes;
		} else if (data->last_ff && byte == 0xd9U && data->soi != SIZE_MAX) {
			video_bflb_dvp2axi_jpeg_done(dev, (i + 1) % data->ring_bytes);
			data->soi = SIZE_MAX;

			/* Restart between two frames, the next one is dropped */
			if (data->jpeg_bytes >= DVP_JPEG_LINE_BUDGET * data->fmt.width) {
				data->jpeg_restart = true;
				return;
			}
		}
		data->last_ff = (byte == 0xffU);
	}

	data->rd = (chunk + DVP_JPEG_CHUNK) % data->ring_bytes;
}

/* Reset the controller, which clears the counters a frame left behind */
static void video_bflb_dvp2axi_reset_block(void)
{
	uint32_t regval = sys_read32(GLB_BASE + GLB_SWRST_CFG0_OFFSET);

	sys_write32(regval | GLB_SWRST_D2XA_MSK, GLB_BASE + GLB_SWRST_CFG0_OFFSET);
	sys_write32(regval & ~GLB_SWRST_D2XA_MSK, GLB_BASE + GLB_SWRST_CFG0_OFFSET);
}

/* Start the frame a compressed stream is captured in over again */
static void video_bflb_dvp2axi_jpeg_restart(const struct device *dev)
{
	const struct video_bflb_dvp2axi_config *cfg = dev->config;
	struct video_bflb_dvp2axi_data *data = dev->data;
	uint32_t regval;

	video_stream_stop(cfg->source_dev, VIDEO_BUF_TYPE_OUTPUT);

	data->jpeg_restart = false;
	data->jpeg_bytes = 0;
	data->rd = 0;
	data->soi = SIZE_MAX;
	data->last_ff = false;

	video_bflb_dvp2axi_reset_block();
	video_bflb_dvp2axi_configure(dev);
	sys_write32(CAM_REG_INT_NORMAL_CLR | CAM_RFIFO_POP,
		    cfg->base + CAM_DVP_FRAME_FIFO_POP_OFFSET);

	/* A stream stopped meanwhile must not be enabled again */
	if (!data->streaming) {
		return;
	}
	regval = sys_read32(cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	sys_write32(regval | CAM_REG_DVP_ENABLE, cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET);

	video_stream_start(cfg->source_dev, VIDEO_BUF_TYPE_OUTPUT);
}

static void video_bflb_dvp2axi_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct video_bflb_dvp2axi_config *cfg = dev->config;
	struct video_bflb_dvp2axi_data *data = dev->data;
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

			if (data->fmt.pitch == 0) {
				video_bflb_dvp2axi_jpeg_parse(dev, frame - cfg->ring);
				sys_write32(CAM_RFIFO_POP,
					    cfg->base + CAM_DVP_FRAME_FIFO_POP_OFFSET);
				if (data->jpeg_restart && data->streaming) {
					video_bflb_dvp2axi_jpeg_restart(dev);
				}
				continue;
			}

			vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);
			if (vbuf == NULL) {
				video_bflb_dvp2axi_raise_signal(data, VIDEO_BUF_ERROR);
			} else {
				/* Lend the frame in the ring rather than copying it out, the ring
				 * holds enough frames for it to be sent before it is written again
				 */
				sys_cache_data_invd_range(frame, data->frame_size);
				data->own_mem[vbuf->index] = vbuf->buffer;
				vbuf->buffer = frame;

				vbuf->bytesused = data->frame_size;
				vbuf->line_offset = 0;
				vbuf->timestamp = k_uptime_get_32();
				k_fifo_put(&data->fifo_out, vbuf);
				video_bflb_dvp2axi_raise_signal(data, VIDEO_BUF_DONE);
			}

			sys_write32(CAM_RFIFO_POP, cfg->base + CAM_DVP_FRAME_FIFO_POP_OFFSET);
		}
	}
}

static int video_bflb_dvp2axi_set_stream(const struct device *dev, bool enable,
				     enum video_buf_type type)
{
	const struct video_bflb_dvp2axi_config *cfg = dev->config;
	struct video_bflb_dvp2axi_data *data = dev->data;
	struct video_buffer *vbuf;
	struct k_fifo pending;
	uint32_t regval;
	int ret;

	if (!enable) {
		if (!data->streaming) {
			return 0;
		}

		/* Let the pipeline drain before the controller is disabled under it */
		ret = video_stream_stop(cfg->source_dev, type);
		for (int i = 0; i < 100; i++) {
			if (sys_read32(cfg->base + CAM_DVP_STATUS_AND_ERROR_OFFSET) &
			    CAM_ST_DVP_IDLE) {
				break;
			}
			k_sleep(K_MSEC(1));
		}

		regval = sys_read32(cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
		regval &= ~CAM_REG_DVP_ENABLE;
		sys_write32(regval, cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
		data->streaming = false;

		/* The ring is about to be reused, so take back the frames lent to queued buffers */
		k_fifo_init(&pending);
		while ((vbuf = k_fifo_get(&data->fifo_out, K_NO_WAIT)) != NULL) {
			video_bflb_dvp2axi_return_frame(dev, vbuf);
			k_fifo_put(&pending, vbuf);
		}
		while ((vbuf = k_fifo_get(&pending, K_NO_WAIT)) != NULL) {
			k_fifo_put(&data->fifo_out, vbuf);
		}

		return ret;
	}

	if (data->streaming) {
		return -EBUSY;
	}

	/* Checked here rather than when setting the format, so that a format can
	 * still be probed for its size by a consumer that will not stream it
	 */
	if (data->ring_bytes > cfg->ring_size) {
		LOG_ERR("Capture ring of %zu bytes holds less than %u frames of %u bytes",
			cfg->ring_size, DVP_RING_FRAMES, data->frame_size);
		return -ENOMEM;
	}

	while (!sys_timepoint_expired(data->fmt_settled)) {
		k_sleep(sys_timepoint_timeout(data->fmt_settled));
	}

	data->rd = 0;
	data->soi = SIZE_MAX;
	data->last_ff = false;
	data->jpeg_bytes = 0;
	data->jpeg_restart = false;
	video_bflb_dvp2axi_reset_block();
	video_bflb_dvp2axi_configure(dev);
	sys_write32(CAM_REG_INT_NORMAL_CLR | CAM_RFIFO_POP,
		    cfg->base + CAM_DVP_FRAME_FIFO_POP_OFFSET);

	regval = sys_read32(cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	regval |= CAM_REG_DVP_ENABLE;
	sys_write32(regval, cfg->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	data->streaming = true;

	ret = video_stream_start(cfg->source_dev, type);
	if (ret < 0) {
		video_bflb_dvp2axi_set_stream(dev, false, type);
		return ret;
	}

	return 0;
}

static int video_bflb_dvp2axi_get_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct video_bflb_dvp2axi_config *cfg = dev->config;
	int ret;

	ret = video_get_format(cfg->source_dev, fmt);
	if (ret < 0) {
		return ret;
	}

	if (fmt->pixelformat == VIDEO_PIX_FMT_JPEG) {
		return 0;
	}

	return video_estimate_fmt_size(fmt);
}

static int video_bflb_dvp2axi_set_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct video_bflb_dvp2axi_config *cfg = dev->config;
	struct video_bflb_dvp2axi_data *data = dev->data;
	uint32_t frame_size;
	int ret;

	if (data->streaming) {
		return -EBUSY;
	}

	ret = video_set_format(cfg->source_dev, fmt);
	if (ret < 0) {
		return ret;
	}
	data->fmt_settled = sys_timepoint_calc(K_MSEC(DVP_FMT_SETTLE_MS));

	/* A compressed frame has no fixed length, only the source knows its worst case */
	if (fmt->pixelformat != VIDEO_PIX_FMT_JPEG) {
		ret = video_estimate_fmt_size(fmt);
		if (ret < 0) {
			return ret;
		}
	}

	ret = video_bflb_dvp2axi_data_mode(fmt->pixelformat);
	if (ret < 0) {
		LOG_ERR("Format %s cannot be stored by the capture interface",
			VIDEO_FOURCC_TO_STR(fmt->pixelformat));
		return ret;
	}
	data->data_mode = (uint8_t)ret;

	if (fmt->pitch == 0) {
		data->fmt = *fmt;
		data->frame_size = DVP_JPEG_CHUNK;
		data->ring_bytes = ROUND_DOWN(cfg->ring_size, DVP_JPEG_CHUNK);
		return 0;
	}

	frame_size = fmt->pitch * fmt->height;
	if ((frame_size % DVP_BURST_BYTES) != 0) {
		LOG_ERR("Frame size %u is not a multiple of %u", frame_size, DVP_BURST_BYTES);
		return -EINVAL;
	}

	data->fmt = *fmt;
	data->frame_size = frame_size;
	data->ring_bytes = frame_size * DVP_RING_FRAMES;

	return 0;
}

static int video_bflb_dvp2axi_get_caps(const struct device *dev, struct video_caps *caps)
{
	const struct video_bflb_dvp2axi_config *cfg = dev->config;

	/* One buffer holds the frame being captured, the other is handed to the application */
	caps->min_vbuf_count = 2;

	return video_get_caps(cfg->source_dev, caps);
}

static int video_bflb_dvp2axi_enqueue(const struct device *dev, struct video_buffer *vbuf)
{
	struct video_bflb_dvp2axi_data *data = dev->data;

	/* A compressed frame is copied out only if it fits, a raw one always has to */
	if (data->fmt.pitch != 0 && vbuf->size < data->frame_size) {
		LOG_ERR("Buffer of %u bytes is too small for a %u byte frame", vbuf->size,
			data->frame_size);
		return -EINVAL;
	}

	if (vbuf->index >= ARRAY_SIZE(data->own_mem)) {
		return -EINVAL;
	}

	video_bflb_dvp2axi_return_frame(dev, vbuf);

	/* The controller writes through AXI, so no dirty line may be left behind */
	sys_cache_data_flush_and_invd_range(vbuf->buffer, MIN(vbuf->size, data->frame_size));

	vbuf->bytesused = 0;
	k_fifo_put(&data->fifo_in, vbuf);

	return 0;
}

static int video_bflb_dvp2axi_dequeue(const struct device *dev, struct video_buffer **vbuf,
				  k_timeout_t timeout)
{
	struct video_bflb_dvp2axi_data *data = dev->data;

	*vbuf = k_fifo_get(&data->fifo_out, timeout);
	if (*vbuf == NULL) {
		return -EAGAIN;
	}

	return 0;
}

static int video_bflb_dvp2axi_flush(const struct device *dev, bool cancel)
{
	struct video_bflb_dvp2axi_data *data = dev->data;
	struct video_buffer *vbuf;

	if (!cancel) {
		while (!k_fifo_is_empty(&data->fifo_in)) {
			k_sleep(K_MSEC(1));
		}
		return 0;
	}

	while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT)) != NULL) {
		k_fifo_put(&data->fifo_out, vbuf);
		video_bflb_dvp2axi_raise_signal(data, VIDEO_BUF_ABORTED);
	}

	return 0;
}

static int video_bflb_dvp2axi_set_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct video_bflb_dvp2axi_config *cfg = dev->config;

	return video_set_frmival(cfg->source_dev, frmival);
}

static int video_bflb_dvp2axi_get_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct video_bflb_dvp2axi_config *cfg = dev->config;

	return video_get_frmival(cfg->source_dev, frmival);
}

static int video_bflb_dvp2axi_enum_frmival(const struct device *dev, struct video_frmival_enum *fie)
{
	const struct video_bflb_dvp2axi_config *cfg = dev->config;

	return video_enum_frmival(cfg->source_dev, fie);
}

#ifdef CONFIG_POLL
static int video_bflb_dvp2axi_set_signal(const struct device *dev, struct k_poll_signal *sig)
{
	struct video_bflb_dvp2axi_data *data = dev->data;

	data->signal_out = sig;

	return 0;
}
#endif

static DEVICE_API(video, video_bflb_dvp2axi_driver_api) = {
	.set_format = video_bflb_dvp2axi_set_fmt,
	.get_format = video_bflb_dvp2axi_get_fmt,
	.set_stream = video_bflb_dvp2axi_set_stream,
	.get_caps = video_bflb_dvp2axi_get_caps,
	.enqueue = video_bflb_dvp2axi_enqueue,
	.dequeue = video_bflb_dvp2axi_dequeue,
	.flush = video_bflb_dvp2axi_flush,
	.set_frmival = video_bflb_dvp2axi_set_frmival,
	.get_frmival = video_bflb_dvp2axi_get_frmival,
	.enum_frmival = video_bflb_dvp2axi_enum_frmival,
#ifdef CONFIG_POLL
	.set_signal = video_bflb_dvp2axi_set_signal,
#endif
};

static int video_bflb_dvp2axi_init(const struct device *dev)
{
	const struct video_bflb_dvp2axi_config *cfg = dev->config;
	struct video_bflb_dvp2axi_data *data = dev->data;
	struct video_format fmt = {.type = VIDEO_BUF_TYPE_OUTPUT};
	int ret;

	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);
	k_sem_init(&data->frame_ready, 0, 1);

	k_thread_create(&data->thread, cfg->stack, cfg->stack_size, video_bflb_dvp2axi_thread,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_VIDEO_BFLB_DVP2AXI_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&data->thread, dev->name);

	ret = video_bflb_dvp2axi_get_fmt(dev, &fmt);
	if (ret < 0) {
		LOG_ERR("Failed to get the source format (%d)", ret);
		return ret;
	}

	return video_bflb_dvp2axi_set_fmt(dev, &fmt);
}

#define SOURCE_DEV(n) DEVICE_DT_GET(DT_INST_PHANDLE(n, source))

#define VIDEO_BFLB_DVP2AXI_INIT(n)                                                                 \
	static uint8_t DVP_RING_SECTION __aligned(32)                                              \
		video_bflb_dvp2axi_ring_##n[CONFIG_VIDEO_BFLB_DVP2AXI_RING_SIZE];                  \
	static K_THREAD_STACK_DEFINE(video_bflb_dvp2axi_stack_##n,                                 \
				     CONFIG_VIDEO_BFLB_DVP2AXI_THREAD_STACK_SIZE);                 \
                                                                                                   \
	static const struct video_bflb_dvp2axi_config video_bflb_dvp2axi_config_##n = {            \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.source_dev = SOURCE_DEV(n),                                                       \
		.ring = video_bflb_dvp2axi_ring_##n,                                               \
		.ring_size = sizeof(video_bflb_dvp2axi_ring_##n),                                  \
		.stack = video_bflb_dvp2axi_stack_##n,                                             \
		.stack_size = K_THREAD_STACK_SIZEOF(video_bflb_dvp2axi_stack_##n),                 \
	};                                                                                         \
                                                                                                   \
	static struct video_bflb_dvp2axi_data video_bflb_dvp2axi_data_##n;                         \
                                                                                                   \
	static int video_bflb_dvp2axi_init_##n(const struct device *dev)                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),                             \
			    video_bflb_dvp2axi_isr, DEVICE_DT_INST_GET(n), 0);                     \
		irq_enable(DT_INST_IRQN(n));                                                       \
                                                                                                   \
		return video_bflb_dvp2axi_init(dev);                                               \
	}                                                                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, video_bflb_dvp2axi_init_##n, NULL,                                \
			      &video_bflb_dvp2axi_data_##n, &video_bflb_dvp2axi_config_##n,        \
			      POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY,                             \
			      &video_bflb_dvp2axi_driver_api);                                     \
                                                                                                   \
	VIDEO_DEVICE_DEFINE(video_bflb_dvp2axi_##n, DEVICE_DT_INST_GET(n), SOURCE_DEV(n));

DT_INST_FOREACH_STATUS_OKAY(VIDEO_BFLB_DVP2AXI_INIT)
