/*
 * Copyright (c) 2025 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdlib.h>

#include <sample_usbd.h>

#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/video/video.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/class/usbd_uvc.h>

LOG_MODULE_REGISTER(uvc_sample, LOG_LEVEL_INF);

const static struct device *const uvc_dev = DEVICE_DT_GET(DT_NODELABEL(uvc));
const static struct device *const video_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));
static const struct device *const videoenc_dev = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_videoenc));

/* Format capabilities of video_dev, used everywhere through the sample */
static struct video_caps video_caps = {.type = VIDEO_BUF_TYPE_OUTPUT};
static struct video_caps videoenc_out_caps = {.type = VIDEO_BUF_TYPE_OUTPUT};
static struct video_caps videoenc_in_caps = {.type = VIDEO_BUF_TYPE_INPUT};

/* Whether the format the host selected is produced by the encoder */
static bool use_videoenc;

/* Buffers allocated once, large enough for any format offered, and reused by every stream */
static struct video_buffer *app_bufs[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX];
static size_t app_buf_size;

#if DT_HAS_CHOSEN(zephyr_videoenc) && CONFIG_VIDEO_BUFFER_POOL_NUM_MAX < 2
#error CONFIG_VIDEO_BUFFER_POOL_NUM_MAX must be >=2 in order to use a zephyr,videoenc
#endif

static bool app_has_videoenc(void)
{
	return (videoenc_dev != NULL);
}

static const struct device *app_uvc_source_dev(void)
{
	if (use_videoenc) {
		return videoenc_dev;
	} else {
		return video_dev;
	}
}

static bool app_caps_have_format(const struct video_caps *caps, uint32_t pixfmt)
{
	for (int i = 0; caps->format_caps[i].pixelformat != 0; i++) {
		if (caps->format_caps[i].pixelformat == pixfmt) {
			return true;
		}
	}

	return false;
}

/* Format the camera hands to the encoder: the first one of the camera the encoder takes */
static uint32_t app_videoenc_input_format(void)
{
	for (int i = 0; video_caps.format_caps[i].pixelformat != 0; i++) {
		uint32_t pixfmt = video_caps.format_caps[i].pixelformat;

		if (app_caps_have_format(&videoenc_in_caps, pixfmt)) {
			return pixfmt;
		}
	}

	return 0;
}

/* Pixel formats present in one of the UVC 1.5 standard */
static bool app_is_supported_format(uint32_t pixfmt)
{
	return pixfmt == VIDEO_PIX_FMT_JPEG ||
	       pixfmt == VIDEO_PIX_FMT_YUYV ||
	       pixfmt == VIDEO_PIX_FMT_NV12 ||
	       pixfmt == VIDEO_PIX_FMT_H264;
}

static bool app_has_supported_format(const struct video_caps *const caps)
{
	const struct video_format_cap *const fmts = caps->format_caps;

	for (int i = 0; fmts[i].pixelformat != 0; i++) {
		if (app_is_supported_format(fmts[i].pixelformat)) {
			return true;
		}
	}

	return false;
}

static int app_add_format(const struct device *uvc_src_dev, uint32_t pixfmt, uint32_t width,
			  uint32_t height, bool has_sup_fmts)
{
	struct video_format fmt = {
		.pixelformat = pixfmt,
		.width = width,
		.height = height,
		.type = VIDEO_BUF_TYPE_OUTPUT,
	};
	size_t size;
	int ret;

	/* If the system has any standard pixel format, only propose them to the host */
	if (has_sup_fmts && !app_is_supported_format(pixfmt)) {
		return 0;
	}

	/* Set the format to get the size */
	ret = video_set_compose_format(uvc_src_dev, &fmt);
	if (ret != 0) {
		LOG_ERR("Could not set the format of %s to %s %ux%u (size %u)",
			uvc_src_dev->name, VIDEO_FOURCC_TO_STR(fmt.pixelformat),
			fmt.width, fmt.height, fmt.size);
		return ret;
	}

	if (fmt.size > (CONFIG_VIDEO_BUFFER_POOL_HEAP_SIZE / CONFIG_VIDEO_BUFFER_POOL_NUM_MAX)) {
		LOG_WRN("Skipping format %ux%u", fmt.width, fmt.height);
		return 0;
	}
	size = fmt.size;

	/* An encoded stream also needs the camera frames it is made of to fit */
	if (uvc_src_dev == videoenc_dev) {
		struct video_format in_fmt = {
			.pixelformat = app_videoenc_input_format(),
			.width = width,
			.height = height,
			.type = VIDEO_BUF_TYPE_OUTPUT,
		};

		ret = video_set_compose_format(video_dev, &in_fmt);
		if (ret != 0 ||
		    in_fmt.size > (CONFIG_VIDEO_BUFFER_POOL_HEAP_SIZE /
				   CONFIG_VIDEO_BUFFER_POOL_NUM_MAX)) {
			LOG_WRN("Skipping format %ux%u", fmt.width, fmt.height);
			return 0;
		}
		size = MAX(size, in_fmt.size);
	}

	ret = uvc_device_add_format(uvc_dev, &fmt);
	if (ret == -ENOMEM) {
		/* If there are too many formats, ignore the error, just list fewer formats */
		return 0;
	}
	if (ret == 0) {
		app_buf_size = MAX(app_buf_size, size);
	}
	return ret;
}

struct video_resolution {
	uint16_t width;
	uint16_t height;
};

static struct video_resolution video_common_fmts[] = {
	{ .width = 160,		.height = 120,	},	/* QQVGA */
	{ .width = 320,		.height = 240,	},	/* QVGA */
	{ .width = 640,		.height = 480,	},	/* VGA */
	{ .width = 854,		.height = 480,	},	/* WVGA */
	{ .width = 800,		.height = 600,	},	/* SVGA */
	{ .width = 1280,	.height = 720,	},	/* HD */
	{ .width = 1280,	.height = 1024,	},	/* SXGA */
	{ .width = 1920,	.height = 1080,	},	/* FHD */
	{ .width = 3840,	.height = 2160,	},	/* UHD */
};

/* Submit the resolutions of a camera format capability, produced as pixfmt by uvc_src_dev */
static int app_add_cap_formats(const struct device *uvc_src_dev, uint32_t pixfmt,
			       const struct video_format_cap *vcap, bool has_sup_fmts)
{
	int count = 1;
	int ret;

	ret = app_add_format(uvc_src_dev, pixfmt, vcap->width_min, vcap->height_min,
			     has_sup_fmts);
	if (ret != 0) {
		return ret;
	}

	if (vcap->width_min != vcap->width_max || vcap->height_min != vcap->height_max) {
		ret = app_add_format(uvc_src_dev, pixfmt, vcap->width_max, vcap->height_max,
				     has_sup_fmts);
		if (ret != 0) {
			return ret;
		}

		count++;
	}

	if (vcap->width_step == 0 && vcap->height_step == 0) {
		return 0;
	}

	/* RANGE Resolution processing */
	for (int j = 0; j < ARRAY_SIZE(video_common_fmts); j++) {
		if (count >= CONFIG_APP_VIDEO_MAX_RESOLUTIONS) {
			break;
		}

		if (!IN_RANGE(video_common_fmts[j].width, vcap->width_min, vcap->width_max) ||
		    !IN_RANGE(video_common_fmts[j].height, vcap->height_min, vcap->height_max)) {
			continue;
		}

		if ((video_common_fmts[j].width - vcap->width_min) % vcap->width_step ||
		    (video_common_fmts[j].height - vcap->height_min) % vcap->height_step) {
			continue;
		}

		ret = app_add_format(uvc_src_dev, pixfmt, video_common_fmts[j].width,
				     video_common_fmts[j].height, has_sup_fmts);
		if (ret != 0) {
			return ret;
		}

		count++;
	}

	return 0;
}

/*
 * Submit to UVC only the formats expected to be working (enough memory for the size, etc.):
 * the formats of the camera, then those the encoder produces out of them, so that the
 * host chooses between a raw and a compressed stream.
 */
static int app_add_filtered_formats(void)
{
	const uint32_t enc_in_fmt = app_has_videoenc() ? app_videoenc_input_format() : 0;
	int ret;

	for (int i = 0; video_caps.format_caps[i].pixelformat != 0; i++) {
		const struct video_format_cap *vcap = &video_caps.format_caps[i];

		ret = app_add_cap_formats(video_dev, vcap->pixelformat, vcap,
					  app_has_supported_format(&video_caps));
		if (ret != 0) {
			return ret;
		}
	}

	if (enc_in_fmt == 0) {
		return 0;
	}

	for (int i = 0; video_caps.format_caps[i].pixelformat != 0; i++) {
		const struct video_format_cap *vcap = &video_caps.format_caps[i];

		if (vcap->pixelformat != enc_in_fmt) {
			continue;
		}

		/*
		 * FIXME - in the meantime that auto-negotiation is supported,
		 * when a video encoder is present, always use the first pixelformat.
		 */
		ret = app_add_cap_formats(videoenc_dev,
					  videoenc_out_caps.format_caps[0].pixelformat, vcap,
					  app_has_supported_format(&videoenc_out_caps));
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

static int app_init_videoenc(const struct device *const dev)
{
	int ret;

	if (!device_is_ready(dev)) {
		LOG_ERR("video encoder %s failed to initialize", dev->name);
		return -ENODEV;
	}

	ret = video_get_caps(dev, &videoenc_out_caps);
	if (ret != 0) {
		LOG_ERR("Unable to retrieve video encoder output capabilities");
		return ret;
	}

	ret = video_get_caps(dev, &videoenc_in_caps);
	if (ret != 0) {
		LOG_ERR("Unable to retrieve video encoder input capabilities");
		return ret;
	}

	/*
	 * FIXME - we should look carefully at both video capture output and encoder input
	 * caps to detect intermediate format.
	 * This is where we should define the format which is going to be used
	 * between the camera and the encoder input
	 */

	return 0;
}

static int app_configure_videoenc(const struct device *const dev,
				  uint32_t width, uint32_t height,
				  uint32_t sink_pixelformat, uint32_t source_pixelformat,
				  struct video_buffer **bufs, uint32_t nb_buffer)
{
	struct video_format fmt = {
		.width = width,
		.height = height,
	};
	int ret;

	/*
	 * Need to configure both input & output of the encoder
	 * and allocate / enqueue buffers to the output of the
	 * encoder
	 */
	fmt.type = VIDEO_BUF_TYPE_INPUT;
	fmt.pixelformat = sink_pixelformat;
	ret = video_set_compose_format(dev, &fmt);
	if (ret != 0) {
		LOG_ERR("Could not set the %s encoder input format", dev->name);
		return ret;
	}

	fmt.type = VIDEO_BUF_TYPE_OUTPUT;
	fmt.pixelformat = source_pixelformat;
	ret = video_set_compose_format(dev, &fmt);
	if (ret != 0) {
		LOG_ERR("Could not set the %s encoder output format", dev->name);
		return ret;
	}

	LOG_INF("Enqueuing %u buffers for encoder output", nb_buffer);

	for (int i = 0; i < nb_buffer; i++) {
		bufs[i]->type = VIDEO_BUF_TYPE_OUTPUT;

		ret = video_enqueue(dev, bufs[i]);
		if (ret != 0) {
			LOG_ERR("Could not enqueue video buffer");
			return ret;
		}
	}

	return 0;
}

static int app_start_videoenc(const struct device *const dev)
{
	int ret;

	ret = video_stream_start(dev, VIDEO_BUF_TYPE_OUTPUT);
	if (ret != 0) {
		LOG_ERR("Failed to start %s output", dev->name);
		return ret;
	}

	ret = video_stream_start(dev, VIDEO_BUF_TYPE_INPUT);
	if (ret != 0) {
		LOG_ERR("Failed to start %s input", dev->name);
		return ret;
	}

	return 0;
}

/* Configure and start the pipeline for the format the host selected */
static int app_start_stream(const struct video_format *host_fmt,
			    struct video_frmival *frmival, struct k_poll_signal *sig,
			    k_timeout_t *timeout)
{
	const struct device *uvc_src_dev;
	struct video_format fmt = *host_fmt;
	uint32_t uvc_buf_count = CONFIG_VIDEO_BUFFER_POOL_NUM_MAX;
	int ret;

	LOG_INF("The host selected format '%s' %ux%u at frame interval %u/%u",
		VIDEO_FOURCC_TO_STR(fmt.pixelformat), fmt.width, fmt.height,
		frmival->numerator, frmival->denominator);

	/* A raw format is sent as captured, an encoded one goes through the encoder */
	use_videoenc = app_has_videoenc() && app_caps_have_format(&videoenc_out_caps,
								  fmt.pixelformat);
	uvc_src_dev = app_uvc_source_dev();

	/* When using encoder, we split the VIDEO_BUFFER_POOL_NUM_MAX in 2 */
	if (use_videoenc) {
		uvc_buf_count /= 2;

		ret = app_configure_videoenc(videoenc_dev, fmt.width, fmt.height,
					     app_videoenc_input_format(), fmt.pixelformat,
					     &app_bufs[uvc_buf_count],
					     CONFIG_VIDEO_BUFFER_POOL_NUM_MAX - uvc_buf_count);
		if (ret != 0) {
			return ret;
		}

		fmt.pixelformat = app_videoenc_input_format();
	}

	fmt.type = VIDEO_BUF_TYPE_OUTPUT;

	ret = video_set_compose_format(video_dev, &fmt);
	if (ret != 0) {
		LOG_ERR("Could not set the format of %s to %s %ux%u (size %u)",
			video_dev->name, VIDEO_FOURCC_TO_STR(fmt.pixelformat),
			fmt.width, fmt.height, fmt.size);
		return ret;
	}

	/*
	 * FIXME - shortcut here since current available encoders do not
	 * have frmival support for the time being so this is done directly
	 * at camera level
	 */
	ret = video_set_frmival(video_dev, frmival);
	if (ret != 0) {
		LOG_WRN("Could not set the framerate of %s", video_dev->name);
	}

	for (int i = 0; i < uvc_buf_count; i++) {
		app_bufs[i]->type = VIDEO_BUF_TYPE_OUTPUT;

		ret = video_enqueue(video_dev, app_bufs[i]);
		if (ret != 0) {
			LOG_ERR("Could not enqueue video buffer");
			return ret;
		}
	}

	LOG_DBG("Preparing signaling for %s input/output", uvc_src_dev->name);

	*timeout = K_FOREVER;

	ret = video_set_signal(uvc_src_dev, sig);
	if (ret != 0) {
		LOG_WRN("Failed to setup the signal on %s output endpoint", uvc_src_dev->name);
		*timeout = K_MSEC(1);
	}

	ret = video_set_signal(uvc_dev, sig);
	if (ret != 0) {
		LOG_ERR("Failed to setup the signal on %s input endpoint", uvc_dev->name);
		return ret;
	}

	/* Frames captured by the camera have to be handed to the encoder as they come */
	if (use_videoenc) {
		ret = video_set_signal(video_dev, sig);
		if (ret != 0) {
			LOG_WRN("Failed to setup the signal on %s output endpoint",
				video_dev->name);
			*timeout = K_MSEC(1);
		}
	}

	LOG_INF("Starting the video transfer");

	ret = video_stream_start(uvc_dev, VIDEO_BUF_TYPE_INPUT);
	if (ret != 0) {
		LOG_ERR("Failed to start %s", uvc_dev->name);
		return ret;
	}

	if (use_videoenc) {
		ret = app_start_videoenc(videoenc_dev);
		if (ret != 0) {
			return ret;
		}
	}

	ret = video_stream_start(video_dev, VIDEO_BUF_TYPE_OUTPUT);
	if (ret != 0) {
		LOG_ERR("Failed to start %s", video_dev->name);
		return ret;
	}

	return 0;
}

/* Move frames along the pipeline until the host stops the stream or selects another format */
static int app_run_stream(const struct video_format *host_fmt, struct k_poll_event *evt,
			  struct k_poll_signal *sig, k_timeout_t timeout)
{
	const struct device *uvc_src_dev = app_uvc_source_dev();
	/* Wake up regularly to notice a stream the host stopped */
	const k_timeout_t wait = K_TIMEOUT_EQ(timeout, K_FOREVER) ? K_MSEC(100) : timeout;
	struct video_format fmt;
	int ret;

	while (true) {
		ret = k_poll(evt, 1, wait);
		if (ret != 0 && ret != -EAGAIN) {
			LOG_ERR("Poll exited with status %d", ret);
			return ret;
		}

		if (use_videoenc) {
			ret = video_transfer_buffer(video_dev, uvc_src_dev,
						    VIDEO_BUF_TYPE_OUTPUT, VIDEO_BUF_TYPE_INPUT,
						    K_NO_WAIT);
			if (ret != 0 && ret != -EAGAIN) {
				LOG_ERR("Failed to transfer from %s to %s",
					video_dev->name, uvc_src_dev->name);
				return ret;
			}
		}

		ret = video_transfer_buffer(uvc_src_dev, uvc_dev,
					    VIDEO_BUF_TYPE_OUTPUT, VIDEO_BUF_TYPE_INPUT,
					    K_NO_WAIT);
		if (ret != 0 && ret != -EAGAIN) {
			LOG_ERR("Failed to transfer from %s to %s",
				uvc_src_dev->name, uvc_dev->name);
			return ret;
		}

		if (use_videoenc) {
			ret = video_transfer_buffer(uvc_src_dev, video_dev,
						    VIDEO_BUF_TYPE_INPUT, VIDEO_BUF_TYPE_OUTPUT,
						    K_NO_WAIT);
			if (ret != 0 && ret != -EAGAIN) {
				LOG_ERR("Failed to transfer from %s to %s",
					uvc_src_dev->name, video_dev->name);
				return ret;
			}
		}

		ret = video_transfer_buffer(uvc_dev, uvc_src_dev,
					    VIDEO_BUF_TYPE_INPUT, VIDEO_BUF_TYPE_OUTPUT,
					    K_NO_WAIT);
		if (ret != 0 && ret != -EAGAIN) {
			LOG_ERR("Failed to transfer from %s to %s",
				uvc_dev->name, uvc_src_dev->name);
			return ret;
		}

		k_poll_signal_reset(sig);

		fmt.type = VIDEO_BUF_TYPE_INPUT;
		ret = video_get_format(uvc_dev, &fmt);
		if (ret == -EAGAIN) {
			LOG_INF("The host stopped the stream");
			return 0;
		}
		if (ret == 0 && (fmt.pixelformat != host_fmt->pixelformat ||
				 fmt.width != host_fmt->width || fmt.height != host_fmt->height)) {
			LOG_INF("The host selected another format");
			return 0;
		}
	}
}

/* Wait until the host stops the stream or selects a format other than fmt */
static void app_wait_host_change(const struct video_format *fmt)
{
	struct video_format cur = {.type = VIDEO_BUF_TYPE_INPUT};

	while (video_get_format(uvc_dev, &cur) == 0 && cur.pixelformat == fmt->pixelformat &&
	       cur.width == fmt->width && cur.height == fmt->height) {
		k_sleep(K_MSEC(100));
	}
}

/* Stop the pipeline and gather all the buffers back */
static int app_stop_stream(void)
{
	struct video_buffer *vbuf;
	k_timepoint_t end = sys_timepoint_calc(K_SECONDS(1));
	uint32_t count = 0;
	int ret;

	/* Stopping an endpoint also hands back the buffers queued in it */
	video_stream_stop(video_dev, VIDEO_BUF_TYPE_OUTPUT);

	if (use_videoenc) {
		video_stream_stop(videoenc_dev, VIDEO_BUF_TYPE_INPUT);
		video_stream_stop(videoenc_dev, VIDEO_BUF_TYPE_OUTPUT);
	}

	video_stream_stop(uvc_dev, VIDEO_BUF_TYPE_INPUT);

	/* Buffers in a USB transfer come back once it completes or is aborted */
	while (count < CONFIG_VIDEO_BUFFER_POOL_NUM_MAX && !sys_timepoint_expired(end)) {
		const struct {
			const struct device *dev;
			enum video_buf_type type;
		} ends[] = {
			{video_dev, VIDEO_BUF_TYPE_OUTPUT},
			{uvc_dev, VIDEO_BUF_TYPE_INPUT},
			{videoenc_dev, VIDEO_BUF_TYPE_INPUT},
			{videoenc_dev, VIDEO_BUF_TYPE_OUTPUT},
		};
		bool found = false;

		for (int i = 0; i < ARRAY_SIZE(ends); i++) {
			if (ends[i].dev == NULL) {
				continue;
			}

			vbuf = &(struct video_buffer){.type = ends[i].type};
			ret = video_dequeue(ends[i].dev, &vbuf, K_NO_WAIT);
			if (ret == 0) {
				count++;
				found = true;
			}
		}

		if (!found) {
			k_sleep(K_MSEC(1));
		}
	}

	if (count < CONFIG_VIDEO_BUFFER_POOL_NUM_MAX) {
		LOG_ERR("Only %u of %u buffers came back", count, CONFIG_VIDEO_BUFFER_POOL_NUM_MAX);
		return -EIO;
	}

	return 0;
}

int main(void)
{
	const struct device *uvc_src_dev = app_has_videoenc() ? videoenc_dev : video_dev;
	struct usbd_context *sample_usbd;
	struct video_format fmt = {0};
	struct video_frmival frmival = {0};
	struct k_poll_signal sig;
	struct k_poll_event evt[1];
	k_timeout_t timeout;
	bool failed;
	int ret;

	if (!device_is_ready(video_dev)) {
		LOG_ERR("video source %s failed to initialize", video_dev->name);
		return -ENODEV;
	}

	ret = video_get_caps(video_dev, &video_caps);
	if (ret != 0) {
		LOG_ERR("Unable to retrieve video capabilities");
		return 0;
	}

	if (app_has_videoenc()) {
		ret = app_init_videoenc(videoenc_dev);
		if (ret != 0) {
			return ret;
		}
	}

	/* Initialize control descriptors from the video device */
	uvc_device_init(uvc_dev, uvc_src_dev);

	/* Fill the table of formats */
	ret = app_add_filtered_formats();
	if (ret != 0) {
		return ret;
	}

	LOG_INF("Preparing %u buffers of %zu bytes", CONFIG_VIDEO_BUFFER_POOL_NUM_MAX,
		app_buf_size);

	for (int i = 0; i < ARRAY_SIZE(app_bufs); i++) {
		app_bufs[i] = video_buffer_aligned_alloc(app_buf_size,
							 CONFIG_VIDEO_BUFFER_POOL_ALIGN, K_NO_WAIT);
		if (app_bufs[i] == NULL) {
			LOG_ERR("Could not allocate the video buffer");
			return -ENOMEM;
		}
	}

	/* Prepare the UVC device for being run */
	ret = uvc_device_enable(uvc_dev);
	if (ret != 0) {
		return ret;
	}

	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		return -ENODEV;
	}

	ret = usbd_enable(sample_usbd);
	if (ret != 0) {
		return ret;
	}

	k_poll_signal_init(&sig);
	k_poll_event_init(&evt[0], K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &sig);

	while (true) {
		LOG_INF("Waiting the host to select the video format");

		while (true) {
			fmt.type = VIDEO_BUF_TYPE_INPUT;

			ret = video_get_format(uvc_dev, &fmt);
			if (ret == 0) {
				break;
			}
			if (ret != -EAGAIN) {
				LOG_ERR("Failed to get the video format");
				return ret;
			}

			k_sleep(K_MSEC(10));
		}

		ret = video_get_frmival(uvc_dev, &frmival);
		if (ret != 0) {
			LOG_ERR("Failed to get the video frame interval");
			return ret;
		}

		ret = app_start_stream(&fmt, &frmival, &sig, &timeout);
		if (ret == 0) {
			ret = app_run_stream(&fmt, evt, &sig, timeout);
		}
		failed = (ret != 0);

		ret = app_stop_stream();
		if (ret != 0) {
			return ret;
		}

		/* Do not retry a format that failed until the host asks for it again */
		if (failed) {
			LOG_ERR("Could not stream '%s' %ux%u", VIDEO_FOURCC_TO_STR(fmt.pixelformat),
				fmt.width, fmt.height);
			app_wait_host_change(&fmt);
		}
	}

	return 0;
}
