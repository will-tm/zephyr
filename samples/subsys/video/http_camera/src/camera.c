/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/video/video.h>

#include "app.h"

LOG_MODULE_REGISTER(camera, LOG_LEVEL_INF);

#define CAPTURE_BUFS  3
/* Frames the sensor takes to apply the exposure carried over to full resolution */
#define SNAPSHOT_SKIP 6

#define LOG_PERIOD    K_SECONDS(60)

static const struct device *const video_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));

static struct video_buffer *latest;
static size_t latest_len;
static uint32_t latest_seq;
static K_MUTEX_DEFINE(frame_lock);
static K_CONDVAR_DEFINE(frame_cond);

static struct video_buffer *snap;
static size_t snap_len;
static int snap_ret;
static K_MUTEX_DEFINE(snap_lock);
static K_SEM_DEFINE(snap_req, 0, 1);
static K_SEM_DEFINE(snap_done, 0, 1);

static struct camera_stats stats;
static K_MUTEX_DEFINE(stats_lock);

static struct camera_settings settings = {
	.stream_qs = 8,
	.photo_qs = 5,
};
static struct camera_settings pending;
static K_MUTEX_DEFINE(settings_lock);
static atomic_t settings_dirty;

/* Frame intervals of the second being measured */
static struct {
	int64_t start_us;
	int64_t last_us;
	uint32_t frames;
	uint64_t sum;
	uint64_t sum_sq;
	uint32_t bytes;
	uint32_t max;
} window;

static K_THREAD_STACK_DEFINE(camera_stack, 2048);
static struct k_thread camera_thread;

static int set_size(uint32_t width, uint32_t height, size_t *size)
{
	struct video_format fmt = {
		.type = VIDEO_BUF_TYPE_OUTPUT,
		.pixelformat = VIDEO_PIX_FMT_JPEG,
		.width = width,
		.height = height,
	};
	int ret;

	ret = video_set_format(video_dev, &fmt);
	if (ret < 0) {
		LOG_ERR("Cannot set %ux%u JPEG (%d)", width, height, ret);
		return ret;
	}

	if (size != NULL) {
		*size = fmt.size;
	}

	return 0;
}

/* Restart the stream at another size, with every buffer queued again */
static int switch_size(uint32_t width, uint32_t height)
{
	struct video_buffer *vbuf;
	int ret;

	video_stream_stop(video_dev, VIDEO_BUF_TYPE_OUTPUT);
	while (video_dequeue(video_dev, &vbuf, K_NO_WAIT) == 0) {
		video_enqueue(video_dev, vbuf);
	}

	ret = set_size(width, height, NULL);
	if (ret < 0) {
		return ret;
	}

	return video_stream_start(video_dev, VIDEO_BUF_TYPE_OUTPUT);
}

static void publish(const struct video_buffer *vbuf)
{
	if (vbuf->bytesused > latest->size) {
		LOG_WRN("Dropped a %u byte frame", vbuf->bytesused);
		stats.dropped++;
		return;
	}

	k_mutex_lock(&frame_lock, K_FOREVER);
	memcpy(latest->buffer, vbuf->buffer, vbuf->bytesused);
	latest_len = vbuf->bytesused;
	latest_seq++;
	k_condvar_broadcast(&frame_cond);
	k_mutex_unlock(&frame_lock);
}

static void set_ctrl(uint32_t id, int32_t val)
{
	struct video_control ctrl = {.id = id, .val = val};
	int ret;

	ret = video_set_ctrl(video_dev, &ctrl);
	if (ret < 0) {
		LOG_WRN("Cannot set control 0x%x to %d (%d)", id, val, ret);
	}
}

static void apply_settings(void)
{
	k_mutex_lock(&settings_lock, K_FOREVER);
	settings = pending;
	k_mutex_unlock(&settings_lock);

	/* The control holds the quantization scale itself */
	set_ctrl(VIDEO_CID_JPEG_COMPRESSION_QUALITY, settings.stream_qs);
	set_ctrl(VIDEO_CID_CONTRAST, settings.contrast);
	set_ctrl(VIDEO_CID_BRIGHTNESS, settings.brightness);
	set_ctrl(VIDEO_CID_SATURATION, settings.saturation);
}

/* Hold the exposure and gain of the stream, which the sensor carries over to the snapshot */
static void set_auto_exposure(bool enable)
{
	set_ctrl(VIDEO_CID_EXPOSURE, enable ? 1 : 0);
	set_ctrl(VIDEO_CID_GAIN, enable ? 1 : 0);
	set_ctrl(VIDEO_CID_WHITE_BALANCE_TEMPERATURE, enable ? 1 : 0);
}

static int take_snapshot(void)
{
	struct video_buffer *vbuf;
	int ret;

	set_auto_exposure(false);
	set_ctrl(VIDEO_CID_JPEG_COMPRESSION_QUALITY, settings.photo_qs);

	ret = switch_size(SNAPSHOT_WIDTH, SNAPSHOT_HEIGHT);
	if (ret < 0) {
		set_ctrl(VIDEO_CID_JPEG_COMPRESSION_QUALITY, settings.stream_qs);
		set_auto_exposure(true);
		return ret;
	}

	snap_len = 0;
	for (int i = 0; i <= SNAPSHOT_SKIP; i++) {
		ret = video_dequeue(video_dev, &vbuf, K_SECONDS(1));
		if (ret < 0) {
			LOG_ERR("No snapshot frame (%d)", ret);
			break;
		}

		if (i == SNAPSHOT_SKIP && vbuf->bytesused <= snap->size) {
			memcpy(snap->buffer, vbuf->buffer, vbuf->bytesused);
			snap_len = vbuf->bytesused;
		}

		video_enqueue(video_dev, vbuf);
	}

	if (switch_size(STREAM_WIDTH, STREAM_HEIGHT) < 0) {
		LOG_ERR("Cannot restore the stream");
	}

	set_ctrl(VIDEO_CID_JPEG_COMPRESSION_QUALITY, settings.stream_qs);
	set_auto_exposure(true);

	return snap_len > 0 ? 0 : -EIO;
}

static void account(uint32_t size)
{
	int64_t now = k_ticks_to_us_floor64(k_uptime_ticks());
	uint64_t interval;
	double mean;

	if (window.last_us != 0) {
		interval = now - window.last_us;
		window.frames++;
		window.sum += interval;
		window.sum_sq += interval * interval;
		window.bytes += size;
		window.max = MAX(window.max, size);
	} else {
		window.start_us = now;
	}
	window.last_us = now;

	k_mutex_lock(&stats_lock, K_FOREVER);
	stats.frames++;
	stats.bytes += size;

	if (now - window.start_us >= USEC_PER_SEC && window.frames > 0) {
		mean = (double)window.sum / window.frames;
		stats.fps_x10 = (uint32_t)(10.0 * USEC_PER_SEC / mean + 0.5);
		stats.interval_us = (uint32_t)mean;
		stats.jitter_us = (uint32_t)sqrt(MAX((double)window.sum_sq / window.frames -
						     mean * mean, 0.0));
		stats.frame_avg = window.bytes / window.frames;
		stats.frame_max = window.max;

		window.start_us = now;
		window.frames = 0;
		window.sum = 0;
		window.sum_sq = 0;
		window.bytes = 0;
		window.max = 0;
	}
	k_mutex_unlock(&stats_lock);
}

static void camera_run(void *p1, void *p2, void *p3)
{
	k_timepoint_t next_log = sys_timepoint_calc(LOG_PERIOD);
	int64_t paused = 0;
	struct video_buffer *vbuf;

	for (;;) {
		if (atomic_cas(&settings_dirty, 1, 0)) {
			apply_settings();
		}

		if (k_sem_take(&snap_req, K_NO_WAIT) == 0) {
			paused = k_uptime_get();
			snap_ret = take_snapshot();
			k_sem_give(&snap_done);
		}

		if (video_dequeue(video_dev, &vbuf, K_MSEC(100)) < 0) {
			continue;
		}

		if (paused != 0) {
			k_mutex_lock(&stats_lock, K_FOREVER);
			stats.snapshots++;
			stats.snap_size = snap_len;
			stats.snap_pause_ms = k_uptime_get() - paused;
			k_mutex_unlock(&stats_lock);
			LOG_INF("Snapshot of %zu bytes, stream paused for %u ms", snap_len,
				stats.snap_pause_ms);
			/* Keep the pause out of the frame interval statistics */
			window.last_us = 0;
			window.frames = 0;
			window.sum = 0;
			window.sum_sq = 0;
			window.bytes = 0;
			window.max = 0;
			paused = 0;
		}

		if (vbuf->bytesused > 0) {
			publish(vbuf);
			account(vbuf->bytesused);
		}

		video_enqueue(video_dev, vbuf);

		if (sys_timepoint_expired(next_log)) {
			LOG_INF("%u.%u fps, %u bytes per frame", stats.fps_x10 / 10,
				stats.fps_x10 % 10, stats.frame_avg);
			next_log = sys_timepoint_calc(LOG_PERIOD);
		}
	}
}

void camera_get_settings(struct camera_settings *out)
{
	k_mutex_lock(&settings_lock, K_FOREVER);
	*out = atomic_get(&settings_dirty) != 0 ? pending : settings;
	k_mutex_unlock(&settings_lock);
}

int camera_set_settings(const struct camera_settings *in)
{
	if (!IN_RANGE(in->stream_qs, QS_MIN, QS_MAX) || !IN_RANGE(in->photo_qs, QS_MIN, QS_MAX) ||
	    !IN_RANGE(in->contrast, -2, 2) || !IN_RANGE(in->brightness, -2, 2) ||
	    !IN_RANGE(in->saturation, -2, 2)) {
		return -EINVAL;
	}

	k_mutex_lock(&settings_lock, K_FOREVER);
	pending = *in;
	atomic_set(&settings_dirty, 1);
	k_mutex_unlock(&settings_lock);

	return 0;
}

void camera_get_stats(struct camera_stats *out)
{
	k_mutex_lock(&stats_lock, K_FOREVER);
	*out = stats;
	k_mutex_unlock(&stats_lock);
}

int camera_frame_copy(uint8_t *buf, size_t size, size_t *len, uint32_t *seq, k_timeout_t timeout)
{
	k_mutex_lock(&frame_lock, K_FOREVER);

	while (latest_seq == *seq) {
		if (k_condvar_wait(&frame_cond, &frame_lock, timeout) < 0) {
			k_mutex_unlock(&frame_lock);
			return -EAGAIN;
		}
	}

	*len = MIN(latest_len, size);
	memcpy(buf, latest->buffer, *len);
	*seq = latest_seq;

	k_mutex_unlock(&frame_lock);

	return 0;
}

int camera_snapshot(const uint8_t **data, size_t *len)
{
	k_mutex_lock(&snap_lock, K_FOREVER);
	k_sem_give(&snap_req);
	k_sem_take(&snap_done, K_FOREVER);

	if (snap_ret < 0) {
		k_mutex_unlock(&snap_lock);
		return snap_ret;
	}

	*data = snap->buffer;
	*len = snap_len;

	return 0;
}

void camera_snapshot_release(void)
{
	k_mutex_unlock(&snap_lock);
}

int camera_start(void)
{
	struct video_buffer *vbuf;
	size_t size;
	int ret;

	if (!device_is_ready(video_dev)) {
		LOG_ERR("%s not ready", video_dev->name);
		return -ENODEV;
	}

	/* Every capture buffer has to hold a snapshot frame */
	ret = set_size(SNAPSHOT_WIDTH, SNAPSHOT_HEIGHT, &size);
	if (ret < 0) {
		return ret;
	}

	snap = video_buffer_alloc(size, K_NO_WAIT);
	latest = video_buffer_alloc(STREAM_FRAME_MAX, K_NO_WAIT);
	if (snap == NULL || latest == NULL) {
		LOG_ERR("Cannot allocate frame buffers");
		return -ENOMEM;
	}

	for (int i = 0; i < CAPTURE_BUFS; i++) {
		vbuf = video_buffer_alloc(size, K_NO_WAIT);
		if (vbuf == NULL) {
			LOG_ERR("Cannot allocate capture buffers");
			return -ENOMEM;
		}

		vbuf->type = VIDEO_BUF_TYPE_OUTPUT;
		video_enqueue(video_dev, vbuf);
	}

	ret = set_size(STREAM_WIDTH, STREAM_HEIGHT, NULL);
	if (ret < 0) {
		return ret;
	}

	pending = settings;
	apply_settings();

	stats.width = STREAM_WIDTH;
	stats.height = STREAM_HEIGHT;

	ret = video_stream_start(video_dev, VIDEO_BUF_TYPE_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Cannot start the stream (%d)", ret);
		return ret;
	}

	k_thread_create(&camera_thread, camera_stack, K_THREAD_STACK_SIZEOF(camera_stack),
			camera_run, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	k_thread_name_set(&camera_thread, "camera");

	return 0;
}
