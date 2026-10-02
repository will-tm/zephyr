/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_H_
#define APP_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#define STREAM_WIDTH    640
#define STREAM_HEIGHT   480
#define SNAPSHOT_WIDTH  1600
#define SNAPSHOT_HEIGHT 1200

/* Largest streamed frame, a larger one is dropped */
#define STREAM_FRAME_MAX 131072

#define MAX_STREAMS 2

/* JPEG quantization scale of the sensor, lower is finer */
#define QS_MIN 5
#define QS_MAX 63

struct camera_settings {
	int32_t stream_qs;
	int32_t photo_qs;
	/* Sensor levels, -2 to 2 */
	int32_t contrast;
	int32_t brightness;
	int32_t saturation;
};

struct camera_stats {
	uint32_t width;
	uint32_t height;
	/* Over the last second */
	uint32_t fps_x10;
	uint32_t interval_us;
	uint32_t jitter_us;
	uint32_t frame_avg;
	uint32_t frame_max;
	/* Since boot */
	uint32_t frames;
	uint32_t bytes;
	uint32_t dropped;
	uint32_t snapshots;
	uint32_t snap_size;
	uint32_t snap_pause_ms;
};

int camera_start(void);
void camera_get_stats(struct camera_stats *stats);
void camera_get_settings(struct camera_settings *settings);
/* Apply settings from the camera thread, before its next frame */
int camera_set_settings(const struct camera_settings *settings);

/* Copy the next frame after *seq into buf, waiting for it up to timeout */
int camera_frame_copy(uint8_t *buf, size_t size, size_t *len, uint32_t *seq, k_timeout_t timeout);

/* Take a full resolution picture, valid until camera_snapshot_release() */
int camera_snapshot(const uint8_t **data, size_t *len);
void camera_snapshot_release(void);

int http_start(void);
void wifi_get_status(int *rssi, uint32_t *phy_x10, uint32_t *channel);

#endif /* APP_H_ */
