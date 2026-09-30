/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Stream frame queue between the dump ring producer and the USB transport.
 * Frame layouts and semantics follow the ESP-SDR ringbuffer (IQC1, IQC8 and
 * CFG1 frames).
 */

#ifndef BL61X_SDR_STREAM_RING_H_
#define BL61X_SDR_STREAM_RING_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#define IQ_CHUNK_SAMPLE_WORDS 1024U
/* Internal RAM budget; the ESP firmware keeps 256 frames in PSRAM */
#define IQ_STREAM_RING_CHUNKS 16U

#define STREAM_FRAME_MAGIC_IQ     "IQC1"
#define STREAM_FRAME_MAGIC_IQ8    "IQC8"
#define STREAM_FRAME_MAGIC_CONFIG "CFG1"
#define STREAM_FRAME_MAGIC_BYTES  4U

#define STREAM_FRAME_FLAG_TIMESTAMP_US32      BIT(31)
#define STREAM_FRAME_FLAG_SOFTWARE_AGC_ACTIVE BIT(30)
#define STREAM_FRAME_FLAG_DCOC_ACTIVE         BIT(29)
#define STREAM_FRAME_AGC_ROBUST_PEAK_S        20U
#define STREAM_FRAME_AGC_ROBUST_PEAK_MAX      0x1ffU
#define STREAM_FRAME_AGC_GAIN_CHANGES_S       4U
#define STREAM_FRAME_AGC_GAIN_CHANGES_MAX     0xffffU
#define STREAM_FRAME_AGC_STATE_MASK           0x0fU

#define IQ_CHUNK_HEADER_BYTES 52U
#define IQ_CHUNK_CRC_BYTES    4U
#define IQ8_BYTES_PER_SAMPLE  2U
#define IQ8_FRAME_WIRE_BYTES                                                                       \
	(IQ_CHUNK_HEADER_BYTES + (IQ8_BYTES_PER_SAMPLE * IQ_CHUNK_SAMPLE_WORDS) +                  \
	 IQ_CHUNK_CRC_BYTES)

struct iq_chunk {
	char magic[4];
	uint32_t sequence;
	uint32_t source_chunk_index;
	uint32_t chunk_counter; /* timestamp_us32 when flags bit 31 is set */
	uint32_t adc_decimation;
	uint32_t sample_rate_hz;
	uint32_t center_freq_mhz;
	uint32_t rx_gain;
	uint32_t flags;
	uint32_t dropped_chunks;
	uint32_t bank_timer_late_misses;
	uint32_t bank_timer_write_ptr;
	uint32_t producer_wake_write_ptr;
	uint32_t samples[IQ_CHUNK_SAMPLE_WORDS];
	uint32_t crc32;
} __packed;

struct config_report {
	char magic[4];
	uint32_t sequence;
	uint32_t uptime_ms;
	uint32_t stage;
	uint32_t rx_gain;
	uint32_t loopback;
	uint32_t tx_tone_enable;
	uint32_t adc_source_sel;
	uint32_t expert_gain_word0;
	uint32_t expert_gain_word1;
	uint32_t ctrl;
	uint32_t mode;
	uint32_t hp_tcm_dump_ctrl;
	uint32_t trigger_interval_chunks;
	uint32_t trigger_duration_chunks;
	uint32_t crc32;
} __packed;

union stream_frame {
	struct iq_chunk iq;
	struct config_report config;
};

BUILD_ASSERT(sizeof(struct iq_chunk) == 4152U, "IQC1 wire size changed");
BUILD_ASSERT(offsetof(struct iq_chunk, samples) == IQ_CHUNK_HEADER_BYTES,
	     "IQC1 header size changed");

struct iq_chunk_report_meta {
	uint32_t source_chunk_index;
	uint32_t adc_decimation;
	uint32_t sample_rate_hz;
	uint32_t center_freq_mhz;
	uint32_t rx_gain;
	uint32_t agc_state;
	bool software_agc_active;
	uint32_t agc_robust_peak;
	uint32_t agc_gain_changes;
	uint32_t dropped_chunks;
	uint32_t bank_timer_late_misses;
	uint32_t bank_timer_write_ptr;
	uint32_t producer_wake_write_ptr;
};

void stream_ring_reset(void);
/* Block up to timeout_ms until a producer commits a frame */
bool stream_ring_wait_frames(uint32_t timeout_ms);
/* Block up to timeout_ms until the consumer frees a slot */
bool stream_ring_wait_slots(uint32_t timeout_ms);
bool stream_ring_reserve_slots(uint32_t count, union stream_frame **slots);
void stream_ring_commit_reserved(uint32_t count);
void stream_ring_release_reserved(uint32_t count);
uint32_t stream_ring_available_slots(void);
bool stream_ring_push_config_report(const struct config_report *report);
void stream_ring_fill_iq_chunk(union stream_frame *frame, const struct iq_chunk_report_meta *meta,
			       const volatile uint32_t *source_words);
void stream_ring_fill_iq_chunk_int8(union stream_frame *frame,
				    const struct iq_chunk_report_meta *meta,
				    const volatile uint32_t *source_words);
union stream_frame *stream_ring_peek(void);
void stream_ring_pop(void);
size_t stream_frame_wire_size(union stream_frame *frame);

#endif /* BL61X_SDR_STREAM_RING_H_ */
