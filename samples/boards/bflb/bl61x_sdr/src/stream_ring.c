/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/crc.h>

#include "iq_format.h"
#include "stream_ring.h"

#define US_PER_S              1000000ULL
/* Re-anchor the timestamp after this many chunks of wall-clock discontinuity */
#define TIMESTAMP_REANCHOR_CHUNKS 8U
#define SOURCE_FORWARD_LIMIT  0x80000000U
#define IQ8_MAGIC_SUFFIX_POS  3U

static union stream_frame frames[IQ_STREAM_RING_CHUNKS] __aligned(4);
static struct k_spinlock lock;
static K_SEM_DEFINE(frames_sem, 0, IQ_STREAM_RING_CHUNKS);
static K_SEM_DEFINE(slots_sem, 0, 1);
static uint32_t ring_read;
static uint32_t ring_write;
static uint32_t ring_count;
static uint32_t ring_reserved;
static uint32_t stream_sequence = 1U;
static bool timestamp_valid;
static uint32_t timestamp_last_source;
static uint32_t timestamp_last_rate_hz;
static uint64_t timestamp_last_us;

static uint64_t iq_timestamp_us(const struct iq_chunk_report_meta *meta);
static void signal_frames(uint32_t count);
static void fill_iq_header(union stream_frame *frame, const struct iq_chunk_report_meta *meta);
static uint16_t *iq8_samples(union stream_frame *frame);
static void finish_iq8_frame(union stream_frame *frame);

static uint64_t iq_timestamp_us(const struct iq_chunk_report_meta *meta)
{
	const uint64_t now = k_cyc_to_us_floor64(k_cycle_get_64());
	uint64_t chunk_us;
	uint64_t timestamp = now;

	if (meta->sample_rate_hz == 0U) {
		return now;
	}

	chunk_us = (((uint64_t)IQ_CHUNK_SAMPLE_WORDS * US_PER_S) + (meta->sample_rate_hz / 2U)) /
		   meta->sample_rate_hz;
	if (timestamp_valid) {
		const uint32_t source_delta = meta->source_chunk_index - timestamp_last_source;
		const bool source_forward =
			(source_delta != 0U) && (source_delta < SOURCE_FORWARD_LIMIT);

		if (source_forward && (meta->sample_rate_hz == timestamp_last_rate_hz)) {
			timestamp = timestamp_last_us +
				    ((((uint64_t)source_delta * IQ_CHUNK_SAMPLE_WORDS * US_PER_S) +
				      (meta->sample_rate_hz / 2U)) /
				     meta->sample_rate_hz);
			/* A capture pause can leave the source counter frozen */
			if (now > (timestamp + (chunk_us * TIMESTAMP_REANCHOR_CHUNKS))) {
				timestamp = now;
			}
		} else {
			const uint64_t next = timestamp_last_us + chunk_us;

			if (timestamp < next) {
				timestamp = next;
			}
		}
	}

	timestamp_valid = true;
	timestamp_last_source = meta->source_chunk_index;
	timestamp_last_rate_hz = meta->sample_rate_hz;
	timestamp_last_us = timestamp;

	return timestamp;
}

static void signal_frames(uint32_t count)
{
	for (uint32_t i = 0U; i < count; i++) {
		k_sem_give(&frames_sem);
	}
}

static void fill_iq_header(union stream_frame *frame, const struct iq_chunk_report_meta *meta)
{
	uint32_t gain_changes = MIN(meta->agc_gain_changes, STREAM_FRAME_AGC_GAIN_CHANGES_MAX);
	uint32_t robust_peak = MIN(meta->agc_robust_peak, STREAM_FRAME_AGC_ROBUST_PEAK_MAX);
	k_spinlock_key_t key;

	memcpy(frame->iq.magic, STREAM_FRAME_MAGIC_IQ, STREAM_FRAME_MAGIC_BYTES);
	/* Shared with CFG1 reports pushed from the stream thread */
	key = k_spin_lock(&lock);
	frame->iq.sequence = stream_sequence;
	stream_sequence++;
	k_spin_unlock(&lock, key);
	frame->iq.source_chunk_index = meta->source_chunk_index;
	frame->iq.chunk_counter = (meta->sample_rate_hz != 0U) ? (uint32_t)iq_timestamp_us(meta)
							       : meta->source_chunk_index;
	frame->iq.adc_decimation = meta->adc_decimation;
	frame->iq.sample_rate_hz = meta->sample_rate_hz;
	frame->iq.center_freq_mhz = meta->center_freq_mhz;
	frame->iq.rx_gain = meta->rx_gain;
	frame->iq.flags = meta->agc_state & STREAM_FRAME_AGC_STATE_MASK;
	frame->iq.flags |= gain_changes << STREAM_FRAME_AGC_GAIN_CHANGES_S;
	frame->iq.flags |= robust_peak << STREAM_FRAME_AGC_ROBUST_PEAK_S;
	if (meta->software_agc_active) {
		frame->iq.flags |= STREAM_FRAME_FLAG_SOFTWARE_AGC_ACTIVE;
	}
	if (meta->sample_rate_hz != 0U) {
		frame->iq.flags |= STREAM_FRAME_FLAG_TIMESTAMP_US32;
	}
	frame->iq.dropped_chunks = meta->dropped_chunks;
	frame->iq.bank_timer_late_misses = meta->bank_timer_late_misses;
	frame->iq.bank_timer_write_ptr = meta->bank_timer_write_ptr;
	frame->iq.producer_wake_write_ptr = meta->producer_wake_write_ptr;
}

/* Frames are 4-byte aligned and samples sit at offset 52 */
static uint16_t *iq8_samples(union stream_frame *frame)
{
	return (uint16_t *)((uint8_t *)frame + offsetof(struct iq_chunk, samples));
}

static void finish_iq8_frame(union stream_frame *frame)
{
	uint8_t *crc = (uint8_t *)&iq8_samples(frame)[IQ_CHUNK_SAMPLE_WORDS];

	frame->iq.magic[IQ8_MAGIC_SUFFIX_POS] = '8';
	/* Wire CRC field directly after the packed samples, kept zero */
	memset(crc, 0, IQ_CHUNK_CRC_BYTES);
}

void stream_ring_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	ring_read = 0U;
	ring_write = 0U;
	ring_count = 0U;
	ring_reserved = 0U;
	k_sem_reset(&frames_sem);
	k_spin_unlock(&lock, key);
}

bool stream_ring_wait_frames(uint32_t timeout_ms)
{
	return k_sem_take(&frames_sem, K_MSEC(timeout_ms)) == 0;
}

bool stream_ring_wait_slots(uint32_t timeout_ms)
{
	return k_sem_take(&slots_sem, K_MSEC(timeout_ms)) == 0;
}

bool stream_ring_reserve_slots(uint32_t count, union stream_frame **slots)
{
	k_spinlock_key_t key;
	bool reserved = false;

	if ((count == 0U) || (count > IQ_STREAM_RING_CHUNKS)) {
		return false;
	}

	key = k_spin_lock(&lock);
	if ((IQ_STREAM_RING_CHUNKS - ring_count - ring_reserved) >= count) {
		for (uint32_t i = 0U; i < count; i++) {
			slots[i] = &frames[(ring_write + i) % IQ_STREAM_RING_CHUNKS];
		}
		ring_write = (ring_write + count) % IQ_STREAM_RING_CHUNKS;
		ring_reserved += count;
		reserved = true;
	}
	k_spin_unlock(&lock, key);

	return reserved;
}

void stream_ring_commit_reserved(uint32_t count)
{
	k_spinlock_key_t key;

	if (count == 0U) {
		return;
	}

	key = k_spin_lock(&lock);
	count = MIN(count, ring_reserved);
	ring_reserved -= count;
	ring_count += count;
	k_spin_unlock(&lock, key);
	signal_frames(count);
}

void stream_ring_release_reserved(uint32_t count)
{
	k_spinlock_key_t key;

	if (count == 0U) {
		return;
	}

	key = k_spin_lock(&lock);
	count = MIN(count, ring_reserved);
	ring_reserved -= count;
	ring_write = (ring_write + IQ_STREAM_RING_CHUNKS - count) % IQ_STREAM_RING_CHUNKS;
	k_spin_unlock(&lock, key);
}

uint32_t stream_ring_available_slots(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t available = IQ_STREAM_RING_CHUNKS - ring_count - ring_reserved;

	k_spin_unlock(&lock, key);

	return available;
}

bool stream_ring_push_config_report(const struct config_report *report)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	bool pushed = false;

	if ((ring_reserved == 0U) && (ring_count < IQ_STREAM_RING_CHUNKS)) {
		union stream_frame *frame = &frames[ring_write];

		frame->config = *report;
		frame->config.sequence = stream_sequence;
		stream_sequence++;
		ring_write = (ring_write + 1U) % IQ_STREAM_RING_CHUNKS;
		ring_count++;
		pushed = true;
	}
	k_spin_unlock(&lock, key);
	if (pushed) {
		signal_frames(1U);
	}

	return pushed;
}

void stream_ring_fill_iq_chunk(union stream_frame *frame, const struct iq_chunk_report_meta *meta,
			       const volatile uint32_t *source_words)
{
	fill_iq_header(frame, meta);
	for (uint32_t i = 0U; i < IQ_CHUNK_SAMPLE_WORDS; i++) {
		frame->iq.samples[i] = source_words[i];
	}
	frame->iq.crc32 = 0U;
}

void stream_ring_fill_iq_chunk_int8(union stream_frame *frame,
				    const struct iq_chunk_report_meta *meta,
				    const volatile uint32_t *source_words)
{
	uint16_t *packed = iq8_samples(frame);

	fill_iq_header(frame, meta);
	if (iq_format_is_default()) {
		for (uint32_t i = 0U; i < IQ_CHUNK_SAMPLE_WORDS; i++) {
			packed[i] = iq_format_iq8_default(source_words[i]);
		}
	} else {
		for (uint32_t i = 0U; i < IQ_CHUNK_SAMPLE_WORDS; i++) {
			packed[i] = iq_format_iq8(source_words[i]);
		}
	}
	finish_iq8_frame(frame);
}

union stream_frame *stream_ring_peek(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	union stream_frame *frame = NULL;

	if (ring_count != 0U) {
		frame = &frames[ring_read];
	}
	k_spin_unlock(&lock, key);

	return frame;
}

void stream_ring_pop(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	if (ring_count != 0U) {
		ring_read = (ring_read + 1U) % IQ_STREAM_RING_CHUNKS;
		ring_count--;
	}
	k_spin_unlock(&lock, key);
	k_sem_give(&slots_sem);
}

size_t stream_frame_wire_size(union stream_frame *frame)
{
	if (memcmp(frame->iq.magic, STREAM_FRAME_MAGIC_IQ, STREAM_FRAME_MAGIC_BYTES) == 0) {
		frame->iq.crc32 = 0U;
		return sizeof(frame->iq);
	}
	if (memcmp(frame->iq.magic, STREAM_FRAME_MAGIC_IQ8, STREAM_FRAME_MAGIC_BYTES) == 0) {
		return IQ8_FRAME_WIRE_BYTES;
	}
	if (memcmp(frame->config.magic, STREAM_FRAME_MAGIC_CONFIG, STREAM_FRAME_MAGIC_BYTES) ==
	    0) {
		frame->config.crc32 = crc32_ieee((const uint8_t *)&frame->config,
						 offsetof(struct config_report, crc32));
		return sizeof(frame->config);
	}

	return 0U;
}
