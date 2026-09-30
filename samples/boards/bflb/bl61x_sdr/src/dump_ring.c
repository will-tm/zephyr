/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Dump ring consumer, ported from the ESP-SDR streaming firmware.
 *
 * The dump engine writes continuously into a ring of RF_RING_WORDS words.
 * The producer turns the hardware write position into an absolute 64-bit
 * write count, emits fixed chunks that are at least a guard distance behind
 * the writer, skips half a ring and counts drops when it falls too far
 * behind, and queues the chunks as stream frames.
 *
 * Single core adaptation: waits longer than two kernel ticks sleep instead of
 * spinning (ending two ticks early, since a sleep may overshoot), and the
 * producer blocks on the frame queue when it is full, so the transport
 * thread keeps running.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "capture.h"
#include "dump_ring.h"
#include "iq_format.h"
#include "iq_usb.h"
#include "rf.h"
#include "stream_ring.h"

#define RING_GUARD_WORDS          (12U * IQ_CHUNK_SAMPLE_WORDS)
#define LIVE_COPY_GUARD_WORDS     256U
#define RING_MAX_CHUNKS_PER_POLL  16U
#define POLL_CLOSE_US             20U
#define POLL_WAKE_MARGIN_US       30000U
#define POLL_WAKE_SLACK_US        1000U
#define US_PER_MS                 1000U
#define US_PER_S                  1000000ULL
#define TICK_US                   (USEC_PER_SEC / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
/* k_usleep() may overshoot by up to two ticks */
#define SLEEP_SLACK_US            (2U * TICK_US)
#define IDLE_DELAY_MS             2
#define APPLY_DELAY_MS            1
#define SLOT_WAIT_MS              1U
#define DC_Q16_SHIFT              16U
#define DC_Q16_SCALE              65536
#define CHUNKS_PER_WINDOW         (RF_RING_WORDS / IQ_CHUNK_SAMPLE_WORDS)
/* Software AGC in auto gain mode: one chunk measured every interval */
#define AGC_INTERVAL_US           2000U

#define PRODUCER_STACK_SIZE       2048
#define PRODUCER_PRIORITY         3

BUILD_ASSERT((RF_RING_WORDS % IQ_CHUNK_SAMPLE_WORDS) == 0U, "dump ring must be chunk-aligned");

enum {
	CHUNK_STREAM_SKIP = 0U,
	CHUNK_STREAM_TRIGGER = 1U,
	CHUNK_STREAM_PRE = 2U,
	CHUNK_STREAM_POST = 3U,
};

static uint32_t emit_chunk;
static bool ring_started;
static uint32_t source_chunk_index;
static uint32_t dropped_chunks;
static uint64_t abs_write_accum;
static uint32_t prev_sa;
static uint64_t abs_track_us;
static uint32_t dbg_lo;
static uint64_t diag_abs_write;
static uint32_t copy_source_chunk;
static bool power_dc_valid;
static int32_t power_dc_i_q16;
static int32_t power_dc_q_q16;
static volatile bool producer_active;
static uint8_t chunk_mark[RING_MAX_CHUNKS_PER_POLL];
static uint32_t decim_buf[IQ_CHUNK_SAMPLE_WORDS];
static uint64_t agc_last_us;
static uint32_t agc_gain_changes;

static void producer_thread(void *p1, void *p2, void *p3);
K_THREAD_DEFINE(producer_tid, PRODUCER_STACK_SIZE, producer_thread, NULL, NULL, NULL,
		PRODUCER_PRIORITY, 0, SYS_FOREVER_MS);

static uint64_t now_us(void);
static uint32_t ring_mod(uint32_t value);
static uint32_t ring_delta(uint32_t cur, uint32_t prev);
static uint32_t abs_u32(int32_t value);
static uint32_t output_chunk_words(const struct capture_config *config);
static uint32_t ring_guard_words(const struct trigger_plan *plan);
static uint32_t interval_next_selected(const struct trigger_plan *plan, uint32_t chunk);
static uint32_t interval_selected_count(const struct trigger_plan *plan, uint32_t first_chunk,
					uint32_t end_chunk);
static bool interval_prev_selected(const struct trigger_plan *plan, uint32_t end_chunk,
				   uint32_t *selected_chunk);
static bool chunk_triggers(const struct trigger_plan *plan, const uint32_t *words, int32_t *sum_i,
			   int32_t *sum_q, uint32_t *sum_n, int32_t dc_i, int32_t dc_q,
			   bool dc_valid);
static void expand_pre_post(uint8_t *mask, uint32_t n, uint32_t pre, uint32_t post);
static uint64_t current_abs_write(void);
static const uint32_t *chunk_words(const struct capture_config *config, uint32_t start_word);
static const uint32_t *chunk_source(const struct capture_config *config, uint32_t c,
				    uint64_t abs_write);
static const uint32_t *exact_chunk_source(const struct capture_config *config,
					  uint32_t source_chunk);
static void fill_and_push_chunk(const struct capture_config *config, const uint32_t *words,
				uint32_t source_chunk, uint32_t sample_rate_hz, uint32_t rx_gain,
				uint32_t agc_state, union stream_frame *slot);
static void process_interval_exact(const struct capture_config *config,
				   const struct trigger_plan *plan, uint64_t abs_write,
				   uint32_t c_hi, uint32_t sample_rate_hz, uint32_t rx_gain);
static void process_interval_latest(const struct capture_config *config,
				    const struct trigger_plan *plan, uint64_t abs_write,
				    uint32_t c_hi, uint32_t sample_rate_hz, uint32_t rx_gain);
static void process_triggered(const struct capture_config *config,
			      const struct trigger_plan *plan, uint64_t abs_write, uint32_t c_hi,
			      uint32_t sample_rate_hz, uint32_t rx_gain);
static void process_dump_ring(const struct capture_config *config,
			      const struct trigger_plan *plan);
static void agc_service(const struct capture_config *config);
static void wait_until_next_ring_window(const struct capture_config *config,
					const struct trigger_plan *plan);

static uint64_t now_us(void)
{
	return k_cyc_to_us_floor64(k_cycle_get_64());
}

static uint32_t ring_mod(uint32_t value)
{
	return value % RF_RING_WORDS;
}

static uint32_t ring_delta(uint32_t cur, uint32_t prev)
{
	return (cur + RF_RING_WORDS - prev) % RF_RING_WORDS;
}

static uint32_t abs_u32(int32_t value)
{
	return (value < 0) ? (0U - (uint32_t)value) : (uint32_t)value;
}

static uint32_t output_chunk_words(const struct capture_config *config)
{
	return IQ_CHUNK_SAMPLE_WORDS * capture_decimation(config);
}

static uint32_t ring_guard_words(const struct trigger_plan *plan)
{
	if ((plan->mode == IQ_TRIGGER_MODE_INTERVAL) && (plan->interval_chunks == 1U) &&
	    (plan->interval_duration_chunks != 0U)) {
		return LIVE_COPY_GUARD_WORDS;
	}

	return RING_GUARD_WORDS;
}

static uint32_t interval_next_selected(const struct trigger_plan *plan, uint32_t chunk)
{
	uint32_t position;

	if ((plan->interval_chunks == 0U) || (plan->interval_duration_chunks == 0U)) {
		return chunk;
	}
	if (chunk < plan->interval_offset_chunks) {
		return plan->interval_offset_chunks;
	}

	position = (chunk - plan->interval_offset_chunks) % plan->interval_chunks;
	if (position < plan->interval_duration_chunks) {
		return chunk;
	}

	return chunk + (plan->interval_chunks - position);
}

static uint32_t interval_selected_count(const struct trigger_plan *plan, uint32_t first_chunk,
					uint32_t end_chunk)
{
	uint32_t duration;
	uint64_t before_first = 0U;
	uint64_t before_end = 0U;
	uint64_t count;

	if (end_chunk <= first_chunk) {
		return 0U;
	}
	if ((plan->interval_chunks == 0U) || (plan->interval_duration_chunks == 0U)) {
		return end_chunk - first_chunk;
	}

	duration = MIN(plan->interval_duration_chunks, plan->interval_chunks);
	if (first_chunk > plan->interval_offset_chunks) {
		uint32_t rel = first_chunk - plan->interval_offset_chunks;

		before_first = (uint64_t)(rel / plan->interval_chunks) * duration;
		before_first += MIN(rel % plan->interval_chunks, duration);
	}
	if (end_chunk > plan->interval_offset_chunks) {
		uint32_t rel = end_chunk - plan->interval_offset_chunks;

		before_end = (uint64_t)(rel / plan->interval_chunks) * duration;
		before_end += MIN(rel % plan->interval_chunks, duration);
	}
	count = before_end - before_first;

	return (count > UINT32_MAX) ? UINT32_MAX : (uint32_t)count;
}

static bool interval_prev_selected(const struct trigger_plan *plan, uint32_t end_chunk,
				   uint32_t *selected_chunk)
{
	uint32_t duration;
	uint32_t period_index;
	uint32_t period_start;
	uint32_t selected_end;

	if (end_chunk == 0U) {
		return false;
	}
	if ((plan->interval_chunks == 0U) || (plan->interval_duration_chunks == 0U)) {
		*selected_chunk = end_chunk - 1U;
		return true;
	}
	if (end_chunk <= plan->interval_offset_chunks) {
		return false;
	}

	duration = MIN(plan->interval_duration_chunks, plan->interval_chunks);
	period_index = (end_chunk - plan->interval_offset_chunks - 1U) / plan->interval_chunks;
	period_start = plan->interval_offset_chunks + (period_index * plan->interval_chunks);
	selected_end = MIN(period_start + duration, end_chunk);
	if (selected_end > period_start) {
		*selected_chunk = selected_end - 1U;
		return true;
	}
	if (period_index == 0U) {
		return false;
	}
	period_start -= plan->interval_chunks;
	*selected_chunk = period_start + duration - 1U;

	return true;
}

/* Scans one chunk's samples; returns true if it triggers (power mode) */
static bool chunk_triggers(const struct trigger_plan *plan, const uint32_t *words, int32_t *sum_i,
			   int32_t *sum_q, uint32_t *sum_n, int32_t dc_i, int32_t dc_q,
			   bool dc_valid)
{
	bool found = false;

	for (uint32_t check = 0U; check < plan->checks_per_chunk; check++) {
		uint32_t raw = words[plan->sample_offsets[check]];
		int32_t i = iq_format_i10(raw);
		int32_t q = iq_format_q10(raw);
		uint32_t mag = abs_u32(i - dc_i) + abs_u32(q - dc_q);

		*sum_i += i;
		*sum_q += q;
		(*sum_n)++;
		if (dc_valid && (mag >= plan->threshold)) {
			found = true;
		}
	}

	return found;
}

static void expand_pre_post(uint8_t *mask, uint32_t n, uint32_t pre, uint32_t post)
{
	uint32_t last_trig = UINT32_MAX;
	uint32_t next_trig = UINT32_MAX;

	for (uint32_t i = 0U; i < n; i++) {
		if (mask[i] == CHUNK_STREAM_TRIGGER) {
			last_trig = i;
		} else if ((last_trig != UINT32_MAX) && ((i - last_trig) <= post) &&
			   (mask[i] == CHUNK_STREAM_SKIP)) {
			mask[i] = CHUNK_STREAM_POST;
		} else {
			/* outside the post window */
		}
	}
	for (uint32_t j = n; j > 0U; j--) {
		uint32_t i = j - 1U;

		if (mask[i] == CHUNK_STREAM_TRIGGER) {
			next_trig = i;
		} else if ((next_trig != UINT32_MAX) && ((next_trig - i) <= pre) &&
			   (mask[i] == CHUNK_STREAM_SKIP)) {
			mask[i] = CHUNK_STREAM_PRE;
		} else {
			/* outside the pre window */
		}
	}
}

/*
 * Accumulates the writer position. When more than half a ring may have
 * passed since the last poll, the elapsed time resolves the wrap count.
 */
static uint64_t current_abs_write(void)
{
	uint64_t now = now_us();
	uint32_t raw_sa = rf_ring_store_addr_raw();
	uint32_t sa = ring_mod(raw_sa);
	uint32_t delta_mod = ring_delta(sa, prev_sa);
	uint32_t delta = delta_mod;

	if (abs_track_us != 0U) {
		uint64_t expected = ((now - abs_track_us) * RF_SAMPLE_RATE_HZ) / US_PER_S;

		if (expected > (RF_RING_WORDS / 2U)) {
			uint64_t wraps = 0U;
			uint64_t delta64;

			if (expected > delta_mod) {
				wraps = (expected - delta_mod + (RF_RING_WORDS / 2U)) /
					RF_RING_WORDS;
			}
			delta64 = (uint64_t)delta_mod + (wraps * RF_RING_WORDS);
			delta = (delta64 > UINT32_MAX) ? UINT32_MAX : (uint32_t)delta64;
		}
	}

	abs_write_accum += delta;
	prev_sa = sa;
	abs_track_us = now;
	dbg_lo = raw_sa;
	diag_abs_write = abs_write_accum;

	return abs_write_accum;
}

static const uint32_t *chunk_words(const struct capture_config *config, uint32_t start_word)
{
	volatile uint32_t *ring = rf_ring_base();
	uint32_t decimation = capture_decimation(config);
	uint32_t source = ring_mod(start_word);

	/* The ring view is non-cached: no maintenance before reading */
	if (decimation == 1U) {
		return (const uint32_t *)&ring[source];
	}

	for (uint32_t i = 0U; i < IQ_CHUNK_SAMPLE_WORDS; i++) {
		decim_buf[i] = ring[source];
		source += decimation;
		if (source >= RF_RING_WORDS) {
			source -= RF_RING_WORDS;
		}
	}

	return decim_buf;
}

static const uint32_t *chunk_source(const struct capture_config *config, uint32_t c,
				    uint64_t abs_write)
{
	uint32_t output_words = output_chunk_words(config);
	uint64_t start_abs = (uint64_t)c * output_words;
	uint32_t start_word = (uint32_t)(start_abs % RF_RING_WORDS);
	uint32_t physical_chunk = (start_word / IQ_CHUNK_SAMPLE_WORDS) % CHUNKS_PER_WINDOW;
	uint64_t abs_chunk = abs_write / output_words;
	int32_t physical_delta =
		(int32_t)physical_chunk - (int32_t)(uint32_t)(abs_chunk % CHUNKS_PER_WINDOW);
	int64_t source_abs;

	if (physical_delta > 0) {
		physical_delta -= (int32_t)CHUNKS_PER_WINDOW;
	}
	source_abs = (int64_t)abs_chunk + physical_delta;
	copy_source_chunk = (source_abs > 0) ? (uint32_t)source_abs : 0U;

	return chunk_words(config, start_word);
}

static const uint32_t *exact_chunk_source(const struct capture_config *config,
					  uint32_t source_chunk)
{
	uint64_t start_abs = (uint64_t)source_chunk * output_chunk_words(config);

	copy_source_chunk = source_chunk;

	return chunk_words(config, (uint32_t)(start_abs % RF_RING_WORDS));
}

static void fill_and_push_chunk(const struct capture_config *config, const uint32_t *words,
				uint32_t source_chunk, uint32_t sample_rate_hz, uint32_t rx_gain,
				uint32_t agc_state, union stream_frame *slot)
{
	struct iq_chunk_report_meta meta = {
		.source_chunk_index = source_chunk,
		.adc_decimation = capture_decimation(config),
		.sample_rate_hz = sample_rate_hz,
		.center_freq_mhz = rf_freq_mhz(),
		.rx_gain = rx_gain,
		.agc_state = agc_state,
		.software_agc_active = config->gain.gain_mode == GAIN_MODE_AUTO,
		.agc_gain_changes = agc_gain_changes,
		.dropped_chunks = dropped_chunks,
		.bank_timer_late_misses = 0U,
		.bank_timer_write_ptr = dbg_lo,
		.producer_wake_write_ptr = rf_ring_store_addr_raw(),
	};

	if (iq_usb_stream_format() == IQ_USB_FORMAT_INT8) {
		stream_ring_fill_iq_chunk_int8(slot, &meta, words);
	} else {
		stream_ring_fill_iq_chunk(slot, &meta, words);
	}
}

/* Gapless interval: every chunk from the oldest one still in the ring */
static void process_interval_exact(const struct capture_config *config,
				   const struct trigger_plan *plan, uint64_t abs_write,
				   uint32_t c_hi, uint32_t sample_rate_hz, uint32_t rx_gain)
{
	uint32_t output_words = output_chunk_words(config);
	uint64_t low_abs = (abs_write > RF_RING_WORDS) ? (abs_write - RF_RING_WORDS + output_words)
						       : 0U;
	uint32_t low_chunk = (uint32_t)((low_abs + output_words - 1U) / output_words);
	union stream_frame *slots[RING_MAX_CHUNKS_PER_POLL];
	uint32_t n;

	ARG_UNUSED(plan);

	if (emit_chunk < low_chunk) {
		dropped_chunks += low_chunk - emit_chunk;
		emit_chunk = low_chunk;
	}
	if (emit_chunk >= c_hi) {
		source_chunk_index = emit_chunk;
		return;
	}

	n = MIN(MIN(c_hi - emit_chunk, RING_MAX_CHUNKS_PER_POLL), stream_ring_available_slots());
	if (n == 0U) {
		source_chunk_index = emit_chunk;
		return;
	}
	if (!stream_ring_reserve_slots(n, slots)) {
		return;
	}
	for (uint32_t i = 0U; i < n; i++) {
		uint32_t source_chunk = emit_chunk + i;

		fill_and_push_chunk(config, exact_chunk_source(config, source_chunk), source_chunk,
				    sample_rate_hz, rx_gain, 0U, slots[i]);
	}
	stream_ring_commit_reserved(n);
	emit_chunk += n;
	source_chunk_index = emit_chunk;
}

/* Sparse interval: only the newest selected chunk, older ones count as drops */
static void process_interval_latest(const struct capture_config *config,
				    const struct trigger_plan *plan, uint64_t abs_write,
				    uint32_t c_hi, uint32_t sample_rate_hz, uint32_t rx_gain)
{
	uint32_t output_words = output_chunk_words(config);
	uint64_t low_abs = (abs_write > RF_RING_WORDS) ? (abs_write - RF_RING_WORDS + output_words)
						       : 0U;
	uint32_t low_chunk = (uint32_t)((low_abs + output_words - 1U) / output_words);
	union stream_frame *slots[1];
	uint32_t latest;

	if (!interval_prev_selected(plan, c_hi, &latest)) {
		return;
	}
	if (latest < low_chunk) {
		dropped_chunks += interval_selected_count(plan, emit_chunk, low_chunk);
		emit_chunk = interval_next_selected(plan, low_chunk);
		source_chunk_index = emit_chunk;
		return;
	}
	if (latest < emit_chunk) {
		source_chunk_index = emit_chunk;
		return;
	}

	dropped_chunks += interval_selected_count(plan, emit_chunk, latest);
	if (!stream_ring_reserve_slots(1U, slots)) {
		dropped_chunks++;
		return;
	}
	fill_and_push_chunk(config, exact_chunk_source(config, latest), latest, sample_rate_hz,
			    rx_gain, 0U, slots[0]);
	stream_ring_commit_reserved(1U);
	emit_chunk = interval_next_selected(plan, latest + 1U);
	source_chunk_index = emit_chunk;
}

/* Power trigger: marks triggering chunks and their pre/post neighbours */
static void process_triggered(const struct capture_config *config,
			      const struct trigger_plan *plan, uint64_t abs_write, uint32_t c_hi,
			      uint32_t sample_rate_hz, uint32_t rx_gain)
{
	union stream_frame *slots[RING_MAX_CHUNKS_PER_POLL];
	uint32_t n = MIN(c_hi - emit_chunk, RING_MAX_CHUNKS_PER_POLL);
	int32_t dc_i = power_dc_i_q16 >> DC_Q16_SHIFT;
	int32_t dc_q = power_dc_q_q16 >> DC_Q16_SHIFT;
	int32_t sum_i = 0;
	int32_t sum_q = 0;
	uint32_t sum_n = 0U;
	uint32_t selected = 0U;
	uint32_t reserve;
	uint32_t filled = 0U;

	for (uint32_t k = 0U; k < n; k++) {
		const uint32_t *words = chunk_source(config, emit_chunk + k, abs_write);

		chunk_mark[k] = chunk_triggers(plan, words, &sum_i, &sum_q, &sum_n, dc_i, dc_q,
					       power_dc_valid)
					? CHUNK_STREAM_TRIGGER
					: CHUNK_STREAM_SKIP;
	}
	expand_pre_post(chunk_mark, n, plan->pre_chunks, plan->post_chunks);
	if (sum_n != 0U) {
		int32_t mean_i = (sum_i / (int32_t)sum_n) * DC_Q16_SCALE;
		int32_t mean_q = (sum_q / (int32_t)sum_n) * DC_Q16_SCALE;

		if (!power_dc_valid) {
			power_dc_i_q16 = mean_i;
			power_dc_q_q16 = mean_q;
			power_dc_valid = true;
		} else {
			power_dc_i_q16 += (mean_i - power_dc_i_q16) >> plan->dc_shift;
			power_dc_q_q16 += (mean_q - power_dc_q_q16) >> plan->dc_shift;
		}
	}

	for (uint32_t k = 0U; k < n; k++) {
		if (chunk_mark[k] != CHUNK_STREAM_SKIP) {
			selected++;
		}
	}
	reserve = MIN(selected, stream_ring_available_slots());
	dropped_chunks += selected - reserve;
	if ((reserve != 0U) && !stream_ring_reserve_slots(reserve, slots)) {
		dropped_chunks += reserve;
		reserve = 0U;
	}
	for (uint32_t k = 0U; (k < n) && (filled < reserve); k++) {
		if (chunk_mark[k] != CHUNK_STREAM_SKIP) {
			const uint32_t *words = chunk_source(config, emit_chunk + k, abs_write);

			fill_and_push_chunk(config, words, copy_source_chunk, sample_rate_hz,
					    rx_gain, 0U, slots[filled]);
			filled++;
		}
	}
	stream_ring_commit_reserved(filled);
	stream_ring_release_reserved(reserve - filled);
	emit_chunk += n;
	source_chunk_index = emit_chunk;
}

static void process_dump_ring(const struct capture_config *config,
			      const struct trigger_plan *plan)
{
	uint64_t abs_write = current_abs_write();
	uint32_t guard_words = ring_guard_words(plan);
	uint32_t output_words = output_chunk_words(config);
	uint32_t sample_rate_hz;
	uint32_t rx_gain;
	uint64_t hi;
	uint64_t emit_abs;
	uint32_t c_hi;

	if (abs_write <= guard_words) {
		return;
	}

	hi = abs_write - guard_words;
	c_hi = (uint32_t)(hi / output_words);
	if (!ring_started) {
		emit_chunk = (c_hi > 0U) ? (c_hi - 1U) : 0U;
		if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
			emit_chunk = interval_next_selected(plan, emit_chunk);
		}
		ring_started = true;
	}

	emit_abs = (uint64_t)emit_chunk * output_words;
	if ((hi > emit_abs) && ((hi - emit_abs) > (RF_RING_WORDS - guard_words))) {
		uint32_t new_emit = (uint32_t)((hi - (RF_RING_WORDS / 2U)) / output_words);

		if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
			dropped_chunks += interval_selected_count(plan, emit_chunk, new_emit);
			emit_chunk = interval_next_selected(plan, new_emit);
		} else {
			dropped_chunks += new_emit - emit_chunk;
			emit_chunk = new_emit;
		}
	}
	if (c_hi <= emit_chunk) {
		return;
	}

	sample_rate_hz = capture_sample_rate_hz(config);
	rx_gain = rf_gain_get();

	if (plan->mode != IQ_TRIGGER_MODE_INTERVAL) {
		process_triggered(config, plan, abs_write, c_hi, sample_rate_hz, rx_gain);
	} else if ((plan->interval_chunks == 1U) && (plan->interval_duration_chunks != 0U)) {
		process_interval_exact(config, plan, abs_write, c_hi, sample_rate_hz, rx_gain);
	} else {
		process_interval_latest(config, plan, abs_write, c_hi, sample_rate_hz, rx_gain);
	}
}

/* Measures the newest complete chunk behind the writer and adjusts the gain */
static void agc_service(const struct capture_config *config)
{
	uint64_t now = now_us();
	uint32_t start;

	if ((config->gain.gain_mode != GAIN_MODE_AUTO) || ((now - agc_last_us) < AGC_INTERVAL_US) ||
	    (diag_abs_write < (RING_GUARD_WORDS + IQ_CHUNK_SAMPLE_WORDS))) {
		return;
	}
	agc_last_us = now;
	start = ring_mod((uint32_t)(diag_abs_write - RING_GUARD_WORDS - IQ_CHUNK_SAMPLE_WORDS));
	start -= start % IQ_CHUNK_SAMPLE_WORDS;
	if (!rf_agc_update(&rf_ring_base()[start], IQ_CHUNK_SAMPLE_WORDS)) {
		agc_gain_changes++;
	}
}

static void wait_until_next_ring_window(const struct capture_config *config,
					const struct trigger_plan *plan)
{
	uint32_t next_chunk = emit_chunk;
	uint64_t ready_abs;
	uint64_t remaining_words;
	uint32_t remaining_us;

	if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
		next_chunk = interval_next_selected(plan, next_chunk);
	}
	ready_abs = (((uint64_t)next_chunk + 1U) * output_chunk_words(config)) +
		    ring_guard_words(plan);
	if (ready_abs <= diag_abs_write) {
		k_yield();
		return;
	}

	remaining_words = ready_abs - diag_abs_write;
	remaining_us = (uint32_t)MIN((remaining_words * US_PER_S) / RF_SAMPLE_RATE_HZ, UINT32_MAX);
	if (remaining_us > (POLL_WAKE_MARGIN_US + POLL_WAKE_SLACK_US)) {
		k_msleep((int32_t)((remaining_us - POLL_WAKE_MARGIN_US) / US_PER_MS));
	} else if (remaining_us > (SLEEP_SLACK_US + POLL_CLOSE_US)) {
		/* Wake early; the next pass busy-waits the remainder */
		k_usleep((int32_t)(remaining_us - SLEEP_SLACK_US));
	} else if (remaining_us > POLL_CLOSE_US) {
		k_busy_wait(remaining_us - POLL_CLOSE_US);
		k_yield();
	} else {
		k_busy_wait(POLL_CLOSE_US);
		k_yield();
	}
}

static void producer_thread(void *p1, void *p2, void *p3)
{
	struct capture_config config;
	struct trigger_plan plan;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		/* Set before the checks so wait_pipeline_idle() cannot miss a pass */
		producer_active = true;
		if (!iq_usb_stream_armed()) {
			producer_active = false;
			k_msleep(IDLE_DELAY_MS);
			continue;
		}
		if (capture_config_applying()) {
			producer_active = false;
			k_msleep(APPLY_DELAY_MS);
			continue;
		}
		capture_snapshot(&config, &plan);
		if (!capture_engine_running()) {
			capture_engine_enable();
			dump_ring_stream_state_reset();
			producer_active = false;
			k_msleep(APPLY_DELAY_MS);
			continue;
		}

		process_dump_ring(&config, &plan);
		agc_service(&config);
		producer_active = false;

		if (stream_ring_available_slots() == 0U) {
			(void)stream_ring_wait_slots(SLOT_WAIT_MS);
		} else {
			wait_until_next_ring_window(&config, &plan);
		}
	}
}

void dump_ring_init(void)
{
	k_thread_name_set(producer_tid, "iq_producer");
	k_thread_start(producer_tid);
}

void dump_ring_restart_tracking(void)
{
	prev_sa = 0U;
	abs_write_accum = 0U;
	abs_track_us = 0U;
}

void dump_ring_stream_state_reset(void)
{
	stream_ring_reset();
	dropped_chunks = 0U;
	source_chunk_index = 0U;
	emit_chunk = 0U;
	ring_started = false;
	abs_track_us = 0U;
	power_dc_valid = false;
	power_dc_i_q16 = 0;
	power_dc_q_q16 = 0;
	/* The engine may already be running: keep abs_write % ring equal to the
	 * physical write position so chunk offsets address the right words
	 */
	prev_sa = ring_mod(rf_ring_store_addr_raw());
	abs_write_accum = prev_sa;
	diag_abs_write = prev_sa;
}

bool dump_ring_producer_active(void)
{
	return producer_active;
}

uint32_t dump_ring_dropped_chunks(void)
{
	return dropped_chunks;
}

uint32_t dump_ring_source_chunk_index(void)
{
	return source_chunk_index;
}

uint64_t dump_ring_abs_write(void)
{
	return diag_abs_write;
}

uint32_t dump_ring_agc_gain_changes(void)
{
	return agc_gain_changes;
}
