/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include "capture.h"
#include "dump_ring.h"
#include "iq_usb.h"
#include "rf.h"
#include "stream_ring.h"

#define HZ_PER_MHZ                1000000U
#define RF_FREQ_DEFAULT_HZ        (RF_FREQ_DEFAULT_MHZ * HZ_PER_MHZ)
#define RF_FREQ_MIN_HZ            (RF_FREQ_MIN_MHZ * HZ_PER_MHZ)
#define RF_FREQ_MAX_HZ            (RF_FREQ_MAX_MHZ * HZ_PER_MHZ)
#define TX_GAIN_DEFAULT           8U
#define BW_DEFAULT_MHZ            20U
#define LOOPBACK_TX_GAIN_DEFAULT  124U
#define LOOPBACK_RX_GAIN_DEFAULT  0x73U
#define LOOPBACK_BB_GAIN_DEFAULT  0x3fU
#define TX_TONE0_STEP_DEFAULT     16
#define TRIGGER_INTERVAL_DEFAULT  2501U
#define TRIGGER_DURATION_DEFAULT  1U
#define RX_FILTER_BW_OPEN         0U
#define RX_FILTER_BW_MIN_MHZ      13U
#define RX_FILTER_BW_MAX_MHZ      54U
#define RX_FILTER_MODE_DEFAULT    16U
#define RX_FILTER_MODE_MAX        32U
#define RX_FILTER_DCAP_MAX        63U
#define RX_FILTER_NARROW_DCAP     60U
#define WIFI_DUMMY_TX_INTERVAL_MS 1000U
#define IQ_AGC_TRIGGER_STATE_MASK_ALL 0xffffU
#define IQ_AGC_TRIGGER_MATCH_FSM      0U
#define IQ_AGC_TRIGGER_MATCH_MAX_GAIN 1U
#define TRIGGER_CFG_INTERVAL      0U
#define TRIGGER_CFG_OFFSET        1U
#define TRIGGER_CFG_DURATION      2U
#define TRIGGER_CFG_CHECKS        0U
#define TRIGGER_CFG_PRE           1U
#define TRIGGER_CFG_POST          2U
#define TRIGGER_CFG_LEVEL         3U
#define TRIGGER_CFG_STATE_MASK    4U
#define TRIGGER_CFG_DC_SHIFT      4U
#define TRIGGER_CFG_MATCH         5U
#define STALE_APPLY_US            500000U
#define PIPELINE_IDLE_TRIES       100U
#define PIPELINE_IDLE_SLEEP_MS    1
#define ARM_DELAY_MS              100

enum {
	CONFIG_STAGE_APPLY_START = 1U,
	CONFIG_STAGE_AFTER_WIFI = 2U,
	CONFIG_STAGE_BEFORE_MODEM_APPLY = 3U,
	CONFIG_STAGE_AFTER_MODEM_APPLY = 4U,
	CONFIG_STAGE_BEFORE_RESTART = 6U,
	CONFIG_STAGE_AFTER_RESTART = 7U,
	CONFIG_STAGE_RESTART_ENTRY = 20U,
	CONFIG_STAGE_AFTER_ENGINE_ENABLE = 21U,
	CONFIG_STAGE_AFTER_STREAM_STATE = 25U,
};

static struct capture_config config = {
	.radio = {.rf_freq_hz = RF_FREQ_DEFAULT_HZ},
	.gain = {.gain_mode = GAIN_MODE_MANUAL,
		 .rx_gain = RF_GAIN_DEFAULT,
		 .tx_gain = TX_GAIN_DEFAULT},
	.bandwidth = {.bw_mhz = BW_DEFAULT_MHZ, .second_chan = SECOND_CHAN_NONE},
	.loopback = {.loopback_tx_gain = LOOPBACK_TX_GAIN_DEFAULT,
		     .loopback_rx_gain = LOOPBACK_RX_GAIN_DEFAULT,
		     .loopback_bb_gain = LOOPBACK_BB_GAIN_DEFAULT},
	.tx = {.tx_tone0_step = TX_TONE0_STEP_DEFAULT},
	.iq_engine = {.adc_decimation = 1U, .adc_source_sel = 0U},
	.trigger = {.trigger_mode = IQ_TRIGGER_MODE_INTERVAL,
		    .trigger_config = {TRIGGER_INTERVAL_DEFAULT, 0U, TRIGGER_DURATION_DEFAULT}},
	.rx_filter = {.filter_bw_mhz = RX_FILTER_BW_OPEN,
		      .rx_filter_mode = RX_FILTER_MODE_DEFAULT,
		      .rx_filter_dcap = RX_FILTER_NARROW_DCAP},
	.wifi_tx = {.wifi_dummy_tx_interval_ms = WIFI_DUMMY_TX_INTERVAL_MS},
	.dc_offset = {.automatic = 0U},
};
static struct trigger_plan plan;
static struct capture_config pending_config;
static struct trigger_plan pending_plan;
static bool apply_pending;
static bool apply_in_progress;
static uint64_t apply_started_us;
static atomic_t capture_armed;
/* A UART snapshot owns the dump window and the RF tuning */
static bool uart_owner;
static struct k_spinlock lock;

static void arm_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(arm_work, arm_work_handler);

static uint64_t now_us(void);
static struct capture_config sanitize(struct capture_config c);
static struct trigger_plan build_trigger_plan(const struct capture_config *c);
static bool consume_pending(void);
static void finish_apply(void);
static void recover_stale_apply(void);
static void push_stage_report(const struct capture_config *c, uint32_t stage);
static void wait_pipeline_idle(void);
static void engine_disable(void);
static void restart_capture(const struct capture_config *c);
static void apply_pending_config(void);

static uint64_t now_us(void)
{
	return k_cyc_to_us_floor64(k_cycle_get_64());
}

static void arm_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	iq_usb_stream_armed_set(true);
}

static struct capture_config sanitize(struct capture_config c)
{
	c.radio.rf_freq_hz = CLAMP(c.radio.rf_freq_hz, RF_FREQ_MIN_HZ, RF_FREQ_MAX_HZ);
	/* rfc_config_channel() resolution is 1 MHz */
	c.radio.rf_freq_hz = ((c.radio.rf_freq_hz + (HZ_PER_MHZ / 2U)) / HZ_PER_MHZ) * HZ_PER_MHZ;
	c.bandwidth.bw_mhz = BW_DEFAULT_MHZ;
	c.bandwidth.second_chan = SECOND_CHAN_NONE;
	c.gain.tx_gain = MIN(c.gain.tx_gain, TX_GAIN_MAX);
	c.dc_offset.automatic = (c.dc_offset.automatic != 0U) ? 1U : 0U;
	if ((c.rx_filter.rx_filter_override == 0U) &&
	    (c.rx_filter.filter_bw_mhz != RX_FILTER_BW_OPEN)) {
		c.rx_filter.filter_bw_mhz = CLAMP(c.rx_filter.filter_bw_mhz, RX_FILTER_BW_MIN_MHZ,
						  RX_FILTER_BW_MAX_MHZ);
	}
	c.rx_filter.rx_filter_dcap = MIN(c.rx_filter.rx_filter_dcap, RX_FILTER_DCAP_MAX);
	c.rx_filter.rx_filter_mode = MIN(c.rx_filter.rx_filter_mode, RX_FILTER_MODE_MAX);

	return c;
}

static struct trigger_plan build_trigger_plan(const struct capture_config *c)
{
	struct trigger_plan p = {
		.mode = c->trigger.trigger_mode,
		.interval_chunks = c->trigger.trigger_config[TRIGGER_CFG_INTERVAL],
		.interval_duration_chunks = c->trigger.trigger_config[TRIGGER_CFG_DURATION],
		.checks_per_chunk = 1U,
	};
	uint32_t sample_stride;

	if (p.interval_chunks == 0U) {
		p.interval_chunks = 1U;
	}
	p.interval_offset_chunks =
		c->trigger.trigger_config[TRIGGER_CFG_OFFSET] % p.interval_chunks;

	if (c->trigger.trigger_mode == IQ_TRIGGER_MODE_AGC) {
		uint32_t match_mode = c->trigger.trigger_config[TRIGGER_CFG_MATCH];

		p.checks_per_chunk = c->trigger.trigger_config[TRIGGER_CFG_CHECKS];
		p.pre_chunks = c->trigger.trigger_config[TRIGGER_CFG_PRE];
		p.post_chunks = c->trigger.trigger_config[TRIGGER_CFG_POST];
		p.max_gain = c->trigger.trigger_config[TRIGGER_CFG_LEVEL];
		p.state_mask = c->trigger.trigger_config[TRIGGER_CFG_STATE_MASK] &
			       IQ_AGC_TRIGGER_STATE_MASK_ALL;
		p.use_fsm_match = match_mode != IQ_AGC_TRIGGER_MATCH_MAX_GAIN;
		p.use_max_gain_match = match_mode != IQ_AGC_TRIGGER_MATCH_FSM;
	} else if (c->trigger.trigger_mode == IQ_TRIGGER_MODE_POWER) {
		p.checks_per_chunk = c->trigger.trigger_config[TRIGGER_CFG_CHECKS];
		p.pre_chunks = c->trigger.trigger_config[TRIGGER_CFG_PRE];
		p.post_chunks = c->trigger.trigger_config[TRIGGER_CFG_POST];
		p.threshold = c->trigger.trigger_config[TRIGGER_CFG_LEVEL];
		p.dc_shift = c->trigger.trigger_config[TRIGGER_CFG_DC_SHIFT];
	} else {
		/* interval mode uses only the interval fields */
	}
	p.checks_per_chunk = CLAMP(p.checks_per_chunk, 1U, IQ_AGC_TRIGGER_MAX_CHECKS_PER_CHUNK);
	sample_stride = MAX(IQ_CHUNK_SAMPLE_WORDS / p.checks_per_chunk, 1U);
	for (uint32_t check = 0U; check < p.checks_per_chunk; check++) {
		p.sample_offsets[check] = (uint16_t)(check * sample_stride);
	}

	return p;
}

static bool consume_pending(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	bool pending = apply_pending && !apply_in_progress;

	if (pending) {
		config = pending_config;
		plan = pending_plan;
		apply_pending = false;
		apply_in_progress = true;
		apply_started_us = now_us();
	}
	k_spin_unlock(&lock, key);

	return pending;
}

static void finish_apply(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	apply_in_progress = false;
	apply_started_us = 0U;
	k_spin_unlock(&lock, key);
}

static void recover_stale_apply(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	if (apply_in_progress && (apply_started_us != 0U) &&
	    ((now_us() - apply_started_us) > STALE_APPLY_US)) {
		apply_in_progress = false;
		apply_started_us = 0U;
	}
	k_spin_unlock(&lock, key);
}

static void push_stage_report(const struct capture_config *c, uint32_t stage)
{
	struct config_report report = {
		.magic = {'C', 'F', 'G', '1'},
		.uptime_ms = k_uptime_get_32(),
		.stage = stage,
		.rx_gain = c->gain.rx_gain,
		.loopback = c->loopback.loopback,
		.tx_tone_enable = c->tx.tx_tone_enable,
		.adc_source_sel = c->iq_engine.adc_source_sel,
		.expert_gain_word0 = c->gain.expert_gain_word0,
		.expert_gain_word1 = c->gain.expert_gain_word1,
		.ctrl = rf_dump_ctrl_read(),
		.mode = rf_dump_cfg_read(),
		.hp_tcm_dump_ctrl = 0U,
		.trigger_interval_chunks = c->trigger.trigger_config[TRIGGER_CFG_INTERVAL],
		.trigger_duration_chunks = c->trigger.trigger_config[TRIGGER_CFG_DURATION],
	};

	(void)stream_ring_push_config_report(&report);
}

static void wait_pipeline_idle(void)
{
	for (uint32_t i = 0U; (i < PIPELINE_IDLE_TRIES) &&
			      (iq_usb_stream_write_active() || dump_ring_producer_active());
	     i++) {
		k_msleep(PIPELINE_IDLE_SLEEP_MS);
	}
}

static void engine_disable(void)
{
	rf_ring_stop();
}

static void restart_capture(const struct capture_config *c)
{
	push_stage_report(c, CONFIG_STAGE_RESTART_ENTRY);
	dump_ring_restart_tracking();
	capture_engine_enable();
	push_stage_report(c, CONFIG_STAGE_AFTER_ENGINE_ENABLE);
	wait_pipeline_idle();
	dump_ring_stream_state_reset();
	push_stage_report(c, CONFIG_STAGE_AFTER_STREAM_STATE);
}

static void apply_pending_config(void)
{
	static struct capture_config applied;
	static bool have_applied;
	struct capture_config c;
	struct trigger_plan p;

	capture_snapshot(&c, &p);
	if (have_applied && (memcmp(&applied, &c, sizeof(c)) == 0) && capture_engine_running()) {
		return;
	}

	push_stage_report(&c, CONFIG_STAGE_APPLY_START);
	wait_pipeline_idle();
	/* Retuning touches the RF block the dump engine reads from */
	engine_disable();
	push_stage_report(&c, CONFIG_STAGE_AFTER_WIFI);
	push_stage_report(&c, CONFIG_STAGE_BEFORE_MODEM_APPLY);
	if (c.gain.gain_mode != GAIN_MODE_AUTO) {
		rf_gain_set(c.gain.rx_gain);
	}
	rf_tune(c.radio.rf_freq_hz / HZ_PER_MHZ);
	rf_dump_sel_set(c.iq_engine.adc_source_sel);
	push_stage_report(&c, CONFIG_STAGE_AFTER_MODEM_APPLY);
	push_stage_report(&c, CONFIG_STAGE_BEFORE_RESTART);
	restart_capture(&c);
	push_stage_report(&c, CONFIG_STAGE_AFTER_RESTART);
	applied = c;
	have_applied = true;
}

void capture_get_config(struct capture_config *out)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	*out = config;
	k_spin_unlock(&lock, key);
}

void capture_snapshot(struct capture_config *out_config, struct trigger_plan *out_plan)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	*out_config = config;
	*out_plan = plan;
	k_spin_unlock(&lock, key);
}

const char *capture_validate_config(const struct capture_config *c)
{
	if (c->gain.gain_mode > GAIN_MODE_EXPERT) {
		return "gain_mode must be 0 (auto), 1 (manual), or 2 (expert)";
	}
	if ((c->gain.gain_mode == GAIN_MODE_MANUAL) && (c->gain.rx_gain >= GAIN_TABLE_ENTRIES)) {
		return "manual rx_gain exceeds the calibrated gain table";
	}
	if (c->gain.tx_gain > TX_GAIN_MAX) {
		return "tx_gain must be in the range 0..63";
	}
	if ((c->radio.frequency_correction_ppb < -RF_CORRECTION_MAX_PPB) ||
	    (c->radio.frequency_correction_ppb > RF_CORRECTION_MAX_PPB)) {
		return "frequency_correction_ppb must be in the range -100000..100000";
	}
	if (c->dc_offset.automatic > 1U) {
		return "dc_offset automatic must be 0 or 1";
	}
	/* BL61x dump words carry no AGC state */
	if (c->trigger.trigger_mode == IQ_TRIGGER_MODE_AGC) {
		return "AGC trigger is not supported on BL61x";
	}
	if (c->trigger.trigger_mode > IQ_TRIGGER_MODE_POWER) {
		return "trigger_mode must be 0 (interval), 1 (agc), or 2 (power)";
	}

	return NULL;
}

void capture_apply_config(const struct capture_config *c)
{
	struct capture_config sanitized = sanitize(*c);
	struct trigger_plan p = build_trigger_plan(&sanitized);
	k_spinlock_key_t key = k_spin_lock(&lock);

	pending_config = sanitized;
	pending_plan = p;
	apply_pending = true;
	k_spin_unlock(&lock, key);
}

void capture_service_pending_config(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	bool uart_busy = uart_owner;

	k_spin_unlock(&lock, key);
	/* Deferred until the UART snapshot releases the dump window */
	if (uart_busy) {
		return;
	}

	recover_stale_apply();
	if (consume_pending()) {
		apply_pending_config();
		finish_apply();
	}
	if ((atomic_get(&capture_armed) == 0) && capture_engine_running()) {
		wait_pipeline_idle();
		engine_disable();
		dump_ring_stream_state_reset();
	}
}

bool capture_config_applying(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	bool applying = apply_pending || apply_in_progress;

	k_spin_unlock(&lock, key);

	return applying;
}

/*
 * Stream ownership: a fresh engine start is serialized in the stream thread
 * and the stream is armed only after it and its reports have completed.
 */
void capture_set_armed(bool armed)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	atomic_set(&capture_armed, armed ? 1 : 0);
	k_spin_unlock(&lock, key);
	if (armed) {
		struct capture_config c;

		/* Keep a configuration the host queued just before arming */
		key = k_spin_lock(&lock);
		c = apply_pending ? pending_config : config;
		k_spin_unlock(&lock, key);
		capture_apply_config(&c);
		(void)k_work_reschedule(&arm_work, K_MSEC(ARM_DELAY_MS));
	} else {
		(void)k_work_cancel_delayable(&arm_work);
		iq_usb_stream_armed_set(false);
	}
}

bool capture_uart_claim(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	bool claimed = (atomic_get(&capture_armed) == 0) && !uart_owner &&
		       !apply_pending && !apply_in_progress && !capture_engine_running();

	if (claimed) {
		uart_owner = true;
	}
	k_spin_unlock(&lock, key);

	return claimed;
}

void capture_uart_release(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	uart_owner = false;
	k_spin_unlock(&lock, key);
}

bool capture_engine_running(void)
{
	return rf_ring_running();
}

void capture_engine_enable(void)
{
	rf_ring_start();
}

uint32_t capture_decimation(const struct capture_config *c)
{
	return CLAMP(c->iq_engine.adc_decimation, 1U, ADC_DECIMATION_MAX);
}

uint32_t capture_sample_rate_hz(const struct capture_config *c)
{
	return RF_SAMPLE_RATE_HZ / capture_decimation(c);
}
