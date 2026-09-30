/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Capture configuration and engine control, mirroring the ESP-SDR streaming
 * firmware: the host configuration document, the trigger plan and the
 * serialized configuration apply sequence.
 */

#ifndef BL61X_SDR_CAPTURE_H_
#define BL61X_SDR_CAPTURE_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/toolchain.h>

#include "rf.h"

#define IQ_TRIGGER_CONFIG_WORDS             16U
#define IQ_AGC_TRIGGER_MAX_CHECKS_PER_CHUNK 32U
#define ADC_DECIMATION_MAX                  10U
#define TX_GAIN_MAX                         63U
#define RF_CORRECTION_MAX_PPB               100000
/* Entries of the RX gain ladder (rf.h) */
#define GAIN_TABLE_ENTRIES                  RF_GAIN_STEPS

enum {
	GAIN_MODE_AUTO = 0U,
	GAIN_MODE_MANUAL = 1U,
	GAIN_MODE_EXPERT = 2U,
};

enum {
	IQ_TRIGGER_MODE_INTERVAL = 0U,
	IQ_TRIGGER_MODE_AGC = 1U,
	IQ_TRIGGER_MODE_POWER = 2U,
};

enum {
	SECOND_CHAN_NONE = 0U,
	SECOND_CHAN_ABOVE = 1U,
	SECOND_CHAN_BELOW = 2U,
};

struct capture_config {
	struct {
		uint32_t stream_wifi_packets;
	} stream;
	struct {
		uint32_t rf_freq_hz;
		int32_t frequency_correction_ppb;
	} radio;
	struct {
		uint32_t gain_mode;
		uint32_t rx_gain;
		uint32_t tx_gain;
		uint32_t expert_gain_word0;
		uint32_t expert_gain_word1;
		uint32_t expert_gain_word2;
	} gain;
	struct {
		uint32_t bw_mhz;
		uint32_t second_chan;
	} bandwidth;
	struct {
		uint32_t loopback;
		uint32_t loopback_tx_gain;
		uint32_t loopback_rx_gain;
		uint32_t loopback_bb_gain;
	} loopback;
	struct {
		uint32_t tx_tone_enable;
		int32_t tx_tone0_step;
	} tx;
	struct {
		uint32_t adc_decimation;
		uint32_t adc_source_sel;
	} iq_engine;
	struct {
		uint32_t trigger_mode;
		uint32_t trigger_config[IQ_TRIGGER_CONFIG_WORDS];
	} trigger;
	struct {
		uint32_t filter_bw_mhz;
		uint32_t rx_filter_override;
		uint32_t rx_filter_mode;
		uint32_t rx_filter_dcap;
	} rx_filter;
	struct {
		uint32_t wifi_dummy_tx_enable;
		uint32_t wifi_dummy_tx_interval_ms;
	} wifi_tx;
	struct {
		uint32_t automatic;
	} dc_offset;
};

BUILD_ASSERT(sizeof(struct capture_config) == 172U, "host CaptureConfig must match firmware");

struct trigger_plan {
	uint32_t mode;
	uint32_t interval_chunks;
	uint32_t interval_offset_chunks;
	uint32_t interval_duration_chunks;
	uint32_t checks_per_chunk;
	uint32_t pre_chunks;
	uint32_t post_chunks;
	uint32_t max_gain;
	uint32_t state_mask;
	uint32_t threshold;
	uint32_t dc_shift;
	bool use_fsm_match;
	bool use_max_gain_match;
	uint16_t sample_offsets[IQ_AGC_TRIGGER_MAX_CHECKS_PER_CHUNK];
};

void capture_get_config(struct capture_config *config);
void capture_snapshot(struct capture_config *config, struct trigger_plan *plan);
/* Validates a host configuration; returns NULL or an error message */
const char *capture_validate_config(const struct capture_config *config);
/* Queues a configuration for the serialized apply in the stream thread */
void capture_apply_config(const struct capture_config *config);
void capture_service_pending_config(void);
bool capture_config_applying(void);
void capture_set_armed(bool armed);
/*
 * Claims the dump window and the RF tuning for a UART command. Fails while a
 * USB host owns the stream or a configuration apply is pending.
 */
bool capture_uart_claim(void);
void capture_uart_release(void);
bool capture_engine_running(void);
void capture_engine_enable(void);
uint32_t capture_decimation(const struct capture_config *config);
uint32_t capture_sample_rate_hz(const struct capture_config *config);

#endif /* BL61X_SDR_CAPTURE_H_ */
