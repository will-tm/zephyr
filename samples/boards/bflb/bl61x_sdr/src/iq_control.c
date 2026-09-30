/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * JSON documents of the ESP-SDR control API. The configuration document
 * keeps the ESP-SDR shape: every key is accepted and echoed. On BL61x the
 * RF frequency, the dump source (adc_source_sel), software decimation,
 * triggers and the stream settings take effect; ESP32 radio specific keys
 * (loopback, TX tones, dummy Wi-Fi TX, filter overrides) are stored only.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/data/json.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "capture.h"
#include "dump_ring.h"
#include "iq_control.h"
#include "iq_usb.h"
#include "rf.h"

#define HOSTNAME        "bl61x-sdr.local"
#define NS_PER_US       1000ULL
#define RX_GAIN_MIN_DB  0U
#define RX_GAIN_STEP_DB 1U

struct cfg_stream {
	uint32_t stream_wifi_packets;
};

struct cfg_radio {
	uint32_t rf_freq_hz;
	int32_t frequency_correction_ppb;
};

struct cfg_gain {
	uint32_t gain_mode;
	uint32_t rx_gain;
	uint32_t tx_gain;
	uint32_t expert_gain_word0;
	uint32_t expert_gain_word1;
	uint32_t expert_gain_word2;
};

struct cfg_bandwidth {
	uint32_t bw_mhz;
	uint32_t second_chan;
};

struct cfg_loopback {
	uint32_t loopback;
	uint32_t loopback_tx_gain;
	uint32_t loopback_rx_gain;
	uint32_t loopback_bb_gain;
};

struct cfg_tx {
	uint32_t tx_tone_enable;
	int32_t tx_tone0_step;
};

struct cfg_iq_engine {
	uint32_t adc_decimation;
	uint32_t adc_source_sel;
};

struct cfg_trigger {
	uint32_t trigger_mode;
	uint32_t trigger_config[IQ_TRIGGER_CONFIG_WORDS];
	size_t trigger_config_len;
};

struct cfg_rx_filter {
	uint32_t filter_bw_mhz;
	uint32_t rx_filter_override;
	uint32_t rx_filter_mode;
	uint32_t rx_filter_dcap;
};

struct cfg_wifi_tx {
	uint32_t wifi_dummy_tx_enable;
	uint32_t wifi_dummy_tx_interval_ms;
};

struct cfg_dc_offset {
	uint32_t automatic;
};

struct cfg_doc {
	struct cfg_stream stream;
	struct cfg_radio radio;
	struct cfg_gain gain;
	struct cfg_bandwidth bandwidth;
	struct cfg_loopback loopback;
	struct cfg_tx tx;
	struct cfg_iq_engine iq_engine;
	struct cfg_trigger trigger;
	struct cfg_rx_filter rx_filter;
	struct cfg_wifi_tx wifi_tx;
	struct cfg_dc_offset dc_offset;
};

struct stream_start_doc {
	uint32_t stream_format;
};

static const struct json_obj_descr stream_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct cfg_stream, stream_wifi_packets, JSON_TOK_UINT),
};

static const struct json_obj_descr radio_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct cfg_radio, rf_freq_hz, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_radio, frequency_correction_ppb, JSON_TOK_NUMBER),
};

static const struct json_obj_descr gain_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct cfg_gain, gain_mode, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_gain, rx_gain, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_gain, tx_gain, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_gain, expert_gain_word0, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_gain, expert_gain_word1, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_gain, expert_gain_word2, JSON_TOK_UINT),
};

static const struct json_obj_descr bandwidth_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct cfg_bandwidth, bw_mhz, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_bandwidth, second_chan, JSON_TOK_UINT),
};

static const struct json_obj_descr loopback_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct cfg_loopback, loopback, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_loopback, loopback_tx_gain, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_loopback, loopback_rx_gain, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_loopback, loopback_bb_gain, JSON_TOK_UINT),
};

static const struct json_obj_descr tx_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct cfg_tx, tx_tone_enable, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_tx, tx_tone0_step, JSON_TOK_NUMBER),
};

static const struct json_obj_descr iq_engine_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct cfg_iq_engine, adc_decimation, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_iq_engine, adc_source_sel, JSON_TOK_UINT),
};

static const struct json_obj_descr trigger_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct cfg_trigger, trigger_mode, JSON_TOK_UINT),
	JSON_OBJ_DESCR_ARRAY(struct cfg_trigger, trigger_config, IQ_TRIGGER_CONFIG_WORDS,
			     trigger_config_len, JSON_TOK_UINT),
};

static const struct json_obj_descr rx_filter_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct cfg_rx_filter, filter_bw_mhz, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_rx_filter, rx_filter_override, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_rx_filter, rx_filter_mode, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_rx_filter, rx_filter_dcap, JSON_TOK_UINT),
};

static const struct json_obj_descr wifi_tx_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct cfg_wifi_tx, wifi_dummy_tx_enable, JSON_TOK_UINT),
	JSON_OBJ_DESCR_PRIM(struct cfg_wifi_tx, wifi_dummy_tx_interval_ms, JSON_TOK_UINT),
};

static const struct json_obj_descr dc_offset_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct cfg_dc_offset, automatic, JSON_TOK_UINT),
};

static const struct json_obj_descr cfg_doc_descr[] = {
	JSON_OBJ_DESCR_OBJECT(struct cfg_doc, stream, stream_descr),
	JSON_OBJ_DESCR_OBJECT(struct cfg_doc, radio, radio_descr),
	JSON_OBJ_DESCR_OBJECT(struct cfg_doc, gain, gain_descr),
	JSON_OBJ_DESCR_OBJECT(struct cfg_doc, bandwidth, bandwidth_descr),
	JSON_OBJ_DESCR_OBJECT(struct cfg_doc, loopback, loopback_descr),
	JSON_OBJ_DESCR_OBJECT(struct cfg_doc, tx, tx_descr),
	JSON_OBJ_DESCR_OBJECT(struct cfg_doc, iq_engine, iq_engine_descr),
	JSON_OBJ_DESCR_OBJECT(struct cfg_doc, trigger, trigger_descr),
	JSON_OBJ_DESCR_OBJECT(struct cfg_doc, rx_filter, rx_filter_descr),
	JSON_OBJ_DESCR_OBJECT(struct cfg_doc, wifi_tx, wifi_tx_descr),
	JSON_OBJ_DESCR_OBJECT(struct cfg_doc, dc_offset, dc_offset_descr),
};

static const struct json_obj_descr stream_start_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct stream_start_doc, stream_format, JSON_TOK_UINT),
};

static void doc_from_config(struct cfg_doc *doc, const struct capture_config *c);
static void config_from_doc(struct capture_config *c, const struct cfg_doc *doc);

static void doc_from_config(struct cfg_doc *doc, const struct capture_config *c)
{
	doc->stream.stream_wifi_packets = c->stream.stream_wifi_packets;
	doc->radio.rf_freq_hz = c->radio.rf_freq_hz;
	doc->radio.frequency_correction_ppb = c->radio.frequency_correction_ppb;
	doc->gain.gain_mode = c->gain.gain_mode;
	doc->gain.rx_gain = c->gain.rx_gain;
	doc->gain.tx_gain = c->gain.tx_gain;
	doc->gain.expert_gain_word0 = c->gain.expert_gain_word0;
	doc->gain.expert_gain_word1 = c->gain.expert_gain_word1;
	doc->gain.expert_gain_word2 = c->gain.expert_gain_word2;
	doc->bandwidth.bw_mhz = c->bandwidth.bw_mhz;
	doc->bandwidth.second_chan = c->bandwidth.second_chan;
	doc->loopback.loopback = c->loopback.loopback;
	doc->loopback.loopback_tx_gain = c->loopback.loopback_tx_gain;
	doc->loopback.loopback_rx_gain = c->loopback.loopback_rx_gain;
	doc->loopback.loopback_bb_gain = c->loopback.loopback_bb_gain;
	doc->tx.tx_tone_enable = c->tx.tx_tone_enable;
	doc->tx.tx_tone0_step = c->tx.tx_tone0_step;
	doc->iq_engine.adc_decimation = c->iq_engine.adc_decimation;
	doc->iq_engine.adc_source_sel = c->iq_engine.adc_source_sel;
	doc->trigger.trigger_mode = c->trigger.trigger_mode;
	memcpy(doc->trigger.trigger_config, c->trigger.trigger_config,
	       sizeof(doc->trigger.trigger_config));
	doc->trigger.trigger_config_len = 0U;
	doc->rx_filter.filter_bw_mhz = c->rx_filter.filter_bw_mhz;
	doc->rx_filter.rx_filter_override = c->rx_filter.rx_filter_override;
	doc->rx_filter.rx_filter_mode = c->rx_filter.rx_filter_mode;
	doc->rx_filter.rx_filter_dcap = c->rx_filter.rx_filter_dcap;
	doc->wifi_tx.wifi_dummy_tx_enable = c->wifi_tx.wifi_dummy_tx_enable;
	doc->wifi_tx.wifi_dummy_tx_interval_ms = c->wifi_tx.wifi_dummy_tx_interval_ms;
	doc->dc_offset.automatic = c->dc_offset.automatic;
}

static void config_from_doc(struct capture_config *c, const struct cfg_doc *doc)
{
	c->stream.stream_wifi_packets = doc->stream.stream_wifi_packets;
	c->radio.rf_freq_hz = doc->radio.rf_freq_hz;
	c->radio.frequency_correction_ppb = doc->radio.frequency_correction_ppb;
	c->gain.gain_mode = doc->gain.gain_mode;
	c->gain.rx_gain = doc->gain.rx_gain;
	c->gain.tx_gain = doc->gain.tx_gain;
	c->gain.expert_gain_word0 = doc->gain.expert_gain_word0;
	c->gain.expert_gain_word1 = doc->gain.expert_gain_word1;
	c->gain.expert_gain_word2 = doc->gain.expert_gain_word2;
	c->bandwidth.bw_mhz = doc->bandwidth.bw_mhz;
	c->bandwidth.second_chan = doc->bandwidth.second_chan;
	c->loopback.loopback = doc->loopback.loopback;
	c->loopback.loopback_tx_gain = doc->loopback.loopback_tx_gain;
	c->loopback.loopback_rx_gain = doc->loopback.loopback_rx_gain;
	c->loopback.loopback_bb_gain = doc->loopback.loopback_bb_gain;
	c->tx.tx_tone_enable = doc->tx.tx_tone_enable;
	c->tx.tx_tone0_step = doc->tx.tx_tone0_step;
	c->iq_engine.adc_decimation = doc->iq_engine.adc_decimation;
	c->iq_engine.adc_source_sel = doc->iq_engine.adc_source_sel;
	c->trigger.trigger_mode = doc->trigger.trigger_mode;
	memcpy(c->trigger.trigger_config, doc->trigger.trigger_config,
	       sizeof(c->trigger.trigger_config));
	c->rx_filter.filter_bw_mhz = doc->rx_filter.filter_bw_mhz;
	c->rx_filter.rx_filter_override = doc->rx_filter.rx_filter_override;
	c->rx_filter.rx_filter_mode = doc->rx_filter.rx_filter_mode;
	c->rx_filter.rx_filter_dcap = doc->rx_filter.rx_filter_dcap;
	c->wifi_tx.wifi_dummy_tx_enable = doc->wifi_tx.wifi_dummy_tx_enable;
	c->wifi_tx.wifi_dummy_tx_interval_ms = doc->wifi_tx.wifi_dummy_tx_interval_ms;
	c->dc_offset.automatic = doc->dc_offset.automatic;
}

int iq_control_build_config_json(char *text, size_t cap)
{
	struct capture_config c;
	const uint32_t *t;
	int n;

	capture_get_config(&c);
	t = c.trigger.trigger_config;
	n = snprintf(text, cap,
		     "{\"stream\":{\"stream_wifi_packets\":%" PRIu32 "},"
		     "\"radio\":{\"rf_freq_hz\":%" PRIu32 ",\"frequency_correction_ppb\":%" PRId32
		     "},"
		     "\"gain\":{\"gain_mode\":%" PRIu32 ",\"rx_gain\":%" PRIu32
		     ",\"tx_gain\":%" PRIu32 ",\"expert_gain_word0\":%" PRIu32
		     ",\"expert_gain_word1\":%" PRIu32 ",\"expert_gain_word2\":%" PRIu32 "},"
		     "\"bandwidth\":{\"bw_mhz\":%" PRIu32 ",\"second_chan\":%" PRIu32 "},"
		     "\"loopback\":{\"loopback\":%" PRIu32 ",\"loopback_tx_gain\":%" PRIu32
		     ",\"loopback_rx_gain\":%" PRIu32 ",\"loopback_bb_gain\":%" PRIu32 "},"
		     "\"tx\":{\"tx_tone_enable\":%" PRIu32 ",\"tx_tone0_step\":%" PRId32 "},"
		     "\"iq_engine\":{\"adc_decimation\":%" PRIu32 ",\"adc_source_sel\":%" PRIu32
		     "},"
		     "\"trigger\":{\"trigger_mode\":%" PRIu32 ",\"trigger_config\":[%" PRIu32
		     ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
		     ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
		     ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]},"
		     "\"rx_filter\":{\"filter_bw_mhz\":%" PRIu32 ",\"rx_filter_override\":%" PRIu32
		     ",\"rx_filter_mode\":%" PRIu32 ",\"rx_filter_dcap\":%" PRIu32 "},"
		     "\"wifi_tx\":{\"wifi_dummy_tx_enable\":%" PRIu32
		     ",\"wifi_dummy_tx_interval_ms\":%" PRIu32 "},"
		     "\"dc_offset\":{\"automatic\":%" PRIu32 "}}",
		     c.stream.stream_wifi_packets, c.radio.rf_freq_hz,
		     c.radio.frequency_correction_ppb, c.gain.gain_mode, c.gain.rx_gain,
		     c.gain.tx_gain, c.gain.expert_gain_word0, c.gain.expert_gain_word1,
		     c.gain.expert_gain_word2, c.bandwidth.bw_mhz, c.bandwidth.second_chan,
		     c.loopback.loopback, c.loopback.loopback_tx_gain, c.loopback.loopback_rx_gain,
		     c.loopback.loopback_bb_gain, c.tx.tx_tone_enable, c.tx.tx_tone0_step,
		     c.iq_engine.adc_decimation, c.iq_engine.adc_source_sel, c.trigger.trigger_mode,
		     t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7], t[8], t[9], t[10], t[11],
		     t[12], t[13], t[14], t[15], c.rx_filter.filter_bw_mhz,
		     c.rx_filter.rx_filter_override, c.rx_filter.rx_filter_mode,
		     c.rx_filter.rx_filter_dcap, c.wifi_tx.wifi_dummy_tx_enable,
		     c.wifi_tx.wifi_dummy_tx_interval_ms, c.dc_offset.automatic);

	return ((n > 0) && ((size_t)n < cap)) ? n : -1;
}

int iq_control_build_status_json(char *text, size_t cap)
{
	struct capture_config c;
	uint32_t decimation;
	uint64_t hardware_time_ns;
	int n;

	capture_get_config(&c);
	decimation = capture_decimation(&c);
	hardware_time_ns = k_cyc_to_us_floor64(k_cycle_get_64()) * NS_PER_US;
	n = snprintf(
		text, cap,
		"{\"hostname\":\"" HOSTNAME "\",\"link_up\":false,"
		"\"has_ipv4\":false,\"ipv4\":\"0.0.0.0\",\"streaming\":%s,"
		"\"rx_sample_rate_hz\":%" PRIu32 ",\"rx_decimation\":%" PRIu32 ","
		"\"stream_owner\":\"%s\",\"usb_mounted\":%s,"
		"\"usb_frames\":%" PRIu32 ",\"usb_send_errors\":%" PRIu32
		",\"usb_stream_format\":%" PRIu32 ",\"stream_format\":%" PRIu32
		",\"usb_tx_uploads\":0,\"usb_tx_errors\":0"
		",\"usb_tx_backpressure_retries\":0,\"usb_tx_commit_rejections\":0,"
		"\"config_applying\":%s,\"stream_epoch\":%" PRIu32 ",\"reset_reason\":0"
		",\"hardware_time_ns\":%" PRIu64
		",\"udp_frames\":0,\"udp_datagrams\":0,\"udp_bytes\":0,\"udp_send_errors\":0"
		",\"tx_udp_datagrams\":0,\"tx_udp_bytes\":0,\"tx_udp_errors\":0"
		",\"tx_udp_stale_datagrams\":0,\"tx_udp_backpressure_retries\":0"
		",\"tx_udp_commit_rejections\":0,\"tx_udp_commits\":0,\"tx_udp_armed\":false"
		",\"tx_replay\":{\"words\":0,\"rate_code\":0,\"segments\":0,\"total_cycles\":0"
		",\"sample_cycles\":0,\"gap_cycles\":0,\"maximum_gap_cycles\":0"
		",\"tcm_stage_copy_max_cycles\":0,\"deadline_late_max_cycles\":0"
		",\"deadline_late_max_word\":0,\"requested_start_time_ns\":0"
		",\"actual_start_time_ns\":0,\"start_error_ns\":0,\"queue_underflow\":false"
		",\"deadline_missed\":false}"
		",\"firmware_dropped_chunks\":%" PRIu32 ",\"source_chunk_index\":%" PRIu32
		",\"dcoc_diag\":0,\"dc_offset_automatic\":%s,\"dcoc_active\":false"
		",\"software_agc\":{\"active\":%s,\"current_gain\":%" PRIu32
		",\"last_robust_peak\":0,\"gain_changes\":%" PRIu32 "}"
		",\"adc_dump_cfg\":%" PRIu32 ",\"adc_dump_mode\":%" PRIu32 ","
		"\"manual_rx_gain\":{\"unit\":\"dB\",\"minimum\":%u,\"maximum\":%u,\"step\":%u}}",
		iq_usb_stream_armed() ? "true" : "false", capture_sample_rate_hz(&c), decimation,
		iq_usb_stream_owned() ? "usb" : "none", iq_usb_mounted() ? "true" : "false",
		iq_usb_frames(), iq_usb_send_errors(), iq_usb_stream_format(),
		iq_usb_stream_format(), capture_config_applying() ? "true" : "false",
		iq_usb_stream_epoch(), hardware_time_ns, dump_ring_dropped_chunks(),
		dump_ring_source_chunk_index(), (c.dc_offset.automatic != 0U) ? "true" : "false",
		(c.gain.gain_mode == GAIN_MODE_AUTO) ? "true" : "false", rf_gain_get(),
		dump_ring_agc_gain_changes(), rf_dump_cfg_read(), rf_dump_ctrl_read(),
		RX_GAIN_MIN_DB,
		GAIN_TABLE_ENTRIES - 1U, RX_GAIN_STEP_DB);

	return ((n > 0) && ((size_t)n < cap)) ? n : -1;
}

const char *iq_control_parse_config_json(char *json, size_t len, struct capture_config *config)
{
	struct cfg_doc doc;
	int ret;

	capture_get_config(config);
	doc_from_config(&doc, config);
	ret = json_obj_parse(json, len, cfg_doc_descr, ARRAY_SIZE(cfg_doc_descr), &doc);
	if (ret < 0) {
		return "malformed JSON";
	}
	config_from_doc(config, &doc);

	return capture_validate_config(config);
}

uint32_t iq_control_parse_stream_format(char *json, size_t len, uint32_t fallback)
{
	struct stream_start_doc doc = {.stream_format = fallback};
	int ret = json_obj_parse(json, len, stream_start_descr, ARRAY_SIZE(stream_start_descr),
				 &doc);

	if ((ret <= 0) || (doc.stream_format > IQ_USB_FORMAT_INT4)) {
		return fallback;
	}

	return doc.stream_format;
}
