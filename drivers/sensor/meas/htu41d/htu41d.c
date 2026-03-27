/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT meas_htu41d

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include "htu41d.h"

LOG_MODULE_REGISTER(HTU41D, CONFIG_SENSOR_LOG_LEVEL);

/* Maximum conversion times indexed by OSR level (datasheet) */
static const uint8_t htu41d_conv_time[] = {
	HTU41D_CONV_TIME_OSR_0,
	HTU41D_CONV_TIME_OSR_1,
	HTU41D_CONV_TIME_OSR_2,
	HTU41D_CONV_TIME_OSR_3,
};

/**
 * Compute CRC-8 over a 2-byte data word.
 *
 * HTU41D uses polynomial x^8 + x^5 + x^4 + 1 (0x31) with zero
 * initialization.
 */
static uint8_t htu41d_compute_crc(const uint8_t *data)
{
	return crc8(data, HTU41D_CRC_DATA_LEN, HTU41D_CRC_POLY, HTU41D_CRC_INIT, false);
}

/**
 * Validate CRC for each 3-byte word in a response buffer.
 *
 * Each word is: data_msb, data_lsb, crc8. Returns true if all
 * words pass CRC verification.
 */
static bool htu41d_validate_response(const uint8_t *buf, uint8_t word_count)
{
	for (uint8_t i = 0; i < word_count; i++) {
		const uint8_t *word = &buf[i * HTU41D_WORD_SIZE];
		uint8_t crc = htu41d_compute_crc(word);

		if (crc != word[2]) {
			LOG_ERR("CRC mismatch word %u (expected 0x%02x, got 0x%02x)",
				i, crc, word[2]);
			return false;
		}
	}

	return true;
}

static int htu41d_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	const struct htu41d_config *cfg = dev->config;
	struct htu41d_data *data = dev->data;
	uint8_t conv_cmd;
	uint8_t read_cmd;
	uint8_t rx_buf[HTU41D_T_RH_BUF_LEN];
	uint8_t conv_delay;
	int ret;

	if ((chan != SENSOR_CHAN_ALL) &&
	    (chan != SENSOR_CHAN_AMBIENT_TEMP) &&
	    (chan != SENSOR_CHAN_HUMIDITY)) {
		return -ENOTSUP;
	}

	/*
	 * Build conversion command: 0x40 + (rh_osr << 3) + (t_osr << 1).
	 * Use highest OSR (3) for both temperature and humidity.
	 */
	conv_cmd = HTU41D_CMD_CONV_BASE
		   + (HTU41D_OSR_DEFAULT << HTU41D_OSR_RH_SHIFT)
		   + (HTU41D_OSR_DEFAULT << HTU41D_OSR_T_SHIFT);

	ret = i2c_write_dt(&cfg->bus, &conv_cmd, sizeof(conv_cmd));
	if (ret < 0) {
		LOG_ERR("Conversion trigger failed: %d", ret);
		return ret;
	}

	/* Wait for conversion to complete (datasheet) */
	conv_delay = htu41d_conv_time[HTU41D_OSR_DEFAULT];
	k_msleep(conv_delay);

	/* Send read command, then clock out the response */
	read_cmd = HTU41D_CMD_READ_T_RH;

	ret = i2c_burst_read_dt(&cfg->bus, read_cmd, rx_buf, sizeof(rx_buf));
	if (ret < 0) {
		LOG_ERR("I2C read failed: %d", ret);
		return ret;
	}

	if (!htu41d_validate_response(rx_buf, HTU41D_T_RH_WORDS)) {
		return -EIO;
	}

	if ((chan == SENSOR_CHAN_ALL) || (chan == SENSOR_CHAN_AMBIENT_TEMP)) {
		data->temp_sample = sys_get_be16(&rx_buf[0]);
	}

	if ((chan == SENSOR_CHAN_ALL) || (chan == SENSOR_CHAN_HUMIDITY)) {
		data->humi_sample = sys_get_be16(&rx_buf[HTU41D_WORD_SIZE]);
	}

	return 0;
}

static int htu41d_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	const struct htu41d_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP: {
		/* T[°C] = -40 + 165 * raw / 65535, in micro-°C */
		int64_t micro = HTU41D_TEMP_OFFSET_MICRO +
				(HTU41D_TEMP_SCALE_MICRO * (int64_t)data->temp_sample) /
					HTU41D_ADC_DIVISOR;

		sensor_value_from_micro(val, micro);
		LOG_DBG("temperature %u = val1:%d, val2:%d",
			data->temp_sample, val->val1, val->val2);
		break;
	}
	case SENSOR_CHAN_HUMIDITY: {
		/* RH[%] = -6 + 125 * raw / 65535, in micro-% */
		int64_t micro = HTU41D_HUMI_OFFSET_MICRO +
				(HTU41D_HUMI_SCALE_MICRO * (int64_t)data->humi_sample) /
					HTU41D_ADC_DIVISOR;

		sensor_value_from_micro(val, micro);
		LOG_DBG("humidity %u = val1:%d, val2:%d",
			data->humi_sample, val->val1, val->val2);
		break;
	}
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int htu41d_init(const struct device *dev)
{
	const struct htu41d_config *cfg = dev->config;
	uint8_t cmd = HTU41D_CMD_RESET;
	int ret;

	if (!i2c_is_ready_dt(&cfg->bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	/* Issue soft reset to bring sensor to a known state */
	ret = i2c_write_dt(&cfg->bus, &cmd, sizeof(cmd));
	if (ret < 0) {
		LOG_ERR("Soft reset failed: %d", ret);
		return ret;
	}

	/* Wait for sensor to complete reset (datasheet: 15 ms max) */
	k_msleep(HTU41D_RESET_TIME_MS);

	LOG_DBG("HTU41D initialized");

	return 0;
}

static DEVICE_API(sensor, htu41d_api) = {
	.sample_fetch = htu41d_sample_fetch,
	.channel_get = htu41d_channel_get,
};

#define HTU41D_INIT(inst)                                                      \
	static struct htu41d_data htu41d_data_##inst;                          \
	static const struct htu41d_config htu41d_config_##inst = {             \
		.bus = I2C_DT_SPEC_INST_GET(inst),                             \
	};                                                                     \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, htu41d_init, NULL,                  \
				     &htu41d_data_##inst,                      \
				     &htu41d_config_##inst, POST_KERNEL,       \
				     CONFIG_SENSOR_INIT_PRIORITY, &htu41d_api);

DT_INST_FOREACH_STATUS_OKAY(HTU41D_INIT)
