/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT meas_htu21d

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#include "htu21d.h"

LOG_MODULE_REGISTER(HTU21D, CONFIG_SENSOR_LOG_LEVEL);

/**
 * Compute CRC-8 over a 2-byte measurement value.
 *
 * HTU21D uses polynomial x^8 + x^5 + x^4 + 1 with zero initialization
 * (datasheet Section 6).
 */
static uint8_t htu21d_compute_crc(const uint8_t *data)
{
	return crc8(data, 2, HTU21D_CRC_POLY, HTU21D_CRC_INIT, false);
}

/**
 * Read a single measurement using hold-master mode.
 *
 * The sensor holds SCL low after the command byte until the conversion
 * completes, then releases it so the master can clock out 3 bytes:
 * data MSB, data LSB, and CRC-8 checksum.
 */
static int htu21d_read_measurement(const struct device *dev, uint8_t cmd,
				   uint16_t *raw)
{
	const struct htu21d_config *cfg = dev->config;
	uint8_t rx_buf[HTU21D_RESPONSE_LEN];
	uint8_t crc;
	int ret;

	ret = i2c_burst_read_dt(&cfg->bus, cmd, rx_buf, sizeof(rx_buf));
	if (ret < 0) {
		LOG_ERR("I2C read failed: %d", ret);
		return ret;
	}

	crc = htu21d_compute_crc(rx_buf);
	if (crc != rx_buf[2]) {
		LOG_ERR("CRC mismatch (expected 0x%02x, got 0x%02x)",
			crc, rx_buf[2]);
		return -EIO;
	}

	/* Mask off status bits 1:0 per datasheet */
	*raw = sys_get_be16(rx_buf) & ~HTU21D_STATUS_BITS_MASK;

	return 0;
}

static int htu21d_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct htu21d_data *data = dev->data;
	int ret;

	if ((chan != SENSOR_CHAN_ALL) &&
	    (chan != SENSOR_CHAN_AMBIENT_TEMP) &&
	    (chan != SENSOR_CHAN_HUMIDITY)) {
		return -ENOTSUP;
	}

	if ((chan == SENSOR_CHAN_ALL) || (chan == SENSOR_CHAN_HUMIDITY)) {
		ret = htu21d_read_measurement(dev, HTU21D_CMD_HUMI_HOLD,
					      &data->humi_sample);
		if (ret < 0) {
			return ret;
		}
	}

	if ((chan == SENSOR_CHAN_ALL) || (chan == SENSOR_CHAN_AMBIENT_TEMP)) {
		ret = htu21d_read_measurement(dev, HTU21D_CMD_TEMP_HOLD,
					      &data->temp_sample);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int htu21d_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	const struct htu21d_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP: {
		/*
		 * T[C] = -46.85 + 175.72 * raw / 65536
		 * Integer math scaled by 2^23 (see htu21d.h for derivation).
		 */
		int32_t temp_23 = (int32_t)data->temp_sample * HTU21D_TEMP_SCALE
				  - (int32_t)HTU21D_TEMP_OFFSET;
		int32_t temp_int = temp_23 >> 23;
		int32_t temp_frac = temp_23 & BIT_MASK(23);

		if (temp_23 < 0) {
			temp_int += 1;
			temp_frac -= BIT(23);
		}

		val->val1 = temp_int;
		/* (temp_frac * 1000000) >> 23 = (temp_frac * 15625) >> 17 */
		val->val2 = (int32_t)((temp_frac * 15625LL) >> 17);

		LOG_DBG("temperature %u = val1:%d, val2:%d",
			data->temp_sample, val->val1, val->val2);
		break;
	}
	case SENSOR_CHAN_HUMIDITY: {
		/*
		 * RH[%] = -6 + 125 * raw / 65536
		 */
		uint32_t rh_16 = (uint32_t)data->humi_sample * HTU21D_HUMI_SCALE;
		int16_t rh_int = (int16_t)(rh_16 >> 16);
		uint16_t rh_frac = (uint16_t)(rh_16 & BIT_MASK(16));

		val->val1 = rh_int - HTU21D_HUMI_OFFSET;
		/* (rh_frac * 1000000) >> 16 = (rh_frac * 15625) >> 10 */
		val->val2 = (int32_t)((rh_frac * 15625U) >> 10);

		if (val->val1 < 0) {
			val->val1 += 1;
			val->val2 -= 1000000;
		}

		LOG_DBG("humidity %u = val1:%d, val2:%d",
			data->humi_sample, val->val1, val->val2);
		break;
	}
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int htu21d_init(const struct device *dev)
{
	const struct htu21d_config *cfg = dev->config;
	uint8_t cmd = HTU21D_CMD_SOFT_RESET;
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

	/* Wait for sensor to complete reset per datasheet (15 ms max) */
	k_msleep(HTU21D_RESET_TIME_MS);

	LOG_DBG("HTU21D initialized");

	return 0;
}

static DEVICE_API(sensor, htu21d_api) = {
	.sample_fetch = htu21d_sample_fetch,
	.channel_get = htu21d_channel_get,
};

#define HTU21D_INIT(inst)                                                      \
	static struct htu21d_data htu21d_data_##inst;                          \
	static const struct htu21d_config htu21d_config_##inst = {             \
		.bus = I2C_DT_SPEC_INST_GET(inst),                             \
	};                                                                     \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, htu21d_init, NULL,                  \
				     &htu21d_data_##inst,                      \
				     &htu21d_config_##inst, POST_KERNEL,       \
				     CONFIG_SENSOR_INIT_PRIORITY, &htu21d_api);

DT_INST_FOREACH_STATUS_OKAY(HTU21D_INIT)
