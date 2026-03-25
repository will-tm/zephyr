/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bmv080

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "bmv080.h"

LOG_MODULE_REGISTER(bmv080, CONFIG_SENSOR_LOG_LEVEL);

/*
 * Bosch SDK I2C callbacks
 *
 * The BMV080 SDK uses 16-bit register addresses and 16-bit word payloads.
 * For I2C, the header (register address) must be shifted left by 1.
 * All data is transferred MSB-first (big-endian on the wire).
 */

static int8_t bmv080_i2c_read_cb(bmv080_sercom_handle_t sercom_handle,
				 uint16_t header, uint16_t *payload,
				 uint16_t payload_length)
{
	const struct device *dev = (const struct device *)sercom_handle;
	const struct bmv080_config *cfg = dev->config;
	uint16_t reg = header << BMV080_I2C_HEADER_SHIFT;
	uint8_t reg_buf[2];
	int ret;

	sys_put_be16(reg, reg_buf);

	ret = i2c_write_read_dt(&cfg->bus, reg_buf, sizeof(reg_buf),
				payload, payload_length * sizeof(uint16_t));
	if (ret < 0) {
		return -1;
	}

	/* Convert payload from big-endian wire format to host order */
	for (uint16_t i = 0; i < payload_length; i++) {
		payload[i] = sys_be16_to_cpu(payload[i]);
	}

	return 0;
}

static int8_t bmv080_i2c_write_cb(bmv080_sercom_handle_t sercom_handle,
				  uint16_t header, const uint16_t *payload,
				  uint16_t payload_length)
{
	const struct device *dev = (const struct device *)sercom_handle;
	const struct bmv080_config *cfg = dev->config;
	uint16_t reg = header << BMV080_I2C_HEADER_SHIFT;
	size_t msg_len = sizeof(uint16_t) + (payload_length * sizeof(uint16_t));
	uint8_t buf[msg_len];
	int ret;

	/* Register address in big-endian */
	sys_put_be16(reg, buf);

	/* Payload words in big-endian */
	for (uint16_t i = 0; i < payload_length; i++) {
		sys_put_be16(payload[i], &buf[sizeof(uint16_t) + (i * sizeof(uint16_t))]);
	}

	ret = i2c_write_dt(&cfg->bus, buf, msg_len);
	if (ret < 0) {
		return -1;
	}

	return 0;
}

static int8_t bmv080_delay_cb(uint32_t duration_ms)
{
	k_msleep(duration_ms);
	return 0;
}

static void bmv080_data_ready_cb(bmv080_output_t output, void *param)
{
	struct bmv080_data *data = param;

	data->output = output;
	data->data_ready = true;
}

static void bmv080_float_to_sensor_value(float val, struct sensor_value *out)
{
	int32_t integer = (int32_t)val;
	float frac = val - (float)integer;
	int32_t micro = (int32_t)(frac * BMV080_MICRO_PER_UNIT);

	/* Normalize sign: both parts must have the same sign */
	if (integer < 0 && micro > 0) {
		integer += 1;
		micro -= BMV080_MICRO_PER_UNIT;
	} else if (integer > 0 && micro < 0) {
		integer -= 1;
		micro += BMV080_MICRO_PER_UNIT;
	}

	out->val1 = integer;
	out->val2 = micro;
}

static int bmv080_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct bmv080_data *data = dev->data;
	bmv080_status_code_t status;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_PM_1_0 &&
	    chan != SENSOR_CHAN_PM_2_5 && chan != SENSOR_CHAN_PM_10) {
		return -ENOTSUP;
	}

	data->data_ready = false;

	status = bmv080_serve_interrupt(data->handle, bmv080_data_ready_cb,
					data);
	if (status != E_BMV080_OK) {
		LOG_ERR("serve_interrupt failed: %d", status);
		return -EIO;
	}

	return 0;
}

static int bmv080_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bmv080_data *data = dev->data;

	if (!data->data_ready) {
		return -ENODATA;
	}

	switch (chan) {
	case SENSOR_CHAN_PM_1_0:
		bmv080_float_to_sensor_value(
			data->output.pm1_mass_concentration, val);
		break;
	case SENSOR_CHAN_PM_2_5:
		bmv080_float_to_sensor_value(
			data->output.pm2_5_mass_concentration, val);
		break;
	case SENSOR_CHAN_PM_10:
		bmv080_float_to_sensor_value(
			data->output.pm10_mass_concentration, val);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static DEVICE_API(sensor, bmv080_driver_api) = {
	.sample_fetch = bmv080_sample_fetch,
	.channel_get = bmv080_channel_get,
};

static int bmv080_init(const struct device *dev)
{
	const struct bmv080_config *cfg = dev->config;
	struct bmv080_data *data = dev->data;
	bmv080_status_code_t status;
	char sensor_id[BMV080_SENSOR_ID_LEN];
	uint16_t major, minor, patch;
	char git_hash[12];
	int32_t commits_ahead;

	if (!i2c_is_ready_dt(&cfg->bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	status = bmv080_get_driver_version(&major, &minor, &patch,
					   git_hash, &commits_ahead);
	if (status != E_BMV080_OK) {
		LOG_ERR("get_driver_version failed: %d", status);
		return -EIO;
	}
	LOG_INF("BMV080 SDK v%u.%u.%u", major, minor, patch);

	status = bmv080_open(&data->handle,
			     (bmv080_sercom_handle_t)dev,
			     bmv080_i2c_read_cb,
			     bmv080_i2c_write_cb,
			     bmv080_delay_cb);
	if (status != E_BMV080_OK) {
		LOG_ERR("bmv080_open failed: %d", status);
		return -EIO;
	}

	status = bmv080_reset(data->handle);
	if (status != E_BMV080_OK) {
		LOG_ERR("bmv080_reset failed: %d", status);
		return -EIO;
	}

	status = bmv080_get_sensor_id(data->handle, sensor_id);
	if (status != E_BMV080_OK) {
		LOG_ERR("get_sensor_id failed: %d", status);
		return -EIO;
	}
	LOG_INF("Sensor ID: %s", sensor_id);

	status = bmv080_start_continuous_measurement(data->handle);
	if (status != E_BMV080_OK) {
		LOG_ERR("start_continuous_measurement failed: %d", status);
		return -EIO;
	}

	return 0;
}

#define BMV080_DEFINE(inst)							\
	static struct bmv080_data bmv080_data_##inst;				\
										\
	static const struct bmv080_config bmv080_config_##inst = {		\
		.bus = I2C_DT_SPEC_INST_GET(inst),				\
	};									\
										\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, bmv080_init, NULL,			\
				     &bmv080_data_##inst,			\
				     &bmv080_config_##inst,			\
				     POST_KERNEL,				\
				     CONFIG_SENSOR_INIT_PRIORITY,		\
				     &bmv080_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BMV080_DEFINE)
