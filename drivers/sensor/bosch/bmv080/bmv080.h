/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BOSCH_BMV080_BMV080_H_
#define ZEPHYR_DRIVERS_SENSOR_BOSCH_BMV080_BMV080_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

#include <bmv080.h>
#include <bmv080_defs.h>

/** BMV080 sensor ID string length including null terminator */
#define BMV080_SENSOR_ID_LEN 13U

/** I2C header shift per Bosch BMV080 I2C protocol */
#define BMV080_I2C_HEADER_SHIFT 1U

/** Conversion factor: micro-units per unit */
#define BMV080_MICRO_PER_UNIT 1000000

struct bmv080_config {
	struct i2c_dt_spec bus;
};

struct bmv080_data {
	bmv080_handle_t handle;
	bmv080_output_t output;
	bool data_ready;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_BOSCH_BMV080_BMV080_H_ */
