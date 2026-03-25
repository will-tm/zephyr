/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_HTU21D_HTU21D_H_
#define ZEPHYR_DRIVERS_SENSOR_HTU21D_HTU21D_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/* I2C commands (HTU21D datasheet Table 3) */
#define HTU21D_CMD_TEMP_HOLD   0xE3U
#define HTU21D_CMD_HUMI_HOLD   0xE5U
#define HTU21D_CMD_TEMP_NOHOLD 0xF3U
#define HTU21D_CMD_HUMI_NOHOLD 0xF5U
#define HTU21D_CMD_WRITE_USER  0xE6U
#define HTU21D_CMD_READ_USER   0xE7U
#define HTU21D_CMD_SOFT_RESET  0xFEU

/* CRC-8 parameters: polynomial x^8 + x^5 + x^4 + 1 (datasheet Section 6) */
#define HTU21D_CRC_POLY 0x31U
#define HTU21D_CRC_INIT 0x00U

/* Each measurement response is 2 data bytes + 1 CRC byte */
#define HTU21D_RESPONSE_LEN 3U

/* Soft reset recovery time per datasheet: 15 ms max */
#define HTU21D_RESET_TIME_MS 15U

/* Status bits in raw measurement word (bits 1:0) */
#define HTU21D_STATUS_BITS_MASK 0x0003U

/*
 * Temperature: T[C] = -46.85 + 175.72 * raw / 65536
 *
 * Scaled by 2^23 for integer math (from Si7006 driver, same formula):
 *   temp_23 = raw * 22492 - 393001039
 *
 * The scale factor 22492 = round(175.72 * 128), and the offset
 * 393001039 = round(46.85 * 128 * 65536) - 5246.
 * The constant 5246 centers rounding error about zero.
 */
#define HTU21D_TEMP_SCALE  22492
#define HTU21D_TEMP_OFFSET (393006285 - 5246)

/*
 * Humidity: RH[%] = -6 + 125 * raw / 65536
 */
#define HTU21D_HUMI_SCALE  125U
#define HTU21D_HUMI_OFFSET 6

struct htu21d_config {
	struct i2c_dt_spec bus;
};

struct htu21d_data {
	uint16_t temp_sample;
	uint16_t humi_sample;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_HTU21D_HTU21D_H_ */
