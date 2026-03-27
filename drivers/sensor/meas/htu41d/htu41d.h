/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_HTU41D_HTU41D_H_
#define ZEPHYR_DRIVERS_SENSOR_HTU41D_HTU41D_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/* I2C commands (HTU41D datasheet) */
#define HTU41D_CMD_RESET       0x1EU
#define HTU41D_CMD_HEATER_ON   0x04U
#define HTU41D_CMD_HEATER_OFF  0x02U
#define HTU41D_CMD_READ_T_RH   0x00U
#define HTU41D_CMD_READ_RH     0x10U
#define HTU41D_CMD_READ_DIAG   0x08U
#define HTU41D_CMD_READ_SN     0x0AU

/*
 * Conversion command byte format (datasheet):
 *   bit 7: 0, bit 6: 1, bit 5: 0
 *   bits 4:3 = humidity OSR (0-3)
 *   bits 2:1 = temperature OSR (0-3)
 *   bit 0: 0
 *
 * Base command is 0x40, with OSR encoded as:
 *   cmd = 0x40 + (rh_osr << 3) + (t_osr << 1)
 */
#define HTU41D_CMD_CONV_BASE   0x40U

/* CRC-8 parameters: polynomial x^8 + x^5 + x^4 + 1 */
#define HTU41D_CRC_POLY        0x31U
#define HTU41D_CRC_INIT        0x00U
#define HTU41D_CRC_DATA_LEN    2U

/*
 * Read T+RH response: temp_msb, temp_lsb, temp_crc,
 *                     humi_msb, humi_lsb, humi_crc
 */
#define HTU41D_WORD_SIZE       3U
#define HTU41D_T_RH_WORDS      2U
#define HTU41D_T_RH_BUF_LEN   (HTU41D_T_RH_WORDS * HTU41D_WORD_SIZE)

/* Reset recovery time per datasheet: 15 ms max */
#define HTU41D_RESET_TIME_MS   15U

/*
 * Maximum conversion times per OSR level in ms (datasheet).
 * The total time is max(T_time, RH_time) since both run in parallel.
 * Values are rounded up from datasheet maximums.
 */
#define HTU41D_CONV_TIME_OSR_0  2U   /* max(1.57, 1.11) */
#define HTU41D_CONV_TIME_OSR_1  4U   /* max(3.06, 2.14) */
#define HTU41D_CONV_TIME_OSR_2  7U   /* max(6.03, 4.21) */
#define HTU41D_CONV_TIME_OSR_3  12U  /* max(11.98, 8.34) */

/* OSR bit field positions in conversion command byte */
#define HTU41D_OSR_T_SHIFT     1U
#define HTU41D_OSR_RH_SHIFT    3U

/* Default oversampling: OSR 3 (highest resolution) */
#define HTU41D_OSR_DEFAULT     3U

/*
 * Datasheet conversion coefficients:
 *
 * Temperature: T[°C] = -40 + 165 * raw / 65535
 * Humidity:    RH[%] = -6 + 125 * raw / 65535
 */
#define HTU41D_TEMP_OFFSET     -40.0
#define HTU41D_TEMP_SCALE      165.0
#define HTU41D_HUMI_OFFSET     -6.0
#define HTU41D_HUMI_SCALE      125.0
#define HTU41D_ADC_DIVISOR     65535

/* Micro-unit scaled versions for use with sensor_value_from_micro() */
#define HTU41D_MICRO_PER_UNIT    1000000LL
#define HTU41D_TEMP_OFFSET_MICRO ((int64_t)(HTU41D_TEMP_OFFSET * HTU41D_MICRO_PER_UNIT))
#define HTU41D_TEMP_SCALE_MICRO  ((int64_t)(HTU41D_TEMP_SCALE * HTU41D_MICRO_PER_UNIT))
#define HTU41D_HUMI_OFFSET_MICRO ((int64_t)(HTU41D_HUMI_OFFSET * HTU41D_MICRO_PER_UNIT))
#define HTU41D_HUMI_SCALE_MICRO  ((int64_t)(HTU41D_HUMI_SCALE * HTU41D_MICRO_PER_UNIT))

struct htu41d_config {
	struct i2c_dt_spec bus;
};

struct htu41d_data {
	uint16_t temp_sample;
	uint16_t humi_sample;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_HTU41D_HTU41D_H_ */
