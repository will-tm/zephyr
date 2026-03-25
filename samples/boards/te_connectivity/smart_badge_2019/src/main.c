/*
 * Copyright (c) 2025 TE Connectivity
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(smart_badge, LOG_LEVEL_INF);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct device *htu21d = DEVICE_DT_GET_ANY(meas_htu21d);
static const struct device *ms5637 = DEVICE_DT_GET_ANY(meas_ms5637);
static const struct device *lsm6dsl = DEVICE_DT_GET_ANY(st_lsm6dsl);

/**
 * Try to init a deferred-init sensor. If device_init() fails (driver error),
 * the device is marked as initialized with a non-zero init_res, so subsequent
 * calls return -EALREADY. To retry, we must clear the initialized flag.
 */
static int init_sensor(const struct device *dev, const char *name)
{
	int ret;

	if (dev == NULL) {
		LOG_WRN("%s: not in devicetree", name);
		return -ENODEV;
	}

	for (int i = 0; i < 10; i++) {
		ret = device_init(dev);
		if (ret == 0) {
			LOG_INF("%s: initialized", name);
			return 0;
		}

		if (ret == -EALREADY && !device_is_ready(dev)) {
			/*
			 * Previous init attempt failed (init_res != 0) but
			 * device is marked initialized. Clear the flag so
			 * device_init() will re-run the driver init.
			 */
			dev->state->initialized = false;
			dev->state->init_res = 0;
		} else if (ret == -EALREADY) {
			/* Already initialized and ready */
			LOG_INF("%s: already initialized", name);
			return 0;
		}

		LOG_WRN("%s: init attempt %d failed (%d)", name, i + 1, ret);
		k_msleep(100);
	}

	LOG_ERR("%s: failed to initialize after 10 attempts", name);
	return -EIO;
}

static void read_htu21d(void)
{
	struct sensor_value temp, hum;

	if (sensor_sample_fetch(htu21d)) {
		LOG_WRN("HTU21D: fetch failed");
		return;
	}

	sensor_channel_get(htu21d, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	sensor_channel_get(htu21d, SENSOR_CHAN_HUMIDITY, &hum);
	LOG_INF("HTU21D: %d.%02d C, %d.%02d %%RH",
		temp.val1, temp.val2 / 10000,
		hum.val1, hum.val2 / 10000);
}

static void read_ms5637(void)
{
	struct sensor_value temp, press;

	if (sensor_sample_fetch(ms5637)) {
		LOG_WRN("MS5637: fetch failed");
		return;
	}

	sensor_channel_get(ms5637, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	sensor_channel_get(ms5637, SENSOR_CHAN_PRESS, &press);
	LOG_INF("MS5637: %d.%02d C, %d.%02d kPa",
		temp.val1, temp.val2 / 10000,
		press.val1, press.val2 / 10000);
}

static void read_lsm6dsl(void)
{
	struct sensor_value accel[3], gyro[3];
	int32_t ax, ay, az, gx, gy, gz;

	if (sensor_sample_fetch(lsm6dsl)) {
		LOG_WRN("LSM6DSL: fetch failed");
		return;
	}

	sensor_channel_get(lsm6dsl, SENSOR_CHAN_ACCEL_XYZ, accel);
	sensor_channel_get(lsm6dsl, SENSOR_CHAN_GYRO_XYZ, gyro);

	/* Convert to milli-units for clean signed formatting */
	ax = sensor_value_to_milli(&accel[0]);
	ay = sensor_value_to_milli(&accel[1]);
	az = sensor_value_to_milli(&accel[2]);
	gx = sensor_value_to_milli(&gyro[0]);
	gy = sensor_value_to_milli(&gyro[1]);
	gz = sensor_value_to_milli(&gyro[2]);

	LOG_INF("LSM6DSL: accel %d.%03d %d.%03d %d.%03d m/s2",
		ax / 1000, abs(ax) % 1000,
		ay / 1000, abs(ay) % 1000,
		az / 1000, abs(az) % 1000);
	LOG_INF("LSM6DSL: gyro %d.%03d %d.%03d %d.%03d dps",
		gx / 1000, abs(gx) % 1000,
		gy / 1000, abs(gy) % 1000,
		gz / 1000, abs(gz) % 1000);
}

int main(void)
{
	int err;
	bool htu_ok, ms_ok, lsm_ok;

	/* Start BLE advertising */
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("BLE init failed (%d)", err);
	} else {
		err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad,
				      ARRAY_SIZE(ad), NULL, 0);
		if (err) {
			LOG_ERR("BLE advertising failed (%d)", err);
		} else {
			LOG_INF("BLE advertising as \"%s\"",
				CONFIG_BT_DEVICE_NAME);
		}
	}

	/* Give sensors time to power up after board reset */
	k_msleep(200);

	/* Initialize sensors (deferred init with retries).
	 * Small delay between each to let I2C bus settle.
	 */
	lsm_ok = (init_sensor(lsm6dsl, "LSM6DSL") == 0);
	if (lsm_ok) {
		/* Set accel and gyro ODR to 12.5 Hz (low power) */
		struct sensor_value odr = { .val1 = 12, .val2 = 500000 };

		sensor_attr_set(lsm6dsl, SENSOR_CHAN_ACCEL_XYZ,
				SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
		sensor_attr_set(lsm6dsl, SENSOR_CHAN_GYRO_XYZ,
				SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	}
	htu_ok = (init_sensor(htu21d, "HTU21D") == 0);
	k_msleep(50);
	ms_ok = (init_sensor(ms5637, "MS5637") == 0);

	if (!htu_ok && !ms_ok && !lsm_ok) {
		LOG_ERR("No sensors available, not starting measurement loop");
		return 0;
	}

	/* Measurement loop */
	while (1) {
		if (htu_ok) {
			read_htu21d();
		}
		if (ms_ok) {
			read_ms5637();
		}
		if (lsm_ok) {
			read_lsm6dsl();
		}
		k_sleep(K_SECONDS(5));
	}

	return 0;
}
