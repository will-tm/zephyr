/*
 * Copyright (c) 2025 TE Connectivity
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

#if CONFIG_FAT_FILESYSTEM_ELM
#include <ff.h>
#endif

LOG_MODULE_REGISTER(smart_badge, LOG_LEVEL_INF);

/* LEDs */
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

/* USB composite device: CDC ACM (shell) + MSC (flash disk) */
USBD_DEVICE_DEFINE(badge_usbd,
		   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   0x2fe3, 0x0100);

USBD_DESC_LANG_DEFINE(badge_lang);
USBD_DESC_MANUFACTURER_DEFINE(badge_mfr, "TE Connectivity");
USBD_DESC_PRODUCT_DEFINE(badge_product, "Smart Badge 2019");

USBD_DESC_CONFIG_DEFINE(badge_fs_cfg, "FS Configuration");

USBD_CONFIGURATION_DEFINE(badge_fs_config, 0, 250, &badge_fs_cfg);

USBD_DEFINE_MSC_LUN(nand, "NAND", "TE", "SmartBadge", "1.00");

static int usb_composite_init(void)
{
	int err;

	err = usbd_add_descriptor(&badge_usbd, &badge_lang);
	if (err) {
		return err;
	}

	err = usbd_add_descriptor(&badge_usbd, &badge_mfr);
	if (err) {
		return err;
	}

	err = usbd_add_descriptor(&badge_usbd, &badge_product);
	if (err) {
		return err;
	}

	err = usbd_add_configuration(&badge_usbd, USBD_SPEED_FS,
				     &badge_fs_config);
	if (err) {
		LOG_ERR("USB add config failed: %d", err);
		return err;
	}

	err = usbd_register_all_classes(&badge_usbd, USBD_SPEED_FS, 1, NULL);
	if (err) {
		LOG_ERR("USB register classes failed: %d", err);
		return err;
	}

	usbd_device_set_code_triple(&badge_usbd, USBD_SPEED_FS,
				    USB_BCC_MISCELLANEOUS, 0x02, 0x01);

	err = usbd_init(&badge_usbd);
	if (err) {
		LOG_ERR("USB init failed: %d", err);
		return err;
	}

	err = usbd_enable(&badge_usbd);
	if (err) {
		LOG_ERR("USB enable failed: %d", err);
	}
	return err;
}

SYS_INIT(usb_composite_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* BLE */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static bool ble_connected;
static bool error_occurred;

/* Sensors */
static const struct device *htu21d = DEVICE_DT_GET_ANY(meas_htu21d);
static const struct device *ms5637 = DEVICE_DT_GET_ANY(meas_ms5637);
static const struct device *lsm6dsl = DEVICE_DT_GET_ANY(st_lsm6dsl);

/* Sensor data cache for display */
static struct {
	int32_t temp_milli;      /* HTU21D temperature in milli-C */
	int32_t hum_milli;       /* HTU21D humidity in milli-%RH */
	int32_t press_milli;     /* MS5637 pressure in milli-kPa */
	int32_t accel_mg[3];     /* LSM6DSL accel in milli-g */
} sensor_data;

/* LED control */
static void led_init(void)
{
	gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_INACTIVE);
}

static void led_set_error(void)
{
	error_occurred = true;
	gpio_pin_set_dt(&led_red, 1);
	gpio_pin_set_dt(&led_green, 0);
	gpio_pin_set_dt(&led_blue, 0);
}

static void led_blink_blue(void)
{
	if (error_occurred) {
		return;
	}
	gpio_pin_set_dt(&led_blue, 1);
	k_msleep(50);
	gpio_pin_set_dt(&led_blue, 0);
}

static void led_set_connected(void)
{
	if (error_occurred) {
		return;
	}
	gpio_pin_set_dt(&led_green, 1);
	gpio_pin_set_dt(&led_blue, 0);
}

static void led_set_advertising(void)
{
	if (error_occurred) {
		return;
	}
	gpio_pin_set_dt(&led_green, 0);
}

/* BLE callbacks */
static void bt_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("BLE connection failed (%u)", err);
		return;
	}
	LOG_INF("BLE connected");
	ble_connected = true;
	led_set_connected();
}

static void bt_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("BLE disconnected (reason %u)", reason);
	ble_connected = false;
	led_set_advertising();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = bt_connected,
	.disconnected = bt_disconnected,
};

/* Sensor init with retry */
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
			dev->state->initialized = false;
			dev->state->init_res = 0;
		} else if (ret == -EALREADY) {
			LOG_INF("%s: already initialized", name);
			return 0;
		}

		LOG_WRN("%s: init attempt %d failed (%d)", name, i + 1, ret);
		k_msleep(100);
	}

	LOG_ERR("%s: failed to initialize after 10 attempts", name);
	return -EIO;
}

/* Sensor reading */
static bool read_sensors(bool htu_ok, bool ms_ok, bool lsm_ok)
{
	bool any_ok = false;

	if (htu_ok) {
		struct sensor_value temp, hum;

		if (sensor_sample_fetch(htu21d) == 0) {
			sensor_channel_get(htu21d, SENSOR_CHAN_AMBIENT_TEMP, &temp);
			sensor_channel_get(htu21d, SENSOR_CHAN_HUMIDITY, &hum);
			sensor_data.temp_milli = sensor_value_to_milli(&temp);
			sensor_data.hum_milli = sensor_value_to_milli(&hum);
			LOG_INF("HTU21D: %d.%02d C, %d.%02d %%RH",
				temp.val1, temp.val2 / 10000,
				hum.val1, hum.val2 / 10000);
			any_ok = true;
		}
	}

	if (ms_ok) {
		struct sensor_value temp, press;

		if (sensor_sample_fetch(ms5637) == 0) {
			sensor_channel_get(ms5637, SENSOR_CHAN_AMBIENT_TEMP, &temp);
			sensor_channel_get(ms5637, SENSOR_CHAN_PRESS, &press);
			sensor_data.press_milli = sensor_value_to_milli(&press);
			LOG_INF("MS5637: %d.%02d C, %d.%02d kPa",
				temp.val1, temp.val2 / 10000,
				press.val1, press.val2 / 10000);
			any_ok = true;
		}
	}

	if (lsm_ok) {
		struct sensor_value accel[3];

		if (sensor_sample_fetch(lsm6dsl) == 0) {
			sensor_channel_get(lsm6dsl, SENSOR_CHAN_ACCEL_XYZ, accel);
			sensor_data.accel_mg[0] = sensor_value_to_milli(&accel[0]);
			sensor_data.accel_mg[1] = sensor_value_to_milli(&accel[1]);
			sensor_data.accel_mg[2] = sensor_value_to_milli(&accel[2]);
			any_ok = true;
		}
	}

	return any_ok;
}

/* E-ink display update */
static void epd_update(const struct device *display)
{
	if (!display || !device_is_ready(display)) {
		return;
	}

	struct display_capabilities caps;
	struct display_buffer_descriptor desc;
	/* 1-bit monochrome: 250 pixels wide = 32 bytes per row */
	static uint8_t buf[32 * 12]; /* 12 rows per text line */
	int y = 0;

	display_get_capabilities(display, &caps);

	/* Blank the display */
	desc.buf_size = sizeof(buf);
	desc.width = caps.x_resolution;
	desc.pitch = caps.x_resolution;

	/* Write sensor text lines using simple pixel-clear approach:
	 * For a real product, use LVGL or a font renderer.
	 * Here we just blank and let the display show the data via LOG.
	 */
	memset(buf, 0xFF, sizeof(buf)); /* white */
	desc.height = 12;

	/* Write each text line as a white band — the real display content
	 * would come from a proper graphics library. For now, just trigger
	 * a refresh so the display is alive.
	 */
	for (int row = 0; row < 10 && y < (int)caps.y_resolution; row++) {
		display_write(display, 0, y, &desc, buf);
		y += 12;
	}

	display_blanking_off(display);

	LOG_INF("E-ink display refreshed (sensor data on serial)");
}

int main(void)
{
	int err;
	bool htu_ok, ms_ok, lsm_ok;
	const struct device *display = DEVICE_DT_GET_ANY(solomon_ssd1680);

	/* Initialize LEDs */
	led_init();

	/* Start BLE (USB composite init handled by SYS_INIT) */
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("BLE init failed (%d)", err);
		led_set_error();
	} else {
		err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad,
				      ARRAY_SIZE(ad), NULL, 0);
		if (err) {
			LOG_ERR("BLE advertising failed (%d)", err);
			led_set_error();
		} else {
			LOG_INF("BLE advertising as \"%s\"",
				CONFIG_BT_DEVICE_NAME);
			led_blink_blue();
		}
	}

	/* Give sensors time to power up */
	k_msleep(200);

	/* Initialize sensors */
	lsm_ok = (init_sensor(lsm6dsl, "LSM6DSL") == 0);
	if (lsm_ok) {
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
		LOG_ERR("No sensors available");
		led_set_error();
	}

	/* Measurement loop */
	int cycle = 0;

	while (1) {
		read_sensors(htu_ok, ms_ok, lsm_ok);

		/* Blink blue LED while advertising (not connected, no error) */
		if (!ble_connected && !error_occurred) {
			led_blink_blue();
		}

		/* Refresh e-ink every 6th cycle (30s) to avoid wear */
		if (display && device_is_ready(display) && (cycle % 6) == 0) {
			epd_update(display);
		}

		cycle++;
		k_sleep(K_SECONDS(5));
	}

	return 0;
}
