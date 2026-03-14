/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * USB remote wakeup test application.
 *
 * Registers a CDC ACM device with remote wakeup capability.
 * When the host suspends the USB bus, the device waits 3 seconds
 * and then initiates a remote wakeup request.
 *
 * Test procedure (Linux host):
 *   1. Flash and boot the device
 *   2. Find device path in /sys/bus/usb/devices/
 *   3. Enable remote wakeup and trigger suspend:
 *        echo enabled > DEVPATH/power/wakeup
 *        echo 1 > DEVPATH/power/autosuspend_delay_ms
 *        echo auto > DEVPATH/power/control
 *   4. Watch UART console for: Suspended, Wakeup sent, Resumed
 *   5. Check dmesg for USB resume events
 */

#include <sample_usbd.h>

#include <zephyr/kernel.h>
#include <zephyr/usb/usbd.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rwup_test, LOG_LEVEL_INF);

#define WAKEUP_DELAY_MS 3000

static struct usbd_context *sample_usbd;
static struct k_work_delayable wakeup_work;

static void wakeup_work_handler(struct k_work *work)
{
	int ret;

	ARG_UNUSED(work);

	if (!usbd_is_suspended(sample_usbd)) {
		LOG_INF("No longer suspended, skipping wakeup");
		return;
	}

	ret = usbd_wakeup_request(sample_usbd);
	if (ret) {
		LOG_ERR("Wakeup request failed: %d", ret);
	} else {
		LOG_INF("Wakeup sent");
	}
}

static void msg_cb(struct usbd_context *const ctx,
		   const struct usbd_msg *const msg)
{
	LOG_INF("USBD: %s", usbd_msg_type_string(msg->type));

	if (usbd_can_detect_vbus(ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(ctx)) {
				LOG_ERR("Failed to enable device support");
			}
		}

		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(ctx)) {
				LOG_ERR("Failed to disable device support");
			}
		}
	}

	if (msg->type == USBD_MSG_SUSPEND) {
		LOG_INF("Suspended, will send wakeup in %d ms", WAKEUP_DELAY_MS);
		k_work_schedule(&wakeup_work, K_MSEC(WAKEUP_DELAY_MS));
	}

	if (msg->type == USBD_MSG_RESUME) {
		k_work_cancel_delayable(&wakeup_work);
		LOG_INF("Resumed");
	}
}

int main(void)
{
	int ret;

	k_work_init_delayable(&wakeup_work, wakeup_work_handler);

	sample_usbd = sample_usbd_init_device(msg_cb);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return 0;
	}

	if (!usbd_can_detect_vbus(sample_usbd)) {
		ret = usbd_enable(sample_usbd);
		if (ret) {
			LOG_ERR("Failed to enable device support");
			return 0;
		}
	}

	LOG_INF("USB remote wakeup test ready");
	LOG_INF("Suspend the USB device from the host to trigger wakeup");

	return 0;
}
