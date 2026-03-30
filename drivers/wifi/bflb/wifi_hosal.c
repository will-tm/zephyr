/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * WiFi HOSAL stubs for BL808 — provides symbols required by libwifi.a
 */

#include <stdint.h>

static int temperature = 25;

int wifi_hosal_rf_turn_on(void *arg)
{
	return 0;
}

int wifi_hosal_rf_turn_off(void *arg)
{
	return 0;
}

int wifi_hosal_pm_init(void)
{
	return 0;
}

int wifi_hosal_pm_event_register(int event, uint32_t code, uint32_t cap_bit, uint16_t priority,
				 void *ops, void *arg, int enable)
{
	return 0;
}

int wifi_hosal_pm_deinit(void)
{
	return 0;
}

int wifi_hosal_pm_state_run(void)
{
	return 0;
}

int wifi_hosal_pm_capacity_set(int level)
{
	return 0;
}

int wifi_hosal_pm_post_event(int event, uint32_t code, uint32_t *retval)
{
	return 0;
}

int wifi_hosal_pm_event_switch(int event, uint32_t code, int enable)
{
	return 0;
}

int hal_get_temperature(void)
{
	return temperature;
}

void hal_set_temperature(int temp)
{
	temperature = temp;
}
