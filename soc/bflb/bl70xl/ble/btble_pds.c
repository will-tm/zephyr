/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/linker/section_tags.h>

#include <bouffalolab/bl70xl/ble/btble_lib_api.h>

#include "bflb_pds.h"
#include "bl70xl_rom_api.h"

#define PDS_SLEEP_MIN_TICKS  152U
#define PDS_EARLY_WAKE_TICKS 82U
#define PDS_BIG_CONVERT_MS   7200000U
#define PDS_MS_PER_SEC       1000U

typedef int (*btble_sleep_cb_t)(void);
typedef void (*btble_wake_cb_t)(void);

/* GP-relative symbols from the BLE controller blob / rom_map.ld */
extern volatile uint8_t pds_start;
extern volatile uint32_t userApplication_before_sleep_callback_ext;
extern volatile uint32_t userApplication_after_sleep_callback;
extern volatile uint32_t btble_sleep_not_allowed_ptr;

/* Platform shim functions */
extern uint64_t bl_rtc_get_counter(void);
extern uint64_t bl_rtc_get_delta_counter(uint64_t ref);
extern uint32_t bl_rtc_ms_to_counter(uint32_t ms);

/* Blob's bt_is_ready returns int; conflicts with Zephyr's bool bt_is_ready */
extern int bt_is_ready(void);
extern int btble_controller_active_on_going(void);
extern void btble_controller_sleep_store(void);
extern void btble_controller_sleep_restore(void);
extern void btble_controller_ke_timer_reset(void);
extern void rf_init_lp(int en);

static uint8_t bleActionOngoing Z_GENERIC_SECTION(.sbss.bleActionOngoing);

void btble_pds_enable(uint8_t enable)
{
	pds_start = enable;
}

static void btble_pds_post_wake(void)
{
	if (bt_is_ready()) {
		btble_controller_sleep_restore();
	}

	if (!bleActionOngoing) {
		if (bt_is_ready()) {
			btble_controller_ke_timer_reset();
		}
		rf_init_lp(0);
	}
}

static void btble_pds_check_after_cb(void)
{
	btble_wake_cb_t after_cb;

	after_cb = (btble_wake_cb_t)(uintptr_t)userApplication_after_sleep_callback;
	if (after_cb != NULL) {
		after_cb();
	}
}

void btble_pds_sleep_cycle(uint32_t expected_idle_ms)
{
	uint64_t rtc_start;
	uint64_t sleep_ticks;
	uint64_t elapsed;
	uint64_t remaining;
	int32_t controller_max;
	unsigned int key;
	btble_sleep_cb_t before_cb;

	rtc_start = bl_rtc_get_counter();

	if (!pds_start) {
		return;
	}

	if (expected_idle_ms < PDS_BIG_CONVERT_MS) {
		sleep_ticks = bl_rtc_ms_to_counter(expected_idle_ms);
	} else {
		sleep_ticks = (uint64_t)bl_rtc_frequency * expected_idle_ms / PDS_MS_PER_SEC;
	}

	if (sleep_ticks <= PDS_SLEEP_MIN_TICKS) {
		return;
	}

	before_cb = (btble_sleep_cb_t)(uintptr_t)userApplication_before_sleep_callback_ext;
	if (before_cb != NULL) {
		if (before_cb() < 0) {
			return;
		}
	}

	key = irq_lock();

	elapsed = bl_rtc_get_delta_counter(rtc_start);
	if (elapsed >= sleep_ticks || sleep_ticks - elapsed <= PDS_SLEEP_MIN_TICKS + 1U) {
		irq_unlock(key);
		return;
	}

	remaining = sleep_ticks - elapsed;

	bleActionOngoing = (uint8_t)btble_controller_active_on_going();

	if (bleActionOngoing) {
		controller_max = btble_controller_sleep(0);

		if (controller_max <= (int32_t)PDS_SLEEP_MIN_TICKS) {
			irq_unlock(key);
			if (btble_sleep_not_allowed_ptr != 0U) {
				((btble_wake_cb_t)(uintptr_t)btble_sleep_not_allowed_ptr)();
			}
			return;
		}

		if ((uint64_t)controller_max < remaining) {
			remaining = (uint64_t)controller_max;
		}
	} else {
		if (bt_is_ready()) {
			btble_controller_sleep_store();
		}
	}

	if (remaining <= PDS_EARLY_WAKE_TICKS) {
		remaining = 0U;
	} else {
		remaining -= PDS_EARLY_WAKE_TICKS;
	}

	bflb_pds_enter_s2ram((uint32_t)remaining);

	irq_unlock(key);

	btble_pds_post_wake();
	btble_pds_check_after_cb();
}
