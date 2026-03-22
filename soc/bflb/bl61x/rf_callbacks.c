/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * RF library callbacks required by libbl616_phyrf.a.
 * These are called by the RF driver during calibration and reset events.
 */

#include <zephyr/kernel.h>
#include <stdint.h>

/*
 * rf_reset_done_callback — Called after RF reset completes.
 */
void rf_reset_done_callback(void)
{
}

/*
 * rf_full_cal_start_callback — Called before RF full calibration.
 * No DMA conflict checking needed during early init.
 */
void rf_full_cal_start_callback(uint32_t addr, uint32_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);
}
