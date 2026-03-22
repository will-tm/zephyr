/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief MAC154/Zigbee peripheral clock and reset for BL61X.
 *
 * Enables the MAC154 peripheral clock gate via GLB_CGEN_CFG2.
 */

#include <stdint.h>
#include <zephyr/sys/util.h>

#include <bflb_mac154.h>
#include <bflb_soc.h>
#include <glb_reg.h>

void bflb_mac154_clock_init(void)
{
	uint32_t tmp;

	/* Enable MAC154 clock gate (GLB_CGEN_CFG2, BL_AHB_SLAVE2_M154 = bit 9) */
	tmp = sys_read32(GLB_BASE + GLB_CGEN_CFG2_OFFSET);
	tmp |= (1U << BL_AHB_SLAVE2_M154);
	sys_write32(tmp, GLB_BASE + GLB_CGEN_CFG2_OFFSET);
}
