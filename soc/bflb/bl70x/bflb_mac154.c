/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief MAC154/Zigbee peripheral clock and reset for BL70X.
 *
 * BL70X does not have these functions in ROM, so we implement them
 * directly via GLB register writes.
 */

#include <stdint.h>
#include <zephyr/sys/util.h>

#include <bflb_mac154.h>
#include <bflb_soc.h>
#include <glb_reg.h>

void bflb_mac154_clock_init(void)
{
	uint32_t tmp;

	/* Enable MAC154/Zigbee module clock (GLB_CLK_CFG1 bit 25) */
	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG1_OFFSET);
	tmp |= (1U << GLB_M154_ZBEN_POS);
	sys_write32(tmp, GLB_BASE + GLB_CLK_CFG1_OFFSET);

	/* Enable MAC154 clock gate (GLB_CGEN_CFG2 bit 4) */
	tmp = sys_read32(GLB_BASE + GLB_CGEN_CFG2_OFFSET);
	tmp |= (1U << GLB_CGEN_S3_POS);
	sys_write32(tmp, GLB_BASE + GLB_CGEN_CFG2_OFFSET);
}
