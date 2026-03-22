/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief MAC154/Zigbee peripheral clock and reset for BL70XL.
 *
 * Wraps ROM-provided GLB functions so the ieee802154 driver does not
 * need to reference ROM symbols directly.
 */

#include <stdint.h>

#include <bflb_mac154.h>

/* ROM functions (symbols provided by romdriver.ld) */
extern void GLB_Set_MAC154_ZIGBEE_CLK(uint8_t enable);
extern void GLB_MAC154_ZIGBEE_Reset(void);

void bflb_mac154_clock_init(void)
{
	GLB_Set_MAC154_ZIGBEE_CLK(1U);
	GLB_MAC154_ZIGBEE_Reset();
}
