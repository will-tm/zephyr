/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * RF library callbacks required by libbl702_rf.a.
 * These are called by the RF driver during calibration and reset events.
 */

#include <zephyr/kernel.h>
#include <stdint.h>

#if defined(CONFIG_IEEE802154_BFLB)
/* Forward declarations from bl702_rf.h */
extern void rf_set_bz_mode(uint8_t mode);

/* From bl702_phy.h */
extern void bz_phy_set_tx_power_offset(int8_t poweroffset_zigbee[16], int8_t poweroffset_ble[4]);

/* From bl_wireless shims */
extern int bl_wireless_power_offset_get(int8_t poweroffset_zigbee[16], int8_t poweroffset_ble[4]);

/* Mode constants from bl702_rf.h */
#define MODE_BLE_ONLY 0
#define MODE_ZB_ONLY  1
#define MODE_BZ_COEX  2
#endif /* CONFIG_IEEE802154_BFLB */

/*
 * rf_reset_done_callback — Called after RF reset completes.
 * Set operating mode and apply power offsets when IEEE 802.15.4 is enabled.
 */
void rf_reset_done_callback(void)
{
#if defined(CONFIG_IEEE802154_BFLB)
	int8_t po_zigbee[16];
	int8_t po_ble[4];

	rf_set_bz_mode(MODE_ZB_ONLY);
	bl_wireless_power_offset_get(po_zigbee, po_ble);
	bz_phy_set_tx_power_offset(po_zigbee, po_ble);
#endif
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
