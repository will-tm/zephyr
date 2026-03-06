/*
 * Copyright (C) Bouffalo Lab 2016-2018
 * SPDX-License-Identifier: Apache-2.0
 *
 * Module parameters — ported from M1s SDK.
 */

#include "bl_defs.h"

struct bl_mod_params bl_mod_params = {
	.ht_on = true,
	.vht_on = false,
	.mcs_map = IEEE80211_VHT_MCS_SUPPORT_0_7,
	.phy_cfg = 2,
	.uapsd_timeout = 3000,
	.sgi = false,
	.sgi80 = false,
	.listen_itv = 1,
	.listen_bcmc = true,
	.lp_clk_ppm = 20,
	.ps_on = false,
	.tx_lft = RWNX_TX_LIFETIME_MS,
	.amsdu_maxnb = NX_TX_PAYLOAD_MAX,
	.uapsd_queues = 0,
};

int bl_handle_dynparams(struct bl_hw *bl_hw)
{
	const int nss = 1;

	if (bl_hw->mod_params->phy_cfg < 0 || bl_hw->mod_params->phy_cfg > 5) {
		bl_hw->mod_params->phy_cfg = 2;
	}

	if (bl_hw->mod_params->mcs_map < 0 || bl_hw->mod_params->mcs_map > 2) {
		bl_hw->mod_params->mcs_map = 0;
	}

	/* HT capabilities */
	bl_hw->ht_cap.cap |= 1 << IEEE80211_HT_CAP_RX_STBC_SHIFT;
	bl_hw->ht_cap.mcs.rx_highest = cpu_to_le16(65 * nss);
	bl_hw->ht_cap.mcs.rx_mask[0] = 0xFF;

	if (bl_hw->mod_params->sgi) {
		bl_hw->ht_cap.cap |= IEEE80211_HT_CAP_SGI_20;
		bl_hw->ht_cap.mcs.rx_highest = cpu_to_le16(72 * nss);
	}
	bl_hw->ht_cap.cap |= IEEE80211_HT_CAP_SM_PS;
	if (!bl_hw->mod_params->ht_on) {
		bl_hw->ht_cap.ht_supported = false;
	}

	return 0;
}
