/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Firmware-to-host message handler and stubs for libwifi.a.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(bl_wifi_stubs, LOG_LEVEL_ERR);

#include "bl_defs.h"
#include "ipc_shared.h"
#include <bl60x_fw_api.h>
#include "lmac_msg.h"

extern struct bl_hw wifi_hw;

/*
 * Scan result callbacks — registered by the Zephyr WiFi driver.
 */
static void (*scan_result_cb)(const uint8_t *payload, uint16_t len,
			      const struct scanu_result_ind *hdr);
static void (*scan_done_cb)(uint8_t status);

void bl_wifi_set_scan_callbacks(void (*result_fn)(const uint8_t *, uint16_t,
						  const struct scanu_result_ind *),
				void (*done_fn)(uint8_t))
{
	scan_result_cb = result_fn;
	scan_done_cb = done_fn;
}

/*
 * Connect/disconnect indication callbacks — registered by the Zephyr WiFi driver.
 */
static void (*connect_ind_cb)(const struct sm_connect_ind *ind);
static void (*disconnect_ind_cb)(const struct sm_disconnect_ind *ind);

void bl_wifi_set_connect_callbacks(void (*connect_fn)(const struct sm_connect_ind *),
				   void (*disconnect_fn)(const struct sm_disconnect_ind *))
{
	connect_ind_cb = connect_fn;
	disconnect_ind_cb = disconnect_fn;
}

/*
 * bl_rx_e2a_handler — firmware-to-host message handler.
 *
 * Called directly by the blob (ke_msg delivery) when the LMAC firmware
 * sends a response or event message to the host. The argument is a
 * pointer to the ke_msg fields (past list header).
 * Field order: [id:2] [dest_id:2] [param_len:2] [src_id:2] [param...]
 */
void bl_rx_e2a_handler(void *arg)
{
	const uint16_t *hdr = (const uint16_t *)arg;
	const uint8_t *params = (const uint8_t *)arg + 8;
	struct ipc_e2a_msg msg = {0};
	uint16_t id;
	uint16_t dest;
	uint16_t plen;
	uint16_t src;

	if (arg == NULL) {
		return;
	}

	id = hdr[0];
	dest = hdr[1];
	plen = hdr[2];
	src = hdr[3];

	if (plen > sizeof(msg.param)) {
		LOG_WRN("E2A: oversized param_len=%u for id=0x%04x", plen, id);
		return;
	}

	msg.id = id;
	msg.dummy_dest_id = dest;
	msg.dummy_src_id = src;
	msg.param_len = plen;
	if (plen) {
		memcpy(msg.param, params, plen);
	}

	/* Handle scan indications directly — they don't match pending cmds */
	if (msg.id == SCANU_RESULT_IND) {
		if (scan_result_cb && msg.param_len >= sizeof(struct scanu_result_ind)) {
			const struct scanu_result_ind *ind =
				(const struct scanu_result_ind *)msg.param;
			uint16_t payload_len =
				(uint16_t)(msg.param_len - sizeof(struct scanu_result_ind));
			scan_result_cb((const uint8_t *)ind->payload, payload_len, ind);
		} else {
			LOG_DBG("scan result dropped param_len=%u", msg.param_len);
		}
		return;
	}

	if (msg.id == SCANU_START_CFM) {
		uint8_t status = 0;

		if (msg.param_len >= 1) {
			status = ((const uint8_t *)msg.param)[0];
		}
		LOG_DBG("scan start cfm status=%u", status);
		if (scan_done_cb) {
			scan_done_cb(status);
		}
		return;
	}

	if (msg.id == SM_CONNECT_IND) {
		if (connect_ind_cb && msg.param_len >= sizeof(struct sm_connect_ind)) {
			connect_ind_cb((const struct sm_connect_ind *)msg.param);
		} else {
			LOG_WRN("SM_CONNECT_IND dropped (no cb or short param_len=%u)",
				msg.param_len);
		}
		return;
	}

	if (msg.id == SM_DISCONNECT_IND) {
		if (disconnect_ind_cb && msg.param_len >= sizeof(struct sm_disconnect_ind)) {
			disconnect_ind_cb((const struct sm_disconnect_ind *)msg.param);
		} else {
			LOG_WRN("SM_DISCONNECT_IND dropped (no cb or short param_len=%u)",
				msg.param_len);
		}
		return;
	}

	LOG_DBG("E2A: id=0x%04x plen=%u", msg.id, msg.param_len);

	wifi_hw.cmd_mgr.msgind(&wifi_hw.cmd_mgr, &msg, NULL);
}

/*
 * Minimal WPA supplicant stubs.
 *
 * The firmware dereferences the wpa_cbs function pointer table
 * without NULL checks (in vif_mgmt_register and mm_sta_add).
 * Register no-op stubs to prevent crashes during ADD_IF/STA_ADD.
 */
#include <supplicant_api.h>

/* RSN IE shared with wpa_supplicant.c */
uint8_t wpa_rsn_ie[22];
uint8_t wpa_rsn_ie_len;

/* WPA supplicant interface */
extern void wpa_sm_init(const wifi_connect_parm_t *p);
extern void wpa_sm_reset(void);
extern int wpa_sm_rx_eapol(uint8_t *src, uint8_t *buf, uint32_t len);

static bool wpa_sta_init_stub(void)
{
	return true;
}
static bool wpa_sta_deinit_stub(void)
{
	return true;
}
/*
 * Build and install an RSN IE so the firmware includes it in the
 * Association Request.  Without this, the AP rejects the association
 * for WPA2 networks and the firmware reports AUTH_FAILURE.
 */
static void wpa_sta_config_impl(wifi_connect_parm_t *p)
{
	if (!p) {
		return;
	}

	LOG_INF("wpa_sta_config: proto=%u key_mgmt=0x%x pairwise=%u group=%u", p->proto,
		p->key_mgmt, p->pairwise_cipher, p->group_cipher);

	/* proto low byte is sec_proto_t; upper bits may have flags */
	uint8_t sec_proto = (uint8_t)(p->proto & 0xFF);

	if (sec_proto < SEC_PROTO_WPA2 &&
	    !(p->key_mgmt & (WPA_KEY_MGMT_PSK | WPA_KEY_MGMT_SAE | WPA_KEY_MGMT_PSK_SHA256))) {
		/* Open/WEP/WPA1 — no RSN IE needed */
		return;
	}

	/*
	 * The firmware populates key_mgmt from its own IE parsing but
	 * leaves pairwise_cipher and group_cipher as 0 (NONE).  We must
	 * fill these in — update_ap_info validates them and fails if they
	 * are NONE for a WPA2 connection.
	 */
	if (p->pairwise_cipher == WIFI_CIPHER_TYPE_NONE) {
		p->pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;
	}
	if (p->group_cipher == WIFI_CIPHER_TYPE_NONE) {
		p->group_cipher = WIFI_CIPHER_TYPE_CCMP;
	}

	/* Select PSK AKM — we have no SAE implementation */
	bool ap_has_sae = (p->key_mgmt & WPA_KEY_MGMT_SAE) != 0;

	p->key_mgmt = WPA_KEY_MGMT_PSK;

	/* Transition mode APs require MFPC even for PSK clients */
	if (ap_has_sae) {
		p->pmf_required = false; /* not required, but capable */
		p->mgmt_group_cipher = WIFI_CIPHER_TYPE_AES_CMAC128;
	}

	/*
	 * Build RSN IE for WPA2-PSK:
	 *   Tag=48, Len=20
	 *   Version: 1
	 *   Group cipher: CCMP
	 *   Pairwise count: 1, Pairwise cipher: CCMP
	 *   AKM count: 1, AKM suite: PSK (02)
	 *   RSN capabilities
	 */
	uint8_t group_suite = (p->group_cipher == WIFI_CIPHER_TYPE_TKIP) ? 2 : 4;
	uint8_t pairwise_suite = (p->pairwise_cipher == WIFI_CIPHER_TYPE_TKIP) ? 2 : 4;
	uint8_t rsn_caps = ap_has_sae ? 0x80 : 0x00; /* MFPC if transition mode */

	wpa_rsn_ie[0] = 48; /* RSN element ID */
	wpa_rsn_ie[1] = 20; /* length */
	wpa_rsn_ie[2] = 1;  /* version LSB */
	wpa_rsn_ie[3] = 0;  /* version MSB */
	/* Group cipher suite */
	wpa_rsn_ie[4] = 0x00;
	wpa_rsn_ie[5] = 0x0f;
	wpa_rsn_ie[6] = 0xac;
	wpa_rsn_ie[7] = group_suite;
	/* Pairwise cipher count */
	wpa_rsn_ie[8] = 1;
	wpa_rsn_ie[9] = 0;
	/* Pairwise cipher suite */
	wpa_rsn_ie[10] = 0x00;
	wpa_rsn_ie[11] = 0x0f;
	wpa_rsn_ie[12] = 0xac;
	wpa_rsn_ie[13] = pairwise_suite;
	/* AKM count */
	wpa_rsn_ie[14] = 1;
	wpa_rsn_ie[15] = 0;
	/* AKM suite: PSK */
	wpa_rsn_ie[16] = 0x00;
	wpa_rsn_ie[17] = 0x0f;
	wpa_rsn_ie[18] = 0xac;
	wpa_rsn_ie[19] = 2;
	wpa_rsn_ie[20] = rsn_caps;
	wpa_rsn_ie[21] = 0x00;

	LOG_INF("RSN IE: group=%u pairwise=%u akm=PSK caps=0x%02x", group_suite, pairwise_suite,
		rsn_caps);

	wpa_rsn_ie_len = sizeof(wpa_rsn_ie);

	int ret = bl_wifi_set_appie_internal(p->vif_idx, WIFI_APPIE_WPA_RSN, wpa_rsn_ie,
					     wpa_rsn_ie_len, true);
	LOG_INF("set_appie RSN ret=%d vif=%u", ret, p->vif_idx);

	/* Note: bl_wifi_sta_update_ap_info_internal() is a stub (always
	 * returns 1) in the current blob — call it anyway for compatibility.
	 */
	bl_wifi_sta_update_ap_info_internal();
}

static void wpa_sta_connect_impl(wifi_connect_parm_t *p)
{
	if (p) {
		LOG_INF("wpa_sta_connect: ssid=%.*s proto=%u sta=%u "
			"bssid=%02x:%02x:%02x:%02x:%02x:%02x",
			p->ssid.len, p->ssid.ssid, p->proto, p->sta_idx,
			p->bssid[0], p->bssid[1], p->bssid[2],
			p->bssid[3], p->bssid[4], p->bssid[5]);
		wpa_sm_init(p);
	}
}
static void wpa_sta_disconnected_stub(uint8_t r)
{
	ARG_UNUSED(r);
	wpa_sm_reset();
}

static int wpa_sta_rx_eapol_impl(uint8_t *src, uint8_t *buf, uint32_t len)
{
	if (!src || !buf) {
		return -1;
	}
	LOG_DBG("EAPOL RX: src=%02x:%02x:%02x:%02x:%02x:%02x len=%u", src[0], src[1], src[2],
		src[3], src[4], src[5], len);
	return wpa_sm_rx_eapol(src, buf, len);
}

static void *wpa_ap_init_stub(wifi_ap_parm_t *p)
{
	ARG_UNUSED(p);
	return NULL;
}
static bool wpa_ap_deinit_stub(void *d)
{
	ARG_UNUSED(d);
	return true;
}

static bool wpa_ap_join_stub(void **sm, uint8_t *mac, uint8_t *ie, uint8_t len)
{
	ARG_UNUSED(sm);
	ARG_UNUSED(mac);
	ARG_UNUSED(ie);
	ARG_UNUSED(len);
	return false;
}

static void wpa_ap_sta_assoc_stub(void *sm, uint8_t idx)
{
	ARG_UNUSED(sm);
	ARG_UNUSED(idx);
}

static bool wpa_ap_remove_stub(void *sm)
{
	ARG_UNUSED(sm);
	return true;
}

static bool wpa_ap_rx_eapol_stub(void *hapd, void *sm, uint8_t *data, size_t len)
{
	ARG_UNUSED(hapd);
	ARG_UNUSED(sm);
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	return false;
}

/*
 * Parse an RSN or WPA IE into wifi_wpa_ie_t.
 *
 * The firmware calls this to understand the AP's security capabilities
 * from scan results.  Returning all zeros causes the firmware to fail
 * connection setup.
 */
static uint8_t rsn_suite_to_cipher(const uint8_t *oui_suite)
{
	/* 00:0f:ac:XX for RSN, 00:50:f2:XX for WPA */
	switch (oui_suite[3]) {
	case 1:
		return WIFI_CIPHER_TYPE_WEP40;
	case 2:
		return WIFI_CIPHER_TYPE_TKIP;
	case 4:
		return WIFI_CIPHER_TYPE_CCMP;
	case 5:
		return WIFI_CIPHER_TYPE_WEP104;
	case 6:
		return WIFI_CIPHER_TYPE_AES_CMAC128;
	default:
		return WIFI_CIPHER_TYPE_UNKNOWN;
	}
}

static int wpa_parse_wpa_ie_impl(const uint8_t *ie, size_t len, wifi_wpa_ie_t *d)
{
	const uint8_t *pos;
	const uint8_t *end;
	uint16_t count;
	bool is_rsn;

	if (!d || !ie || len < 2) {
		return -1;
	}

	memset(d, 0, sizeof(*d));

	/* Determine IE type: RSN (tag 48) or WPA (tag 221 with OUI 00:50:f2:01) */
	if (ie[0] == 48) {
		is_rsn = true;
		pos = ie + 2; /* skip tag + length */
		end = ie + 2 + ie[1];
	} else if (ie[0] == 221 && len >= 6 && ie[2] == 0x00 && ie[3] == 0x50 && ie[4] == 0xf2 &&
		   ie[5] == 0x01) {
		is_rsn = false;
		pos = ie + 6; /* skip tag + length + OUI + type */
		end = ie + 2 + ie[1];
	} else {
		return -1;
	}

	d->proto = is_rsn ? WPA_PROTO_RSN : WPA_PROTO_WPA;

	/* Version (2 bytes) */
	if (pos + 2 > end) {
		return -1;
	}
	pos += 2;

	/* Group cipher suite (4 bytes) */
	if (pos + 4 > end) {
		return 0; /* partial IE is OK */
	}
	d->group_cipher = rsn_suite_to_cipher(pos);
	pos += 4;

	/* Pairwise cipher suite count + list */
	if (pos + 2 > end) {
		return 0;
	}
	count = pos[0] | (pos[1] << 8);
	pos += 2;

	if (count > 0 && pos + 4 <= end) {
		d->pairwise_cipher = rsn_suite_to_cipher(pos);
	}
	pos += count * 4;

	/* AKM suite count + list */
	if (pos + 2 > end) {
		return 0;
	}
	count = pos[0] | (pos[1] << 8);
	pos += 2;

	if (count > 0 && pos + 4 <= end) {
		uint8_t akm_type = pos[3];

		/* Map AKM suite type to key_mgmt bitmask */
		switch (akm_type) {
		case 1:
			d->key_mgmt = WPA_KEY_MGMT_IEEE8021X;
			break;
		case 2:
			d->key_mgmt = WPA_KEY_MGMT_PSK;
			break;
		case 6:
			d->key_mgmt = is_rsn ? WPA_KEY_MGMT_PSK_SHA256 : WPA_KEY_MGMT_PSK;
			break;
		case 8:
			d->key_mgmt = WPA_KEY_MGMT_SAE;
			break;
		default:
			d->key_mgmt = WPA_KEY_MGMT_PSK;
			break;
		}
	}
	pos += count * 4;

	/* RSN capabilities (2 bytes) */
	if (pos + 2 <= end) {
		d->capabilities = pos[0] | (pos[1] << 8);
	}

	LOG_DBG("wpa_parse_ie: proto=%d group=%d pairwise=%d key_mgmt=0x%x caps=0x%x", d->proto,
		d->group_cipher, d->pairwise_cipher, d->key_mgmt, d->capabilities);

	return 0;
}

static void wpa_reg_diag_tlv_stub(void *cb)
{
	ARG_UNUSED(cb);
}

static uint8_t *wpa3_build_sae_msg_stub(uint8_t *bssid, uint8_t *mac, uint8_t *pass, uint32_t type,
					size_t *len)
{
	ARG_UNUSED(bssid);
	ARG_UNUSED(mac);
	ARG_UNUSED(pass);
	ARG_UNUSED(type);
	ARG_UNUSED(len);
	return NULL;
}

static int wpa3_parse_sae_msg_stub(uint8_t *buf, size_t len, uint32_t type, uint16_t status)
{
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
	ARG_UNUSED(type);
	ARG_UNUSED(status);
	return -1;
}

static void wpa3_clear_sae_stub(void)
{
}

static const struct wpa_funcs wpa_stub_cbs = {
	.wpa_sta_init = wpa_sta_init_stub,
	.wpa_sta_deinit = wpa_sta_deinit_stub,
	.wpa_sta_config = wpa_sta_config_impl,
	.wpa_sta_connect = wpa_sta_connect_impl,
	.wpa_sta_disconnected_cb = wpa_sta_disconnected_stub,
	.wpa_sta_rx_eapol = wpa_sta_rx_eapol_impl,
	.wpa_ap_init = wpa_ap_init_stub,
	.wpa_ap_deinit = wpa_ap_deinit_stub,
	.wpa_ap_join = wpa_ap_join_stub,
	.wpa_ap_sta_associated = wpa_ap_sta_assoc_stub,
	.wpa_ap_remove = wpa_ap_remove_stub,
	.wpa_ap_rx_eapol = wpa_ap_rx_eapol_stub,
	.wpa_parse_wpa_ie = wpa_parse_wpa_ie_impl,
	.wpa_reg_diag_tlv_cb = wpa_reg_diag_tlv_stub,
	.wpa3_build_sae_msg = wpa3_build_sae_msg_stub,
	.wpa3_parse_sae_msg = wpa3_parse_sae_msg_stub,
	.wpa3_clear_sae = wpa3_clear_sae_stub,
};

int bl_supplicant_init(void)
{
	LOG_DBG("registering WPA stub callbacks");
	bl_wifi_register_wpa_cb_internal(&wpa_stub_cbs);
	return 0;
}

/*
 * Override printf to sink calls from binary blobs (RF calibration spam
 * from libbl606p_phyrf.a).  This library is linked with --whole-archive
 * so our definition takes precedence over picolibc's printf, which would
 * otherwise call vfprintf(stdout, ...) and crash because stdout's put
 * function pointer is NULL in bare-metal Zephyr.
 */
int printf(const char *fmt, ...)
{
	ARG_UNUSED(fmt);
	return 0;
}
