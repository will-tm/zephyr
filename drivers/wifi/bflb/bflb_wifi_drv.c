/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bouffalo Lab BL808 WiFi driver — Zephyr wifi_mgmt_ops integration.
 */

#define DT_DRV_COMPAT bflb_bl808x_wifi

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/conn_mgr/connectivity_wifi_mgmt.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>
#include <stddef.h>

#include <bflb_soc.h>
#include "bl_main.h"
#include "lmac_mac.h"
#include "lmac_msg.h"
#include "ieee80211.h"
#include <bl60x_fw_api.h>

LOG_MODULE_REGISTER(bflb_wifi, LOG_LEVEL_ERR);

/* GLB registers for WiFi clock enable */
#define GLB_CGEN_CFG2_OFFSET 0x588
#define GLB_WIFI_CFG0_OFFSET 0x3B0

static void bflb_wifi_clock_enable(void)
{
	uint32_t regval;

	/* Ungate WiFi clock: CGEN_CFG2 bit 4 */
	regval = sys_read32(GLB_BASE + GLB_CGEN_CFG2_OFFSET);
	regval |= BIT(4);
	sys_write32(regval, GLB_BASE + GLB_CGEN_CFG2_OFFSET);

	/* Set WiFi MAC core clock divider to 1 (40MHz from 80MHz PLL) */
	regval = sys_read32(GLB_BASE + GLB_WIFI_CFG0_OFFSET);
	regval = (regval & ~0xF) | 1; /* MAC core div = 1 */
	sys_write32(regval, GLB_BASE + GLB_WIFI_CFG0_OFFSET);
}

/* Cached AP info from last scan — firmware needs BSSID+channel for connect */
#define SCAN_AP_CACHE_SIZE 16

struct scan_ap_entry {
	uint8_t ssid[WIFI_SSID_MAX_LEN];
	uint8_t ssid_len;
	uint8_t bssid[6];
	uint16_t center_freq;
	int8_t rssi;
};

struct bflb_wifi_data {
	struct net_if *iface;
	uint8_t mac_addr[6];
	scan_result_cb_t scan_cb;
	struct k_work_delayable scan_done_work;
	uint32_t scan_result_count;
	uint32_t scan_timeout_ms;
	int64_t scan_deadline_ms;
	bool initialized;
	enum wifi_iface_state state;
	uint8_t connected_bssid[6];
	uint8_t connected_channel;
	uint8_t connected_ssid[WIFI_SSID_MAX_LEN];
	uint8_t connected_ssid_len;
	enum wifi_security_type connected_security;
	struct scan_ap_entry ap_cache[SCAN_AP_CACHE_SIZE];
	uint8_t ap_cache_count;
};

static struct bflb_wifi_data bflb_wifi_data_inst;

/* Registered with wifi_stubs.c */
extern void bl_wifi_set_scan_callbacks(void (*result_fn)(const uint8_t *, uint16_t,
							 const struct scanu_result_ind *),
				       void (*done_fn)(uint8_t));
extern void bl_wifi_set_connect_callbacks(void (*connect_fn)(const struct sm_connect_ind *),
					  void (*disconnect_fn)(const struct sm_disconnect_ind *));
extern void bl_wifi_set_rx_iface(struct net_if *iface);

/* 802.11 IE parsing: find IE by ID in a buffer of tagged parameters */
static const uint8_t *find_ie(const uint8_t *ies, uint16_t ies_len, uint8_t ie_id)
{
	const uint8_t *pos = ies;
	const uint8_t *end = ies + ies_len;

	while (pos + 2 <= end) {
		uint8_t id = pos[0];
		uint8_t len = pos[1];

		if (pos + 2 + len > end) {
			break;
		}
		if (id == ie_id) {
			return pos;
		}
		pos += 2 + len;
	}
	return NULL;
}

/* 2.4GHz frequency to channel */
static uint8_t freq_to_channel(uint16_t freq)
{
	if (freq >= 2412 && freq <= 2472) {
		return (freq - 2407) / 5;
	}
	if (freq == 2484) {
		return 14;
	}
	return 0;
}

static uint32_t bflb_scan_timeout_ms(uint8_t scan_mode, uint32_t duration_scan_us)
{
	int channel_cnt;
	uint32_t dwell_ms;
	uint32_t timeout_ms;

	channel_cnt = bl_main_get_channel_nums();
	if (channel_cnt <= 0) {
		channel_cnt = 11;
	}

	if (duration_scan_us != 0U) {
		dwell_ms = MAX(1U, duration_scan_us / 1000U);
		timeout_ms = ((uint32_t)channel_cnt * dwell_ms) + 750U;
	} else if (scan_mode == SCAN_PASSIVE) {
		timeout_ms = ((uint32_t)channel_cnt * 120U) + 750U;
	} else {
		timeout_ms = ((uint32_t)channel_cnt * 60U) + 750U;
	}

	return MAX(timeout_ms, 2000U);
}

/* Determine security type from beacon/probe response IEs */
static enum wifi_security_type parse_security(const uint8_t *ies, uint16_t ies_len,
					      uint16_t cap_info)
{
	bool privacy = (cap_info & BIT(4)) != 0;
	const uint8_t *rsn_ie = find_ie(ies, ies_len, 48); /* RSN IE */
	const uint8_t *wpa_ie = NULL;

	/* Search for WPA vendor IE (Microsoft OUI: 00:50:F2:01) */
	const uint8_t *pos = ies;
	const uint8_t *end = ies + ies_len;

	while (pos + 2 <= end) {
		uint8_t len = pos[1];

		if (pos + 2 + len > end) {
			break;
		}
		if (pos[0] == 221 && len >= 4 && pos[2] == 0x00 && pos[3] == 0x50 &&
		    pos[4] == 0xF2 && pos[5] == 0x01) {
			wpa_ie = pos;
			break;
		}
		pos += 2 + len;
	}

	if (rsn_ie) {
		return WIFI_SECURITY_TYPE_PSK; /* WPA2 (simplified) */
	}
	if (wpa_ie) {
		return WIFI_SECURITY_TYPE_WPA_PSK;
	}
	if (privacy) {
		return WIFI_SECURITY_TYPE_WEP;
	}
	return WIFI_SECURITY_TYPE_NONE;
}

static void bflb_on_scan_result(const uint8_t *payload, uint16_t payload_len,
				const struct scanu_result_ind *ind)
{
	struct bflb_wifi_data *data = &bflb_wifi_data_inst;
	const struct ieee80211_mgmt *mgmt;
	const uint8_t *ies;
	const uint8_t *bssid;
	uint16_t ies_len;
	uint16_t cap_info;
	uint16_t frame_len;

	if (!data->scan_cb || !data->iface) {
		return;
	}

	data->scan_result_count++;

	if (payload_len < sizeof(struct ieee80211_mgmt)) {
		return;
	}

	mgmt = (const struct ieee80211_mgmt *)payload;
	frame_len = MIN(payload_len, ind->length);

	if (ieee80211_is_beacon(mgmt->frame_control)) {
		size_t var_offset = offsetof(struct ieee80211_mgmt, u.beacon.variable);

		if (frame_len <= var_offset) {
			return;
		}

		cap_info = mgmt->u.beacon.capab_info;
		ies = mgmt->u.beacon.variable;
		ies_len = frame_len - var_offset;
		bssid = mgmt->bssid;
	} else if (ieee80211_is_probe_resp(mgmt->frame_control)) {
		size_t var_offset = offsetof(struct ieee80211_mgmt, u.probe_resp.variable);

		if (frame_len <= var_offset) {
			return;
		}

		cap_info = mgmt->u.probe_resp.capab_info;
		ies = mgmt->u.probe_resp.variable;
		ies_len = frame_len - var_offset;
		bssid = mgmt->bssid;
	} else {
		return;
	}

	struct wifi_scan_result res = {0};

	/* SSID from IE 0 */
	const uint8_t *ssid_ie = find_ie(ies, ies_len, 0);

	if (ssid_ie) {
		uint8_t ssid_len = ssid_ie[1];

		if (ssid_len > WIFI_SSID_MAX_LEN) {
			ssid_len = WIFI_SSID_MAX_LEN;
		}
		memcpy(res.ssid, ssid_ie + 2, ssid_len);
		res.ssid_length = ssid_len;
	}

	/* Skip hidden SSIDs (length 0) */
	if (res.ssid_length == 0) {
		return;
	}

	memcpy(res.mac, bssid, 6);
	res.mac_length = 6;
	res.rssi = ind->rssi;
	res.channel = freq_to_channel(ind->center_freq);
	res.band = WIFI_FREQ_BAND_2_4_GHZ;
	res.security = parse_security(ies, ies_len, cap_info);

	/* Cache AP info for connect — update existing entry or add new one */
	if (res.ssid_length > 0) {
		struct scan_ap_entry *slot = NULL;

		for (int i = 0; i < data->ap_cache_count; i++) {
			if (data->ap_cache[i].ssid_len == res.ssid_length &&
			    memcmp(data->ap_cache[i].ssid, res.ssid, res.ssid_length) == 0) {
				/* Keep strongest signal */
				if (ind->rssi > data->ap_cache[i].rssi) {
					slot = &data->ap_cache[i];
				}
				break;
			}
		}
		if (!slot && data->ap_cache_count < SCAN_AP_CACHE_SIZE) {
			slot = &data->ap_cache[data->ap_cache_count++];
		}
		if (slot) {
			memcpy(slot->ssid, res.ssid, res.ssid_length);
			slot->ssid_len = res.ssid_length;
			memcpy(slot->bssid, bssid, 6);
			slot->center_freq = ind->center_freq;
			slot->rssi = ind->rssi;
		}
	}

	data->scan_cb(data->iface, 0, &res);

	if (data->scan_deadline_ms > 0) {
		int64_t remaining_ms = data->scan_deadline_ms - k_uptime_get();

		if (remaining_ms < 750) {
			remaining_ms = 750;
		}
		k_work_reschedule(&data->scan_done_work, K_MSEC(remaining_ms));
	} else {
		k_work_reschedule(&data->scan_done_work, K_MSEC(750));
	}
	k_yield();
}

static void bflb_finish_scan(struct bflb_wifi_data *data)
{
	if (!data->scan_cb || !data->iface) {
		return;
	}

	LOG_INF("scan done: %u results", data->scan_result_count);
	data->scan_cb(data->iface, 0, NULL);
	data->scan_cb = NULL;
	data->scan_result_count = 0;
	data->scan_timeout_ms = 0;
	data->scan_deadline_ms = 0;
}

static void bflb_scan_done_work_fn(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct bflb_wifi_data *data = CONTAINER_OF(dwork, struct bflb_wifi_data, scan_done_work);

	bflb_finish_scan(data);
}

static void bflb_on_scan_done(uint8_t status)
{
	struct bflb_wifi_data *data = &bflb_wifi_data_inst;

	if (!data->scan_cb || !data->iface) {
		return;
	}

	if (status != 0) {
		bflb_finish_scan(data);
		return;
	}

	if (data->scan_timeout_ms == 0U) {
		data->scan_timeout_ms = 2000U;
	}
	data->scan_deadline_ms = k_uptime_get() + data->scan_timeout_ms;
	k_work_reschedule(&data->scan_done_work, K_MSEC(data->scan_timeout_ms));
}

static int bflb_wifi_scan(const struct device *dev, struct wifi_scan_params *params,
			  scan_result_cb_t cb)
{
	struct bflb_wifi_data *data = dev->data;
	uint8_t scan_mode = SCAN_ACTIVE;
	uint32_t duration_scan_us = 0;
	int ret;

	if (data->scan_cb) {
		return -EBUSY;
	}

	data->scan_cb = cb;
	data->scan_result_count = 0;
	data->ap_cache_count = 0;

	if (params != NULL) {
		/*
		 * Zephyr scan_type is only a hint. Accept an explicit passive
		 * dwell time as a passive-scan request too.
		 */
		if (params->scan_type == WIFI_SCAN_TYPE_PASSIVE ||
		    (params->dwell_time_passive > 0 && params->dwell_time_active == 0)) {
			scan_mode = SCAN_PASSIVE;
			duration_scan_us = (uint32_t)params->dwell_time_passive * 1000U;
		} else {
			duration_scan_us = (uint32_t)params->dwell_time_active * 1000U;
		}
	}

	data->scan_timeout_ms = bflb_scan_timeout_ms(scan_mode, duration_scan_us);
	data->scan_deadline_ms = 0;

	ret = bl_main_scan(NULL, NULL, 0, NULL, NULL, scan_mode, duration_scan_us);
	if (ret) {
		data->scan_cb = NULL;
		data->scan_timeout_ms = 0;
		data->scan_deadline_ms = 0;
	}
	return ret;
}

static void bflb_on_connect_ind(const struct sm_connect_ind *ind)
{
	struct bflb_wifi_data *data = &bflb_wifi_data_inst;

	if (!data->iface) {
		return;
	}

	if (ind->status_code == WLAN_FW_SUCCESSFUL) {
		data->state = WIFI_STATE_COMPLETED;
		memcpy(data->connected_bssid, ind->bssid.array, 6);
		data->connected_channel = freq_to_channel(ind->center_freq);
		LOG_INF("connected to %02x:%02x:%02x:%02x:%02x:%02x ch=%u", ind->bssid.array[0],
			ind->bssid.array[1], ind->bssid.array[2], ind->bssid.array[3],
			ind->bssid.array[4], ind->bssid.array[5], data->connected_channel);
		wifi_mgmt_raise_connect_result_event(data->iface, 0);
	} else {
		data->state = WIFI_STATE_DISCONNECTED;
		LOG_WRN("connect failed: status_code=%u reason_code=%u "
			"assoc_req_ie_len=%u assoc_rsp_ie_len=%u",
			ind->status_code, ind->reason_code, ind->assoc_req_ie_len,
			ind->assoc_rsp_ie_len);
		/* Dump assoc request IEs for debugging — walk tagged params */
		if (ind->assoc_req_ie_len > 0) {
			const uint8_t *ie = (const uint8_t *)ind->assoc_ie_buf;
			const uint8_t *ie_end =
				ie + MIN(ind->assoc_req_ie_len, (uint16_t)SM_ASSOC_IE_LEN);

			while (ie + 2 <= ie_end && ie + 2 + ie[1] <= ie_end) {
				LOG_WRN("  IE id=%u len=%u", ie[0], ie[1]);
				ie += 2 + ie[1];
			}
		}
		wifi_mgmt_raise_connect_result_event(data->iface, -1);
	}
}

static void bflb_on_disconnect_ind(const struct sm_disconnect_ind *ind)
{
	struct bflb_wifi_data *data = &bflb_wifi_data_inst;

	if (!data->iface) {
		return;
	}

	LOG_INF("disconnected: status_code=%u reason_code=%u", ind->status_code, ind->reason_code);
	data->state = WIFI_STATE_DISCONNECTED;
	memset(data->connected_bssid, 0, sizeof(data->connected_bssid));
	data->connected_channel = 0;
	data->connected_ssid_len = 0;
	wifi_mgmt_raise_disconnect_result_event(data->iface, 0);
}

static const struct scan_ap_entry *bflb_find_ap(struct bflb_wifi_data *data, const uint8_t *ssid,
						uint8_t ssid_len)
{
	for (int i = 0; i < data->ap_cache_count; i++) {
		if (data->ap_cache[i].ssid_len == ssid_len &&
		    memcmp(data->ap_cache[i].ssid, ssid, ssid_len) == 0) {
			return &data->ap_cache[i];
		}
	}
	return NULL;
}

static int bflb_wifi_connect(const struct device *dev, struct wifi_connect_req_params *params)
{
	struct bflb_wifi_data *data = dev->data;
	const uint8_t *psk = NULL;
	int psk_len = 0;
	const uint8_t *bssid = NULL;
	uint16_t freq = 0;
	int ret;

	if (data->state == WIFI_STATE_COMPLETED) {
		return -EALREADY;
	}
	if (data->state == WIFI_STATE_ASSOCIATING) {
		return -EINPROGRESS;
	}

	if (params->security != WIFI_SECURITY_TYPE_NONE) {
		psk = params->psk;
		psk_len = params->psk_length;
	}

	/* Look up BSSID + channel from scan cache */
	const struct scan_ap_entry *ap = bflb_find_ap(data, params->ssid, params->ssid_length);

	if (ap) {
		bssid = ap->bssid;
		freq = ap->center_freq;
		LOG_INF("AP found in cache: bssid=%02x:%02x:%02x:%02x:%02x:%02x freq=%u", bssid[0],
			bssid[1], bssid[2], bssid[3], bssid[4], bssid[5], freq);
	} else {
		LOG_WRN("AP \"%.*s\" not in scan cache — connecting without BSSID/channel",
			params->ssid_length, params->ssid);
	}

	/* Save connection params for status reporting */
	memcpy(data->connected_ssid, params->ssid, params->ssid_length);
	data->connected_ssid_len = params->ssid_length;
	data->connected_security = params->security;
	data->state = WIFI_STATE_ASSOCIATING;

	LOG_INF("connecting to \"%.*s\" security=%u", params->ssid_length, params->ssid,
		params->security);

	ret = bl_main_connect(params->ssid, params->ssid_length, psk, psk_len, NULL, 0, bssid, 0,
			      freq, 0);
	if (ret) {
		data->state = WIFI_STATE_DISCONNECTED;
		LOG_ERR("bl_main_connect failed: %d", ret);
	}
	return ret;
}

static int bflb_wifi_disconnect(const struct device *dev)
{
	struct bflb_wifi_data *data = dev->data;

	if (data->state == WIFI_STATE_DISCONNECTED) {
		return -EALREADY;
	}

	LOG_INF("disconnecting");
	data->state = WIFI_STATE_DISCONNECTED;
	data->connected_ssid_len = 0;
	/* carrier_off handled in bflb_on_disconnect_ind when firmware confirms */
	return bl_main_disconnect();
}

static int bflb_wifi_status(const struct device *dev, struct wifi_iface_status *status)
{
	struct bflb_wifi_data *data = dev->data;

	memset(status, 0, sizeof(*status));
	status->state = data->state;
	status->band = WIFI_FREQ_BAND_2_4_GHZ;

	if (data->state == WIFI_STATE_COMPLETED) {
		memcpy(status->bssid, data->connected_bssid, 6);
		status->channel = data->connected_channel;
		memcpy(status->ssid, data->connected_ssid, data->connected_ssid_len);
		status->ssid_len = data->connected_ssid_len;
		status->security = data->connected_security;
		status->link_mode = WIFI_4;
		status->iface_mode = WIFI_MODE_INFRA;
	}

	return 0;
}

static const struct wifi_mgmt_ops bflb_wifi_mgmt = {
	.scan = bflb_wifi_scan,
	.connect = bflb_wifi_connect,
	.disconnect = bflb_wifi_disconnect,
	.iface_status = bflb_wifi_status,
};

extern int bl_output_raw(const uint8_t *frame, uint16_t len,
			 uint8_t vif_idx, uint8_t sta_idx);

static int bflb_wifi_send(const struct device *dev, struct net_pkt *pkt)
{
	struct bflb_wifi_data *data = dev->data;
	uint16_t len;
	uint8_t buf[NET_ETH_MTU + sizeof(struct net_eth_hdr)];
	int ret;

	if (data->state != WIFI_STATE_COMPLETED) {
		return -ENETDOWN;
	}

	len = net_pkt_get_len(pkt);
	if (len > sizeof(buf)) {
		return -ENOMEM;
	}

	if (net_pkt_read(pkt, buf, len) < 0) {
		return -EIO;
	}

	ret = bl_output_raw(buf, len, 0, 0);
	return ret;
}

static void bflb_wifi_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct bflb_wifi_data *data = dev->data;
	ssize_t ret;

	data->iface = iface;

	ret = hwinfo_get_device_id(data->mac_addr, sizeof(data->mac_addr));
	if (ret != sizeof(data->mac_addr)) {
		memset(data->mac_addr, 0, sizeof(data->mac_addr));
		LOG_WRN("Failed to read MAC from efuse, using default");
	}

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr), NET_LINK_ETHERNET);
	bl_main_set_mac_addr(data->mac_addr);
	bl_wifi_set_rx_iface(iface);

	/* Initialize Ethernet L2 (ARP cache, carrier work, etc.) and mark
	 * as WiFi interface.  ethernet_init sets eth_if_type via ctx, and
	 * every ETHERNET_L2 driver must call it — see nrf_wifi, esp32, etc.
	 */
	ethernet_init(iface);

	struct ethernet_context *eth_ctx = net_if_l2_data(iface);

	eth_ctx->eth_if_type = L2_ETH_IF_TYPE_WIFI;

	LOG_INF("WiFi MAC: %02x:%02x:%02x:%02x:%02x:%02x", data->mac_addr[0], data->mac_addr[1],
		data->mac_addr[2], data->mac_addr[3], data->mac_addr[4], data->mac_addr[5]);
}

static enum ethernet_hw_caps bflb_wifi_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);
	return (enum ethernet_hw_caps)0;
}

static const struct net_wifi_mgmt_offload bflb_wifi_api = {
	.wifi_iface.iface_api.init = bflb_wifi_iface_init,
	.wifi_iface.get_capabilities = bflb_wifi_get_capabilities,
	.wifi_iface.send = bflb_wifi_send,
	.wifi_mgmt_api = &bflb_wifi_mgmt,
};

static int bflb_wifi_init(const struct device *dev)
{
	bflb_wifi_clock_enable();
	bl_wifi_set_scan_callbacks(bflb_on_scan_result, bflb_on_scan_done);
	bl_wifi_set_connect_callbacks(bflb_on_connect_ind, bflb_on_disconnect_ind);
	k_work_init_delayable(&bflb_wifi_data_inst.scan_done_work, bflb_scan_done_work_fn);

	struct bl_hw *bl_hw;
	int ret = bl_main_rtthread_start(&bl_hw);

	if (ret) {
		LOG_ERR("bl_main_rtthread_start failed: %d", ret);
		return ret;
	}
	return 0;
}

NET_DEVICE_DT_INST_DEFINE(0, bflb_wifi_init, NULL, &bflb_wifi_data_inst, NULL,
			  CONFIG_WIFI_INIT_PRIORITY, &bflb_wifi_api, ETHERNET_L2,
			  NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);

CONNECTIVITY_WIFI_MGMT_BIND(Z_DEVICE_DT_DEV_ID(DT_DRV_INST(0)));
