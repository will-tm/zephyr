/*
 * Copyright (C) Bouffalo Lab 2016-2018
 * SPDX-License-Identifier: Apache-2.0
 *
 * Core WiFi driver — ported from M1s SDK.
 * lwIP/netif removed; uses Zephyr IRQ API for interrupt setup.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/util.h>

#include "bl_main.h"
#include "bl_defs.h"
#include "bl_utils.h"
#include "bl_platform.h"
#include "bl_msg_tx.h"
#include "bl_irqs.h"
#include "bl_tx.h"
#include "bl_mod_params.h"
#include "bl_os_private.h"
#include "reg_ipc_app.h"

#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/sys/libc-hooks.h>
#include <zephyr/sys/printk-hooks.h>
#include <zephyr/logging/log.h>
#include <bflb_soc.h>

LOG_MODULE_REGISTER(bflb_wifi_main, LOG_LEVEL_ERR);

/* Use Zephyr's ARG_UNUSED instead of vendor USER_UNUSED */

struct bl_hw wifi_hw;
static uint8_t wifi_mac_addr[6] = {0x18, 0xB9, 0x05, 0x00, 0x00, 0x01};

#define WIFI_NETIF_FLAG_UP        0x01
#define WIFI_NETIF_FLAG_BROADCAST 0x02
#define WIFI_NETIF_FLAG_ETHARP    0x08
#define WIFI_NETIF_FLAG_IGMP      0x20

/*
 * Layout-compatible subset of lwIP's struct netif for the BL808 Wi-Fi blob.
 *
 * The vendor build uses lwIP directly, and both host code and blob code can
 * dereference fields like input/state/hwaddr through this object. A tiny
 * stand-in with hwaddr at the front is not ABI-safe.
 */
struct pbuf;

typedef int8_t err_t;
typedef err_t (*netif_input_fn)(struct pbuf *p, struct netif *inp);
typedef err_t (*netif_output_fn)(struct netif *netif, struct pbuf *p, const void *ipaddr);
typedef err_t (*netif_linkoutput_fn)(struct netif *netif, struct pbuf *p);
typedef void (*netif_status_callback_fn)(struct netif *netif);
typedef err_t (*netif_igmp_mac_filter_fn)(struct netif *netif, const void *group, int action);
typedef void (*dhcp_quick_connect_callback_fn)(struct netif *netif);

typedef uint32_t ip_addr_t;

struct addr_ext {
	uint8_t arp_for_us_disable;
	dhcp_quick_connect_callback_fn dhcp_qc_callback;
};

struct netif {
	struct netif *next;
	ip_addr_t ip_addr;
	ip_addr_t netmask;
	ip_addr_t gw;
	netif_input_fn input;
	netif_output_fn output;
	netif_linkoutput_fn linkoutput;
	netif_status_callback_fn status_callback;
	netif_status_callback_fn link_callback;
	void *state;
	void *client_data[3];
	const char *hostname;
	uint16_t mtu;
	uint8_t hwaddr[6];
	uint8_t hwaddr_len;
	uint8_t flags;
	char name[2];
	uint8_t num;
	netif_igmp_mac_filter_fn igmp_mac_filter;
	struct pbuf *loop_first;
	struct pbuf *loop_last;
	struct addr_ext addr_ext;
};

struct wlan_netif {
	int mode;
	uint8_t vif_index;
	uint8_t mac[6];
	uint8_t dhcp_started;
	struct {
		uint32_t ip;
		uint32_t mask;
		uint32_t gw;
		uint32_t dns1;
		uint32_t dns2;
	} ipv4;
	struct netif netif;
	union {
		struct {
			int8_t rssi;
		} sta;
	};
};

static err_t wifi_netif_input_stub(struct pbuf *p, struct netif *inp)
{
	ARG_UNUSED(p);
	ARG_UNUSED(inp);
	return 0;
}

static err_t wifi_netif_output_stub(struct netif *netif, struct pbuf *p, const void *ipaddr)
{
	ARG_UNUSED(netif);
	ARG_UNUSED(p);
	ARG_UNUSED(ipaddr);
	return 0;
}

static err_t wifi_netif_linkoutput_stub(struct netif *netif, struct pbuf *p)
{
	ARG_UNUSED(netif);
	ARG_UNUSED(p);
	return 0;
}

static void wifi_netif_status_stub(struct netif *netif)
{
	ARG_UNUSED(netif);
}

static void wifi_netif_link_stub(struct netif *netif)
{
	ARG_UNUSED(netif);
}

static struct wlan_netif wifi_sta;
static struct wlan_netif wifi_ap;
static bool wifi_sta_enabled;
static bool wifi_ap_enabled;

static void wifi_netif_init(struct wlan_netif *wlan, int mode, char name0, char name1,
			    const uint8_t *mac)
{
	struct netif *netif = &wlan->netif;

	memset(wlan, 0, sizeof(*wlan));
	wlan->mode = mode;
	wlan->vif_index = 0xff;
	memcpy(wlan->mac, mac, 6);
	memset(netif, 0, sizeof(*netif));
	netif->input = wifi_netif_input_stub;
	netif->output = wifi_netif_output_stub;
	netif->linkoutput = wifi_netif_linkoutput_stub;
	netif->status_callback = wifi_netif_status_stub;
	netif->link_callback = wifi_netif_link_stub;
	netif->state = wlan;
	netif->mtu = 1500;
	memcpy(netif->hwaddr, mac, 6);
	netif->hwaddr_len = 6;
	netif->flags = WIFI_NETIF_FLAG_UP | WIFI_NETIF_FLAG_BROADCAST | WIFI_NETIF_FLAG_ETHARP |
		       WIFI_NETIF_FLAG_IGMP;
	netif->name[0] = name0;
	netif->name[1] = name1;
}

static void wifi_netifs_refresh(void)
{
	wifi_netif_init(&wifi_sta, 0, 's', 't', wifi_mac_addr);
	wifi_netif_init(&wifi_ap, 1, 'a', 'p', wifi_mac_addr);
}

struct netif *wifi_mgmr_sta_netif_get(void)
{
	return &wifi_sta.netif;
}

struct netif *wifi_mgmr_ap_netif_get(void)
{
	return &wifi_ap.netif;
}

wifi_interface_t wifi_mgmr_sta_enable(void)
{
	if (!wifi_sta_enabled) {
		wifi_netif_init(&wifi_sta, 0, 's', 't', wifi_mac_addr);
		wifi_sta_enabled = true;
		LOG_INF("STA enable");
	}

	return &wifi_sta;
}

int wifi_mgmr_sta_disable(wifi_interface_t *interface)
{
	ARG_UNUSED(interface);
	wifi_sta_enabled = false;
	return 0;
}

int wifi_mgmr_sta_mac_set(uint8_t mac[6])
{
	if (mac == NULL) {
		return -1;
	}

	memcpy(wifi_mac_addr, mac, sizeof(wifi_mac_addr));
	memcpy(wifi_sta.mac, mac, sizeof(wifi_sta.mac));
	memcpy(wifi_ap.mac, mac, sizeof(wifi_ap.mac));
	memcpy(wifi_sta.netif.hwaddr, mac, sizeof(wifi_sta.netif.hwaddr));
	memcpy(wifi_ap.netif.hwaddr, mac, sizeof(wifi_ap.netif.hwaddr));
	return 0;
}

int wifi_mgmr_sta_mac_get(uint8_t mac[6])
{
	if (mac == NULL) {
		return -1;
	}

	memcpy(mac, wifi_sta.mac, sizeof(wifi_sta.mac));
	return 0;
}

int bl_cfg80211_connect(struct bl_hw *bl_hw, struct cfg80211_connect_params *sme);

void bl_main_set_mac_addr(const uint8_t *mac)
{
	if (mac == NULL) {
		return;
	}

	wifi_mgmr_sta_mac_set((uint8_t *)mac);
	wifi_netifs_refresh();
	LOG_INF("host MAC set to %02x:%02x:%02x:%02x:%02x:%02x", wifi_mac_addr[0], wifi_mac_addr[1],
		wifi_mac_addr[2], wifi_mac_addr[3], wifi_mac_addr[4], wifi_mac_addr[5]);
}

static void bl_set_vers(struct mm_version_cfm *version_cfm_ptr)
{
	uint32_t vers = version_cfm_ptr->version_lmac;

	ARG_UNUSED(vers);

	LOG_DBG("[version] lmac %u.%u.%u.%u", (unsigned int)((vers >> 24) & 0xFF),
		(unsigned int)((vers >> 16) & 0xFF), (unsigned int)((vers >> 8) & 0xFF),
		(unsigned int)((vers >> 0) & 0xFF));
	LOG_DBG("[version] version_machw_1 %08X", (unsigned int)version_cfm_ptr->version_machw_1);
	LOG_DBG("[version] version_machw_2 %08X", (unsigned int)version_cfm_ptr->version_machw_2);
	LOG_DBG("[version] version_phy_1 %08X", (unsigned int)version_cfm_ptr->version_phy_1);
	LOG_DBG("[version] version_phy_2 %08X", (unsigned int)version_cfm_ptr->version_phy_2);
	LOG_DBG("[version] features %08X", (unsigned int)version_cfm_ptr->features);
}

int bl_open(struct bl_hw *bl_hw)
{
	/* Interface open — no-op for now */
	return 0;
}

int bl_main_connect(const uint8_t *ssid, int ssid_len, const uint8_t *psk, int psk_len,
		    const uint8_t *pmk, int pmk_len, const uint8_t *mac, const uint8_t band,
		    const uint16_t freq, const uint32_t flags)
{
	struct cfg80211_connect_params sme;

	memset(&sme, 0, sizeof(struct cfg80211_connect_params));
	sme.crypto.n_ciphers_pairwise = 0;
	sme.ssid_len = ssid_len;
	sme.ssid = ssid;
	sme.auth_type = NL80211_AUTHTYPE_AUTOMATIC;
	sme.key = psk;
	sme.key_len = psk_len;
	sme.pmk = pmk;
	sme.pmk_len = pmk_len;
	sme.flags = flags;

	if (mac) {
		sme.bssid = mac;
	}

	if (freq > 0) {
		sme.channel.center_freq = freq;
		sme.channel.band = band;
		sme.channel.flags = 0;
	}

	return bl_cfg80211_connect(&wifi_hw, &sme);
}

int bl_main_disconnect(void)
{
	bl_send_sm_disconnect_req(&wifi_hw);
	return 0;
}

int bl_main_powersaving(int mode)
{
	return bl_send_mm_powersaving_req(&wifi_hw, mode);
}

int bl_main_denoise(int mode)
{
	return bl_send_mm_denoise_req(&wifi_hw, mode);
}

int bl_main_monitor(void)
{
	struct mm_monitor_cfm cfm;

	memset(&cfm, 0, sizeof(cfm));
	bl_send_monitor_enable(&wifi_hw, &cfm);
	return 0;
}

int bl_main_monitor_disable(void)
{
	struct mm_monitor_cfm cfm;

	memset(&cfm, 0, sizeof(cfm));
	bl_send_monitor_disable(&wifi_hw, &cfm);
	return 0;
}

int bl_main_phy_up(void)
{
	int error;

	error = bl_send_start(&wifi_hw);
	if (error) {
		return -1;
	}
	return 0;
}

int bl_main_channel_set(int channel)
{
	bl_send_channel_set_req(&wifi_hw, channel);
	return 0;
}

int bl_main_monitor_channel_set(int channel, int use_40MHZ)
{
	struct mm_monitor_channel_cfm cfm;

	bl_send_monitor_channel_set(&wifi_hw, &cfm, channel, use_40MHZ);
	return 0;
}

int bl_main_beacon_interval_set(uint16_t beacon_int)
{
	struct mm_set_beacon_int_cfm cfm;

	bl_send_beacon_interval_set(&wifi_hw, &cfm, beacon_int);
	return 0;
}

int bl_main_if_remove(uint8_t vif_index)
{
	LOG_INF("MM_REMOVE_IF_REQ Sending with vif_index %u...", vif_index);
	bl_send_remove_if(&wifi_hw, vif_index);
	LOG_INF("MM_REMOVE_IF_REQ Done");
	return 0;
}

int bl_main_raw_send(uint8_t *pkt, int len)
{
	return bl_send_scanu_raw_send(&wifi_hw, pkt, len);
}

int bl_main_rate_config(uint8_t sta_idx, uint16_t fixed_rate_cfg)
{
	return bl_send_me_rate_config_req(&wifi_hw, sta_idx, fixed_rate_cfg);
}

int bl_main_set_country_code(char *country_code)
{
	bl_msg_update_channel_cfg((const char *)country_code);
	bl_send_me_chan_config_req(&wifi_hw);
	return 0;
}

int bl_main_get_channel_nums(void)
{
	return bl_msg_get_channel_nums();
}

int bl_main_if_add(int is_sta, struct netif *netif, uint8_t *vif_index)
{
	struct mm_add_if_cfm add_if_cfm;
	int error = 0;
	int vif_id;
	struct netif *target_netif;

	target_netif = netif;
	if (target_netif == NULL) {
		target_netif = is_sta ? wifi_mgmr_sta_netif_get() : wifi_mgmr_ap_netif_get();
	}

	LOG_INF("MM_ADD_IF_REQ Sending: %s", is_sta ? "STA" : "AP");
	error = bl_send_add_if(&wifi_hw, target_netif->hwaddr,
			       is_sta ? NL80211_IFTYPE_STATION : NL80211_IFTYPE_AP, false,
			       &add_if_cfm);
	LOG_INF("MM_ADD_IF_REQ Done");
	if (error) {
		return error;
	}

	if (add_if_cfm.status != 0) {
		return -EIO;
	}

	vif_id = is_sta ? BL_VIF_STA : BL_VIF_AP;
	wifi_hw.vif_table[vif_id].vif_idx = add_if_cfm.inst_nbr;
	wifi_hw.vif_table[vif_id].dev = target_netif;
	wifi_hw.vif_table[vif_id].up = 1;
	wifi_hw.vif_table[vif_id].links_num = 0;
	if (is_sta) {
		wifi_sta.vif_index = add_if_cfm.inst_nbr;
		wifi_hw.vif_index_sta = add_if_cfm.inst_nbr;
	} else {
		wifi_ap.vif_index = add_if_cfm.inst_nbr;
		wifi_hw.vif_index_ap = add_if_cfm.inst_nbr;
	}
	*vif_index = vif_id;

	LOG_INF("vif_index from LMAC is %d, vif_id: %d", add_if_cfm.inst_nbr, vif_id);

	return error;
}

int bl_main_apm_start(char *ssid, char *password, int channel, uint8_t vif_index,
		      uint8_t hidden_ssid, uint16_t bcn_int)
{
	int error;
	struct apm_start_cfm start_ap_cfm;

	memset(&start_ap_cfm, 0, sizeof(start_ap_cfm));
	error = bl_send_apm_start_req(&wifi_hw, &start_ap_cfm, ssid, password, channel, vif_index,
				      hidden_ssid, bcn_int);
	wifi_hw.ap_bcmc_idx = start_ap_cfm.bcmc_idx;
	return error;
}

int bl_main_apm_stop(uint8_t vif_index)
{
	return bl_send_apm_stop_req(&wifi_hw, vif_index);
}

int bl_main_apm_sta_cnt_get(uint8_t *sta_cnt)
{
	*sta_cnt = (uint8_t)ARRAY_SIZE(wifi_hw.sta_table);
	return 0;
}

int bl_main_apm_sta_info_get(struct wifi_apm_sta_info *apm_sta_info, uint8_t idx)
{
	struct bl_sta *sta;

	sta = &(wifi_hw.sta_table[idx]);
	if (sta->is_used == 0) {
		return 0;
	}
	apm_sta_info->sta_idx = sta->sta_idx;
	apm_sta_info->is_used = sta->is_used;
	apm_sta_info->rssi = sta->rssi;
	apm_sta_info->tsflo = sta->tsflo;
	apm_sta_info->tsfhi = sta->tsfhi;
	apm_sta_info->data_rate = sta->data_rate;
	memcpy(apm_sta_info->sta_mac, sta->sta_addr.array, 6);
	return 0;
}

int bl_main_apm_sta_delete(uint8_t sta_idx)
{
	struct bl_sta *sta;
	struct apm_sta_del_cfm sta_del_cfm;

	if (sta_idx >= ARRAY_SIZE(wifi_hw.sta_table)) {
		return -1;
	}
	sta = &wifi_hw.sta_table[sta_idx];

	memset(&sta_del_cfm, 0, sizeof(struct apm_sta_del_cfm));
	bl_send_apm_sta_del_req(&wifi_hw, &sta_del_cfm, sta_idx, sta->vif_idx);
	if (sta_del_cfm.status != 0) {
		return -1;
	}

	memset(sta, 0, sizeof(struct bl_sta));
	return 0;
}

int bl_main_cfg_task_req(uint32_t ops, uint32_t task, uint32_t element, uint32_t type, void *arg1,
			 void *arg2)
{
	return bl_send_cfg_task_req(&wifi_hw, ops, task, element, type, arg1, arg2);
}

int bl_main_scan(struct netif *netif, uint16_t *fixed_channels, uint16_t channel_num,
		 struct mac_addr *bssid, struct mac_ssid *ssid, uint8_t scan_mode,
		 uint32_t duration_scan)
{
	struct bl_send_scanu_para scanu_para;
	struct netif *target_netif;

	target_netif = netif;
	if (target_netif == NULL) {
		(void)wifi_mgmr_sta_enable();
		target_netif = wifi_mgmr_sta_netif_get();
	}

	scanu_para.channels = fixed_channels;
	scanu_para.channel_num = channel_num;
	scanu_para.bssid = bssid;
	scanu_para.ssid = ssid;
	scanu_para.mac = target_netif->hwaddr;
	scanu_para.scan_mode = scan_mode;
	scanu_para.duration_scan = duration_scan;

	if (channel_num == 0U) {
		scanu_para.channels = NULL;
		scanu_para.channel_num = 0;
		return bl_send_scanu_req(&wifi_hw, &scanu_para);
	} else {
		if (bl_get_fixed_channels_is_valid(fixed_channels, channel_num)) {
			return bl_send_scanu_req(&wifi_hw, &scanu_para);
		}
	}
	return -EINVAL;
}

int bl_main_connect_abort(uint8_t *status)
{
	struct sm_connect_abort_cfm connect_abort_cfm = {};
	bl_send_sm_connect_abort_req(&wifi_hw, &connect_abort_cfm);
	*status = connect_abort_cfm.status;
	return 0;
}

/*
 * bl_wifi_enable_irq — enable WiFi hardware IRQs.
 *
 * In the SDK, this registers mac_irq and bl_irq_handler with the
 * interrupt controller. In Zephyr, we use irq_connect_dynamic().
 *
 * mac_irq and bl_irq_handler are symbols from libwifi.a.
 */
extern void mac_irq(void);
extern void bl_irq_handler(void);

volatile uint32_t wifi_mac_isr_cnt;
volatile uint32_t wifi_ipc_isr_cnt;

static void wifi_mac_isr(const void *arg)
{
	ARG_UNUSED(arg);
	wifi_mac_isr_cnt++;
	mac_irq();
}

static void wifi_ipc_isr(const void *arg)
{
	ARG_UNUSED(arg);
	wifi_ipc_isr_cnt++;
	bl_irq_handler();
}

int bl_wifi_enable_irq(void)
{
	irq_connect_dynamic(WIFI_IRQn, 7, wifi_mac_isr, NULL, 0);
	irq_connect_dynamic(WIFI_IPC_IRQn, 7, wifi_ipc_isr, NULL, 0);
	irq_enable(WIFI_IRQn);
	irq_enable(WIFI_IPC_IRQn);
	LOG_DBG("WiFi IRQs enabled (MAC=%d, IPC=%d)", WIFI_IRQn, WIFI_IPC_IRQn);
	return 0;
}

static int cfg80211_init(struct bl_hw *bl_hw)
{
	int ret = 0;
	struct mm_version_cfm version_cfm = {};

	INIT_LIST_HEAD(&bl_hw->vifs);

	bl_hw->mod_params = &bl_mod_params;

	LOG_INF("calling bl_platform_on");
	ret = bl_platform_on(bl_hw);
	if (ret) {
		LOG_INF("bl_platform_on FAILED");
		goto err_out;
	}
	LOG_INF("bl_platform_on OK");

	/* Flush D-cache so IPC shared structures in PSRAM are visible to hardware */
	sys_cache_data_flush_all();

	LOG_INF("enabling IPC IRQs");
	ipc_host_enable_irq(bl_hw->ipc_env, IPC_IRQ_E2A_ALL);
	bl_wifi_enable_irq();
	LOG_INF("IRQs enabled");

	/* Reset FW */
	LOG_INF("calling bl_send_reset");
	ret = bl_send_reset(bl_hw);
	LOG_INF("bl_send_reset returned %d", ret);
	if (ret) {
		goto err_out;
	}
	bl_os_msleep(5);
	LOG_INF("calling bl_send_version_req");
	ret = bl_send_version_req(bl_hw, &version_cfm);
	LOG_INF("bl_send_version_req returned %d", ret);
	if (ret) {
		goto err_out;
	}
	bl_set_vers(&version_cfm);
	LOG_INF("calling bl_handle_dynparams");
	ret = bl_handle_dynparams(bl_hw);
	LOG_INF("bl_handle_dynparams returned %d", ret);
	if (ret) {
		goto err_out;
	}

	/* Set parameters to firmware (must come BEFORE START per SDK) */
	LOG_INF("calling bl_send_me_config_req");
	bl_send_me_config_req(bl_hw);

	/* Set channel parameters to firmware */
	LOG_INF("calling bl_send_me_chan_config_req");
	bl_send_me_chan_config_req(bl_hw);

	/* Start PHY/MAC — must come AFTER ME_CONFIG/ME_CHAN_CONFIG so the
	 * scan engine knows which channels are available.
	 */
	LOG_INF("calling bl_send_start");
	ret = bl_send_start(bl_hw);
	LOG_INF("bl_send_start returned %d", ret);
	if (ret) {
		goto err_out;
	}

	/* Flush D-cache so WiFi MAC DMA descriptors written by the firmware
	 * are visible to the MAC DMA engine.
	 */
	sys_cache_data_flush_all();

err_out:
	LOG_INF("cfg80211_init done ret=%d", ret);
	return ret;
}

int bl_cfg80211_connect(struct bl_hw *bl_hw, struct cfg80211_connect_params *sme)
{
	struct sm_connect_cfm sm_connect_cfm;
	int error;

	error = bl_send_sm_connect_req(bl_hw, sme, &sm_connect_cfm);
	if (error) {
		return error;
	}

	switch (sm_connect_cfm.status) {
	case CO_OK:
		error = 0;
		break;
	case CO_BUSY:
		error = -EINPROGRESS;
		break;
	case CO_OP_IN_PROGRESS:
		error = -EALREADY;
		break;
	default:
		error = -EIO;
		break;
	}

	return error;
}

int bl_cfg80211_disconnect(struct bl_hw *bl_hw)
{
	return bl_send_sm_disconnect_req(bl_hw);
}

void bl_main_event_handle(int param, struct ke_tx_fc *tx_fc_field)
{
	/*
	 * Always run the IPC bottom-half so pending ACKs are processed.
	 *
	 * The original SDK only calls bl_irq_bottomhalf when param == 0
	 * because the IPC hardware interrupt (WIFI_IPC_IRQn) handles ACKs
	 * in the ISR. On BL808 same-core, the IPC interrupt does not
	 * reliably fire, so the bottom-half is the only ACK delivery path.
	 * Without this, bl_send_start hangs: the firmware generates a TX
	 * event (param != 0) after START, skipping the bottom-half and
	 * leaving the ACK unprocessed.
	 */
	bl_irq_bottomhalf(&wifi_hw);

	bl_tx_try_flush(param, tx_fc_field);
}

void bl_main_lowlevel_init(void)
{
	bl_irqs_init(&wifi_hw);
}

int bl_main_tx_still_free(void)
{
	return ipc_host_txdesc_left(wifi_hw.ipc_env, 0, 0);
}

/*
 * WiFi firmware task — runs wifi_main() from libwifi.a.
 * This is the LMAC firmware that processes IPC messages from
 * the host driver. Must be running before bl_send_reset().
 */
extern void wifi_main(void *param);

#define WIFI_FW_STACK_SIZE    32768
#define WIFI_INIT_STACK_SIZE  8192
#define WIFI_IFADD_STACK_SIZE 4096

/*
 * Place WiFi thread stacks in OCRAM to free WRAM for DMA buffers.
 * Without CONFIG_USERSPACE, K_THREAD_STACK_DEFINE is just an aligned array
 * in __kstackmem; we replicate that layout with an OCRAM section attribute.
 * K_THREAD_STACK_SIZEOF(sym) = sizeof(sym) - K_KERNEL_STACK_RESERVED, so
 * it works correctly with these manually-sectioned arrays.
 */
#define OCRAM_STACK_DEFINE(sym, size)                                                              \
	static struct z_thread_stack_element __attribute__((section("OCRAM")))                     \
	__aligned(Z_KERNEL_STACK_OBJ_ALIGN) sym[K_KERNEL_STACK_LEN(size)]

OCRAM_STACK_DEFINE(wifi_fw_stack, WIFI_FW_STACK_SIZE);
static struct k_thread wifi_fw_thread;

static bool cfg80211_initialized;
static bool cfg80211_init_started;
static void wifi_ifadd_start_async(void);

OCRAM_STACK_DEFINE(wifi_init_stack, WIFI_INIT_STACK_SIZE);
static struct k_thread wifi_init_thread;

OCRAM_STACK_DEFINE(wifi_ifadd_stack, WIFI_IFADD_STACK_SIZE);
static struct k_thread wifi_ifadd_thread;
static bool wifi_ifadd_started;

static void cfg80211_init_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/* Firmware thread needs time to finish RF calibration before
	 * it can process host commands in the event loop.
	 */
	k_msleep(1000);
	int ret = cfg80211_init(&wifi_hw);

	if (ret == 0) {
		cfg80211_initialized = true;
		wifi_ifadd_start_async();
	}
}

static void cfg80211_init_start_async(void)
{
	if (cfg80211_initialized || cfg80211_init_started) {
		return;
	}

	cfg80211_init_started = true;
	k_thread_create(&wifi_init_thread, wifi_init_stack, K_THREAD_STACK_SIZEOF(wifi_init_stack),
			cfg80211_init_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
	k_thread_name_set(&wifi_init_thread, "wifi_init");
}

static void wifi_ifadd_thread_fn(void *p1, void *p2, void *p3)
{
	uint8_t vif_index = 0xff;
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("if_add thread: starting STA add");
	ret = bl_main_if_add(1, NULL, &vif_index);
	LOG_INF("if_add thread: bl_main_if_add returned %d (vif=%u)", ret, vif_index);

	/* SDK calls bl_send_start a second time via bl_main_phy_up() when
	 * transitioning from IfaceDown→Idle.  This activates the radio/PHY
	 * so that scanning actually works.
	 */
	if (ret == 0) {
		LOG_INF("if_add thread: phy_up (second START)");
		ret = bl_send_start(&wifi_hw);
		LOG_INF("if_add thread: phy_up returned %d", ret);
	}

	wifi_ifadd_started = false;
}

static void wifi_ifadd_start_async(void)
{
	if (wifi_ifadd_started) {
		return;
	}

	wifi_ifadd_started = true;
	k_thread_create(&wifi_ifadd_thread, wifi_ifadd_stack,
			K_THREAD_STACK_SIZEOF(wifi_ifadd_stack), wifi_ifadd_thread_fn, NULL, NULL,
			NULL, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
	k_thread_name_set(&wifi_ifadd_thread, "wifi_ifadd");
}

int bl_main_rtthread_start(struct bl_hw **bl_hw)
{
	printk_hook_fn_t printk_hook = __printk_get_hook();

	wifi_hw.vif_index_sta = -1;
	wifi_hw.vif_index_ap = -1;
	wifi_hw.sta_idx = -1;
	wifi_hw.ap_bcmc_idx = -1;
	cfg80211_initialized = false;
	cfg80211_init_started = false;
	wifi_ifadd_started = false;
	wifi_sta_enabled = false;
	wifi_ap_enabled = false;
	wifi_netifs_refresh();

	LOG_INF("step 1: lowlevel_init");
	bl_main_lowlevel_init();

	if (printk_hook != NULL) {
		__stdout_hook_install(printk_hook);
		LOG_INF("stdout hook installed from printk hook %p", printk_hook);
	}

	LOG_INF("step 2: starting wifi_main thread");
	k_thread_create(&wifi_fw_thread, wifi_fw_stack, K_THREAD_STACK_SIZEOF(wifi_fw_stack),
			(k_thread_entry_t)wifi_main, NULL, NULL, NULL, K_PRIO_COOP(7), 0,
			K_NO_WAIT);
	k_thread_name_set(&wifi_fw_thread, "wifi_fw");

	LOG_INF("cfg80211_init AUTO (background thread)");
	cfg80211_init_start_async();

	*bl_hw = &wifi_hw;
	return 0;
}
