/*
 * Copyright (C) Bouffalo Lab 2016-2018
 * SPDX-License-Identifier: Apache-2.0
 *
 * LMAC message TX — ported from M1s SDK.
 */

#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <bl_os_private.h>
#include <utils_tlv_bl.h>
#include <bl60x_fw_api.h>
#include <zephyr/logging/log.h>

#include "bl_msg_tx.h"
#include "bl_utils.h"

LOG_MODULE_REGISTER(bflb_msg_tx, LOG_LEVEL_ERR);

static const struct mac_addr mac_addr_bcst = {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
static const struct mac_addr mac_addr_zero = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

static const struct ieee80211_channel bl_channels_24_General[] = {
	{.band = NL80211_BAND_2GHZ, .center_freq = 2412, .hw_value = 1, .max_power = 20},
	{.band = NL80211_BAND_2GHZ, .center_freq = 2417, .hw_value = 2, .max_power = 20},
	{.band = NL80211_BAND_2GHZ, .center_freq = 2422, .hw_value = 3, .max_power = 20},
	{.band = NL80211_BAND_2GHZ, .center_freq = 2427, .hw_value = 4, .max_power = 20},
	{.band = NL80211_BAND_2GHZ, .center_freq = 2432, .hw_value = 5, .max_power = 20},
	{.band = NL80211_BAND_2GHZ, .center_freq = 2437, .hw_value = 6, .max_power = 20},
	{.band = NL80211_BAND_2GHZ, .center_freq = 2442, .hw_value = 7, .max_power = 20},
	{.band = NL80211_BAND_2GHZ, .center_freq = 2447, .hw_value = 8, .max_power = 20},
	{.band = NL80211_BAND_2GHZ, .center_freq = 2452, .hw_value = 9, .max_power = 20},
	{.band = NL80211_BAND_2GHZ, .center_freq = 2457, .hw_value = 10, .max_power = 20},
	{.band = NL80211_BAND_2GHZ, .center_freq = 2462, .hw_value = 11, .max_power = 20},
	{.band = NL80211_BAND_2GHZ, .center_freq = 2467, .hw_value = 12, .max_power = 20},
	{.band = NL80211_BAND_2GHZ, .center_freq = 2472, .hw_value = 13, .max_power = 20},
	{.band = NL80211_BAND_2GHZ, .center_freq = 2484, .hw_value = 14, .max_power = 20},
};

static const struct ieee80211_dot_d country_list[] = {
	{.code = "CN", .channel_num = 13, .channels = bl_channels_24_General},
	{.code = "JP", .channel_num = 14, .channels = bl_channels_24_General},
	{.code = "US", .channel_num = 11, .channels = bl_channels_24_General},
	{.code = "EU", .channel_num = 13, .channels = bl_channels_24_General},
};

static int channel_num_default;
static const struct ieee80211_channel *channels_default;
static const struct ieee80211_dot_d *country_default;

static void bl_msg_ensure_channel_cfg(void)
{
	if (channel_num_default > 0 && channels_default != NULL && country_default != NULL) {
		return;
	}

	/* Default to US channels until userspace overrides the regulatory domain. */
	bl_msg_update_channel_cfg("US");
}

static int cfg80211_get_channel_list(const char *code, int *channel_num,
				     const struct ieee80211_channel **channels,
				     const struct ieee80211_dot_d **out_country)
{
	int i;

	for (i = 0; i < (int)ARRAY_SIZE(country_list); i++) {
		if (strcmp(country_list[i].code, code) == 0) {
			if (channel_num) {
				*channel_num = country_list[i].channel_num;
			}
			if (channels) {
				*channels = country_list[i].channels;
			}
			if (out_country) {
				*out_country = &country_list[i];
			}
			return 0;
		}
	}
	return -1;
}

void bl_msg_update_channel_cfg(const char *code)
{
	if (cfg80211_get_channel_list(code, &channel_num_default, &channels_default,
				      &country_default)) {
		channel_num_default = ARRAY_SIZE(bl_channels_24_General);
		channels_default = bl_channels_24_General;
		country_default = &country_list[0];
		LOG_INF("%s NOT found, using General instead, num of channel %d", code,
			channel_num_default);
	} else {
		LOG_INF("country code %s used, num of channel %d", code, channel_num_default);
	}
}

int bl_msg_get_channel_nums(void)
{
	bl_msg_ensure_channel_cfg();
	return channel_num_default;
}

int bl_get_fixed_channels_is_valid(uint16_t *channels, uint16_t channel_num)
{
	int i;
	int channel;

	if (channel_num == 0U) {
		return 0;
	}

	for (i = 0; i < channel_num; i++) {
		channel = channels[i];
		if ((channel == 0) || (channel > bl_msg_get_channel_nums())) {
			return 0;
		}
	}
	return 1;
}

uint16_t phy_channel_to_freq(uint8_t band, int channel)
{
	uint16_t freq = 0xFFFF;

	if (band == PHY_BAND_2G4) {
		if ((channel < 1) || (channel > 14)) {
			return freq;
		}
		if (channel == 14) {
			freq = 2484;
		} else {
			freq = 2407 + channel * 5;
		}
	} else if (band == PHY_BAND_5G) {
		if ((channel < 1) || (channel > 165)) {
			return freq;
		}
		freq = 5000 + channel * 5;
	}

	return freq;
}

uint8_t phy_freq_to_channel(uint8_t band, uint16_t freq)
{
	uint8_t channel = 0;

	if (band == PHY_BAND_2G4) {
		if ((freq < 2412) || (freq > 2484)) {
			return 0;
		}
		if (freq == 2484) {
			channel = 14;
		} else {
			channel = (freq - 2407) / 5;
		}
	}

	return channel;
}

static inline void *bl_msg_zalloc(ke_msg_id_t const id, ke_task_id_t const dest_id,
				  ke_task_id_t const src_id, uint16_t const param_len)
{
	struct lmac_msg *msg;

	msg = (struct lmac_msg *)bl_os_malloc(sizeof(struct lmac_msg) + param_len);
	if (msg == NULL) {
		LOG_DBG("%s: msg allocation failed\n", __func__);
		return NULL;
	}
	memset(msg, 0, sizeof(struct lmac_msg) + param_len);

	msg->id = id;
	msg->dest_id = dest_id;
	msg->src_id = src_id;
	msg->param_len = param_len;

	return msg->param;
}

static inline bool is_non_blocking_msg(int id)
{
	return ((id == MM_TIM_UPDATE_REQ) || (id == ME_RC_SET_RATE_REQ) ||
		(id == MM_BFMER_ENABLE_REQ) || (id == ME_TRAFFIC_IND_REQ));
}

static int bl_send_msg(struct bl_hw *bl_hw, const void *msg_params, int reqcfm, ke_msg_id_t reqid,
		       void *cfm)
{
	struct lmac_msg *msg;
	struct bl_cmd *cmd;
	bool nonblock;
	int ret;

	msg = container_of((void *)msg_params, struct lmac_msg, param);

	if (!bl_hw->ipc_env) {
		LOG_DBG("%s: bypassing (restart must have failed)", __func__);
		bl_os_free(msg);
		return -EBUSY;
	}

	nonblock = is_non_blocking_msg(msg->id);

	cmd = bl_os_malloc(sizeof(struct bl_cmd));
	if (cmd == NULL) {
		bl_os_free(msg);
		return -ENOMEM;
	}
	memset(cmd, 0, sizeof(struct bl_cmd));
	cmd->result = EINTR;
	cmd->id = msg->id;
	cmd->reqid = reqid;
	cmd->a2e_msg = msg;
	cmd->e2a_msg = cfm;
	if (nonblock) {
		cmd->flags = RWNX_CMD_FLAG_NONBLOCK;
	}
	if (reqcfm) {
		cmd->flags |= RWNX_CMD_FLAG_REQ_CFM;
	}
	ret = bl_hw->cmd_mgr.queue(&bl_hw->cmd_mgr, cmd);

	if (!nonblock) {
		bl_os_free(cmd);
	} else {
		ret = cmd->result;
	}

	return ret;
}

int bl_send_reset(struct bl_hw *bl_hw)
{
	void *void_param;

	void_param = bl_msg_zalloc(MM_RESET_REQ, TASK_MM, DRV_TASK_ID, 0);
	if (!void_param) {
		return -ENOMEM;
	}

	return bl_send_msg(bl_hw, void_param, 1, MM_RESET_CFM, NULL);
}

int bl_send_monitor_enable(struct bl_hw *bl_hw, struct mm_monitor_cfm *cfm)
{
	struct mm_monitor_req *req;

	req = bl_msg_zalloc(MM_MONITOR_REQ, TASK_MM, DRV_TASK_ID, sizeof(struct mm_monitor_req));
	if (!req) {
		return -ENOMEM;
	}

	req->enable = 1;
	return bl_send_msg(bl_hw, req, 1, MM_MONITOR_CFM, cfm);
}

int bl_send_monitor_disable(struct bl_hw *bl_hw, struct mm_monitor_cfm *cfm)
{
	struct mm_monitor_req *req;

	req = bl_msg_zalloc(MM_MONITOR_REQ, TASK_MM, DRV_TASK_ID, sizeof(struct mm_monitor_req));
	if (!req) {
		return -ENOMEM;
	}

	req->enable = 0;
	return bl_send_msg(bl_hw, req, 1, MM_MONITOR_CFM, cfm);
}

int bl_send_beacon_interval_set(struct bl_hw *bl_hw, struct mm_set_beacon_int_cfm *cfm,
				uint16_t beacon_int)
{
	struct mm_set_beacon_int_req *req;

	req = bl_msg_zalloc(MM_SET_BEACON_INT_REQ, TASK_MM, DRV_TASK_ID,
			    sizeof(struct mm_set_beacon_int_req));
	if (!req) {
		return -ENOMEM;
	}

	req->beacon_int = beacon_int;
	return bl_send_msg(bl_hw, req, 1, MM_SET_BEACON_INT_CFM, cfm);
}

int bl_send_monitor_channel_set(struct bl_hw *bl_hw, struct mm_monitor_channel_cfm *cfm,
				int channel, int use_40Mhz)
{
	struct mm_monitor_channel_req *req;

	req = bl_msg_zalloc(MM_MONITOR_CHANNEL_REQ, TASK_MM, DRV_TASK_ID,
			    sizeof(struct mm_monitor_channel_req));
	if (!req) {
		return -ENOMEM;
	}

	req->freq = phy_channel_to_freq(PHY_BAND_2G4, channel);
	return bl_send_msg(bl_hw, req, 1, MM_MONITOR_CHANNEL_CFM, cfm);
}

int bl_send_version_req(struct bl_hw *bl_hw, struct mm_version_cfm *cfm)
{
	void *void_param;

	void_param = bl_msg_zalloc(MM_VERSION_REQ, TASK_MM, DRV_TASK_ID, 0);
	if (!void_param) {
		return -ENOMEM;
	}

	return bl_send_msg(bl_hw, void_param, 1, MM_VERSION_CFM, cfm);
}

int bl_send_me_config_req(struct bl_hw *bl_hw)
{
	struct me_config_req *req;
	uint8_t *ht_mcs = (uint8_t *)&(bl_hw->ht_cap.mcs);
	int i, ret;

	req = bl_msg_zalloc(ME_CONFIG_REQ, TASK_ME, DRV_TASK_ID, sizeof(struct me_config_req));
	if (!req) {
		return -ENOMEM;
	}

	req->ht_supp = 1;
	req->vht_supp = 0;
	req->ht_cap.ht_capa_info = cpu_to_le16(bl_hw->ht_cap.cap);

	req->ht_cap.a_mpdu_param = 0x3;

	for (i = 0; i < sizeof(bl_hw->ht_cap.mcs); i++) {
		req->ht_cap.mcs_rate[i] = ht_mcs[i];
	}
	req->ht_cap.ht_extended_capa = 0;
	req->ht_cap.tx_beamforming_capa = 0;
	req->ht_cap.asel_capa = 0;

	req->ps_on = bl_hw->mod_params->ps_on;
	req->tx_lft = bl_hw->mod_params->tx_lft;

	/* Firmware does not send ME_CONFIG_CFM — don't wait for it */
	ret = bl_send_msg(bl_hw, req, 0, ME_CONFIG_CFM, NULL);
	return ret;
}

static uint8_t passive_scan_flag(uint32_t flags)
{
	if (flags & (IEEE80211_CHAN_NO_IR | IEEE80211_CHAN_RADAR)) {
		return SCAN_PASSIVE_BIT;
	}
	return 0;
}

int bl_send_me_chan_config_req(struct bl_hw *bl_hw)
{
	struct me_chan_config_req *req;
	int i;

	bl_msg_ensure_channel_cfg();

	req = bl_msg_zalloc(ME_CHAN_CONFIG_REQ, TASK_ME, DRV_TASK_ID,
			    sizeof(struct me_chan_config_req));
	if (!req) {
		return -ENOMEM;
	}

	req->chan2G4_cnt = 0;
	for (i = 0; i < channel_num_default; i++) {
		req->chan2G4[req->chan2G4_cnt].flags = 0;
		if (channels_default[i].flags & IEEE80211_CHAN_DISABLED) {
			req->chan2G4[req->chan2G4_cnt].flags |= SCAN_DISABLED_BIT;
		}
		req->chan2G4[req->chan2G4_cnt].flags |=
			passive_scan_flag(channels_default[i].flags);
		req->chan2G4[req->chan2G4_cnt].band = NL80211_BAND_2GHZ;
		req->chan2G4[req->chan2G4_cnt].freq = channels_default[i].center_freq;
		req->chan2G4[req->chan2G4_cnt].tx_power = channels_default[i].max_power;
		req->chan2G4_cnt++;
		if (req->chan2G4_cnt == SCAN_CHANNEL_2G4) {
			break;
		}
	}

	/* Firmware does not send ME_CHAN_CONFIG_CFM — don't wait for it */
	return bl_send_msg(bl_hw, req, 0, ME_CHAN_CONFIG_CFM, NULL);
}

int bl_send_me_rate_config_req(struct bl_hw *bl_hw, uint8_t sta_idx, uint16_t fixed_rate_cfg)
{
	struct me_rc_set_rate_req *req;

	req = bl_msg_zalloc(ME_RC_SET_RATE_REQ, TASK_ME, DRV_TASK_ID,
			    sizeof(struct me_rc_set_rate_req));
	if (!req) {
		return -ENOMEM;
	}

	req->sta_idx = sta_idx;
	req->fixed_rate_cfg = fixed_rate_cfg;
	req->power_table_req = 1;

	return bl_send_msg(bl_hw, req, 0, 0, NULL);
}

int bl_send_start(struct bl_hw *bl_hw)
{
	struct mm_start_req *start_req_param;

	start_req_param =
		bl_msg_zalloc(MM_START_REQ, TASK_MM, DRV_TASK_ID, sizeof(struct mm_start_req));
	if (!start_req_param) {
		return -ENOMEM;
	}

	memset(&start_req_param->phy_cfg, 0, sizeof(start_req_param->phy_cfg));
	start_req_param->phy_cfg.parameters[0] = 0x1;
	start_req_param->uapsd_timeout = (u32_l)bl_hw->mod_params->uapsd_timeout;
	start_req_param->lp_clk_accuracy = (u16_l)bl_hw->mod_params->lp_clk_ppm;

	return bl_send_msg(bl_hw, start_req_param, 1, MM_START_CFM, NULL);
}

int bl_send_add_if(struct bl_hw *bl_hw, const unsigned char *mac, enum nl80211_iftype iftype,
		   bool p2p, struct mm_add_if_cfm *cfm)
{
	struct mm_add_if_req *add_if_req_param;

	add_if_req_param =
		bl_msg_zalloc(MM_ADD_IF_REQ, TASK_MM, DRV_TASK_ID, sizeof(struct mm_add_if_req));
	if (!add_if_req_param) {
		return -ENOMEM;
	}

	memcpy(&(add_if_req_param->addr.array[0]), mac, ETH_ALEN);
	switch (iftype) {
	case NL80211_IFTYPE_P2P_CLIENT:
		add_if_req_param->p2p = true;
		__attribute__((fallthrough));
	case NL80211_IFTYPE_STATION:
		add_if_req_param->type = MM_STA;
		break;
	case NL80211_IFTYPE_ADHOC:
		add_if_req_param->type = MM_IBSS;
		break;
	case NL80211_IFTYPE_P2P_GO:
		add_if_req_param->p2p = true;
		__attribute__((fallthrough));
	case NL80211_IFTYPE_AP:
		add_if_req_param->type = MM_AP;
		break;
	case NL80211_IFTYPE_MESH_POINT:
		add_if_req_param->type = MM_MESH_POINT;
		break;
	case NL80211_IFTYPE_AP_VLAN:
		return -1;
	default:
		add_if_req_param->type = MM_STA;
		break;
	}

	return bl_send_msg(bl_hw, add_if_req_param, 1, MM_ADD_IF_CFM, cfm);
}

int bl_send_set_vif_state(struct bl_hw *bl_hw, uint8_t inst_nbr, uint16_t aid, bool active)
{
	struct mm_set_vif_state_req *req;

	req = bl_msg_zalloc(MM_SET_VIF_STATE_REQ, TASK_MM, DRV_TASK_ID,
			    sizeof(struct mm_set_vif_state_req));
	if (!req) {
		return -ENOMEM;
	}

	req->inst_nbr = inst_nbr;
	req->aid = aid;
	req->active = active ? 1 : 0;

	/* Firmware may not send MM_SET_VIF_STATE_CFM — don't wait for it */
	return bl_send_msg(bl_hw, req, 0, MM_SET_VIF_STATE_CFM, NULL);
}

int bl_send_remove_if(struct bl_hw *bl_hw, uint8_t inst_nbr)
{
	struct mm_remove_if_req *remove_if_req_param;

	remove_if_req_param = bl_msg_zalloc(MM_REMOVE_IF_REQ, TASK_MM, DRV_TASK_ID,
					    sizeof(struct mm_remove_if_req));
	if (!remove_if_req_param) {
		return -ENOMEM;
	}

	remove_if_req_param->inst_nbr = inst_nbr;
	return bl_send_msg(bl_hw, remove_if_req_param, 1, MM_REMOVE_IF_CFM, NULL);
}

int bl_send_scanu_req(struct bl_hw *bl_hw, struct bl_send_scanu_para *scanu_para)
{
	struct scanu_start_req *req;
	int i, index;
	int vif_idx;
	uint8_t chan_flags = 0;
	const struct ieee80211_channel *chan;

	bl_msg_ensure_channel_cfg();

	req = bl_msg_zalloc(SCANU_START_REQ, TASK_SCANU, DRV_TASK_ID,
			    sizeof(struct scanu_start_req));
	if (!req) {
		return -ENOMEM;
	}

	vif_idx = bl_hw->vif_table[BL_VIF_STA].vif_idx;
	if (vif_idx < 0 || vif_idx > 0xff) {
		vif_idx = (bl_hw->vif_index_sta >= 0) ? bl_hw->vif_index_sta : 0;
	}
	req->vif_idx = (uint8_t)vif_idx;
	if (scanu_para->channel_num == 0U) {
		req->chan_cnt = channel_num_default;
	} else {
		req->chan_cnt = scanu_para->channel_num;
	}

	req->ssid_cnt = 1;
	if (scanu_para->ssid != NULL && scanu_para->ssid->length) {
		req->ssid[0].length = scanu_para->ssid->length;
		memcpy(req->ssid[0].array, scanu_para->ssid->array, req->ssid[0].length);
	} else {
		req->ssid[0].length = 0;
		if (req->ssid_cnt == 0 || scanu_para->scan_mode == SCAN_PASSIVE) {
			chan_flags |= SCAN_PASSIVE_BIT;
		}
	}
	if (scanu_para->bssid) {
		memcpy((uint8_t *)&(req->bssid), (uint8_t *)scanu_para->bssid, ETH_ALEN);
	} else {
		req->bssid = mac_addr_bcst;
	}
	memcpy(&(req->mac), scanu_para->mac, ETH_ALEN);
	req->no_cck = true;

	req->add_ie_len = 0;
	req->add_ies = 0;

	for (i = 0; i < req->chan_cnt; i++) {
		index = (channel_num_default == req->chan_cnt) ? i : (scanu_para->channels[i] - 1);
		chan = &(channels_default[index]);

		req->chan[i].band = chan->band;
		req->chan[i].freq = chan->center_freq;
		req->chan[i].flags = chan_flags | passive_scan_flag(chan->flags);
		req->chan[i].tx_power = chan->max_power;
	}

	req->duration_scan = scanu_para->duration_scan;

	return bl_send_msg(bl_hw, req, 0, 0, NULL);
}

int bl_send_scanu_raw_send(struct bl_hw *bl_hw, uint8_t *pkt, int len)
{
	struct scanu_raw_send_req *req;
	struct scanu_raw_send_cfm cfm;

	req = bl_msg_zalloc(SCANU_RAW_SEND_REQ, TASK_SCANU, DRV_TASK_ID,
			    sizeof(struct scanu_raw_send_req));
	if (!req) {
		return -ENOMEM;
	}

	req->pkt = pkt;
	req->len = len;

	return bl_send_msg(bl_hw, req, 1, SCANU_RAW_SEND_CFM, &cfm);
}

int bl_send_sm_connect_req(struct bl_hw *bl_hw, struct cfg80211_connect_params *sme,
			   struct sm_connect_cfm *cfm)
{
	struct sm_connect_req *req;
	int i;
	u32_l flags = sme->flags;

	req = bl_msg_zalloc(SM_CONNECT_REQ, TASK_SM, DRV_TASK_ID, sizeof(struct sm_connect_req));
	if (!req) {
		return -ENOMEM;
	}

	req->ctrl_port_ethertype = ETH_P_PAE;

	if (sme->bssid && !MAC_ADDR_CMP(sme->bssid, mac_addr_bcst.array) &&
	    !MAC_ADDR_CMP(sme->bssid, mac_addr_zero.array)) {
		for (i = 0; i < ETH_ALEN; i++) {
			req->bssid.array[i] = sme->bssid[i];
		}
	} else {
		req->bssid = mac_addr_bcst;
	}
	req->vif_idx = bl_hw->vif_table[BL_VIF_STA].vif_idx;
	if (sme->channel.center_freq) {
		req->chan.band = sme->channel.band;
		req->chan.freq = sme->channel.center_freq;
		req->chan.flags = passive_scan_flag(sme->channel.flags);
	} else {
		req->chan.freq = (u16_l)-1;
	}
	for (i = 0; i < sme->ssid_len; i++) {
		req->ssid.array[i] = sme->ssid[i];
	}
	req->ssid.length = sme->ssid_len;
	req->flags = flags;
	req->listen_interval = bl_mod_params.listen_itv;
	req->dont_wait_bcmc = !bl_mod_params.listen_bcmc;

	if (sme->auth_type == NL80211_AUTHTYPE_AUTOMATIC) {
		req->auth_type = NL80211_AUTHTYPE_OPEN_SYSTEM;
	} else {
		req->auth_type = sme->auth_type;
	}

	req->uapsd_queues = bl_mod_params.uapsd_queues;
	req->is_supplicant_enabled = 1;
	if (sme->key_len) {
		memcpy(req->phrase, sme->key, sme->key_len);
	}
	if (sme->pmk_len) {
		memcpy(req->phrase_pmk, sme->pmk, sme->pmk_len);
	}

	return bl_send_msg(bl_hw, req, 1, SM_CONNECT_CFM, cfm);
}

int bl_send_sm_disconnect_req(struct bl_hw *bl_hw)
{
	struct sm_disconnect_req *req;

	req = bl_msg_zalloc(SM_DISCONNECT_REQ, TASK_SM, DRV_TASK_ID,
			    sizeof(struct sm_disconnect_req));
	if (!req) {
		return -ENOMEM;
	}

	req->vif_idx = bl_hw->vif_table[BL_VIF_STA].vif_idx;
	return bl_send_msg(bl_hw, req, 1, SM_DISCONNECT_CFM, NULL);
}

int bl_send_sm_connect_abort_req(struct bl_hw *bl_hw, struct sm_connect_abort_cfm *cfm)
{
	struct sm_connect_abort_req *req;

	req = bl_msg_zalloc(SM_CONNECT_ABORT_REQ, TASK_SM, DRV_TASK_ID,
			    sizeof(struct sm_connect_abort_req));
	if (!req) {
		return -ENOMEM;
	}

	req->vif_idx = bl_hw->vif_table[BL_VIF_STA].vif_idx;
	return bl_send_msg(bl_hw, req, 1, SM_CONNECT_ABORT_CFM, cfm);
}

int bl_send_mm_powersaving_req(struct bl_hw *bl_hw, int mode)
{
	struct mm_set_ps_mode_req *req;

	req = bl_msg_zalloc(MM_SET_PS_MODE_REQ, TASK_MM, DRV_TASK_ID,
			    sizeof(struct mm_set_ps_mode_req));
	if (!req) {
		return -ENOMEM;
	}

	req->new_state = mode;
	return bl_send_msg(bl_hw, req, 1, MM_SET_PS_MODE_CFM, NULL);
}

int bl_send_mm_denoise_req(struct bl_hw *bl_hw, int mode)
{
	struct mm_set_denoise_req *req;

	req = bl_msg_zalloc(MM_DENOISE_REQ, TASK_MM, DRV_TASK_ID,
			    sizeof(struct mm_set_denoise_req));
	if (!req) {
		return -ENOMEM;
	}

	req->denoise_mode = mode;
	return bl_send_msg(bl_hw, req, 1, MM_SET_PS_MODE_CFM, NULL);
}

int bl_send_apm_start_req(struct bl_hw *bl_hw, struct apm_start_cfm *cfm, char *ssid,
			  char *password, int channel, uint8_t vif_index, uint8_t hidden_ssid,
			  uint16_t bcn_int)
{
	struct apm_start_req *req;
	uint8_t rate[] = {0x82, 0x84, 0x8b, 0x96, 0x12, 0x24, 0x48, 0x6c, 0x0c, 0x18, 0x30, 0x60};

	req = bl_msg_zalloc(APM_START_REQ, TASK_APM, DRV_TASK_ID, sizeof(struct apm_start_req));
	if (!req) {
		return -ENOMEM;
	}

	req->chan.band = NL80211_BAND_2GHZ;
	req->chan.freq = phy_channel_to_freq(req->chan.band, channel);
	req->chan.flags = 0;
	req->chan.tx_power = 0;

	req->center_freq1 = req->chan.freq;
	req->center_freq2 = 0;
	req->ch_width = PHY_CHNL_BW_20;
	req->hidden_ssid = hidden_ssid;
	req->bcn_addr = 0;
	req->bcn_len = 0;
	req->tim_oft = 0;
	req->bcn_int = bcn_int;
	req->flags = 0x08;
	req->ctrl_port_ethertype = 0x8e88; /* ETH_P_PAE in LE byte order */
	req->tim_len = 6;
	req->vif_idx = vif_index;

	if (strlen(password)) {
		req->ap_sec_type = 1;
	} else {
		req->ap_sec_type = 0;
	}
	req->apm_emb_enabled = 1;
	memcpy(req->ssid.array, ssid, strlen(ssid));
	memcpy(req->phrase, password, strlen(password));
	req->ssid.length = strlen(ssid);
	req->rate_set.length = 12;
	memcpy(req->rate_set.array, rate, req->rate_set.length);
	req->beacon_period = 0x1;
	req->qos_supported = 1;

	return bl_send_msg(bl_hw, req, 1, APM_START_CFM, cfm);
}

int bl_send_apm_stop_req(struct bl_hw *bl_hw, uint8_t vif_idx)
{
	struct apm_stop_req *req;

	req = bl_msg_zalloc(APM_STOP_REQ, TASK_APM, DRV_TASK_ID, sizeof(struct apm_stop_req));
	if (!req) {
		return -ENOMEM;
	}

	req->vif_idx = vif_idx;
	return bl_send_msg(bl_hw, req, 1, APM_STOP_CFM, NULL);
}

int bl_send_apm_sta_del_req(struct bl_hw *bl_hw, struct apm_sta_del_cfm *cfm, uint8_t sta_idx,
			    uint8_t vif_idx)
{
	struct apm_sta_del_req *req;

	req = bl_msg_zalloc(APM_STA_DEL_REQ, TASK_APM, DRV_TASK_ID, sizeof(struct apm_sta_del_req));
	if (!req) {
		return -ENOMEM;
	}

	req->vif_idx = vif_idx;
	req->sta_idx = sta_idx;
	return bl_send_msg(bl_hw, req, 1, APM_STA_DEL_CFM, cfm);
}

int bl_send_apm_conf_max_sta_req(struct bl_hw *bl_hw, uint8_t max_sta_supported)
{
	struct apm_conf_max_sta_req *req;

	req = bl_msg_zalloc(APM_CONF_MAX_STA_REQ, TASK_APM, DRV_TASK_ID,
			    sizeof(struct apm_conf_max_sta_req));
	if (!req) {
		return -ENOMEM;
	}

	req->max_sta_supported = max_sta_supported;
	return bl_send_msg(bl_hw, req, 1, APM_CONF_MAX_STA_CFM, NULL);
}

int bl_send_cfg_task_req(struct bl_hw *bl_hw, uint32_t ops, uint32_t task, uint32_t element,
			 uint32_t type, void *arg1, void *arg2)
{
	struct cfg_start_req *req;
#define ENTRY_BUF_SIZE 8

	req = bl_msg_zalloc(CFG_START_REQ, TASK_CFG, DRV_TASK_ID,
			    sizeof(struct cfg_start_req) + 32);
	if (!req) {
		return -ENOMEM;
	}

	req->ops = ops;
	switch (req->ops) {
	case CFG_ELEMENT_TYPE_OPS_SET:
		req->u.set[0].task = task;
		req->u.set[0].element = element;
		req->u.set[0].type = type;
		req->u.set[0].length =
			utils_tlv_bl_pack_auto(req->u.set[0].buf, ENTRY_BUF_SIZE, type, arg1);
		break;
	case CFG_ELEMENT_TYPE_OPS_DUMP_DEBUG:
		req->u.set[0].task = task;
		req->u.set[0].element = element;
		req->u.set[0].length = 0;
		break;
	default:
		break;
	}

	return bl_send_msg(bl_hw, req, 1, CFG_START_CFM, NULL);
}

int bl_send_channel_set_req(struct bl_hw *bl_hw, int channel)
{
	struct mm_set_channel_req *param;
	struct mm_set_channel_cfm cfm;

	param = bl_msg_zalloc(MM_SET_CHANNEL_REQ, TASK_MM, DRV_TASK_ID,
			      sizeof(struct mm_set_channel_req));
	if (!param) {
		return -ENOMEM;
	}

	memset(&cfm, 0, sizeof(struct mm_set_channel_cfm));

	param->band = PHY_BAND_2G4;
	param->type = PHY_CHNL_BW_20;
	param->prim20_freq = phy_channel_to_freq(param->band, channel);
	param->center1_freq = phy_channel_to_freq(param->band, channel);
	param->center2_freq = phy_channel_to_freq(param->band, channel);
	param->index = 0;
	param->tx_power = 15;

	return bl_send_msg(bl_hw, param, 1, MM_SET_CHANNEL_CFM, &cfm);
}
