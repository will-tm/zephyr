/*
 * Copyright (C) Bouffalo Lab 2016-2018
 * SPDX-License-Identifier: Apache-2.0
 *
 * Utility functions — ported from M1s SDK.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/logging/log.h>

#include "ipc_shared.h"
#include "ipc_host.h"
#include "bl_utils.h"
#include "bl_rx.h"
#include "bl_tx.h"
#include "bl_cmds.h"
#include "bl_os_private.h"

LOG_MODULE_REGISTER(bl_utils, LOG_LEVEL_ERR);

#undef os_printf
#define os_printf(...)                                                                             \
	do {                                                                                       \
	} while (0)

extern struct bl_hw wifi_hw;

/* RX callback — set by bflb_wifi_drv.c to deliver frames to net stack */
static struct net_if *rx_iface;

void bl_wifi_set_rx_iface(struct net_if *iface)
{
	rx_iface = iface;
}

/* SDK wifi_pkt structure — firmware delivers RX frames through this */
struct wifi_pkt {
	uint32_t pkt[4];
	void *pbuf[4];
	uint16_t len[4];
};

/*
 * tcpip_stack_input — packet reception callback from firmware.
 * Copies the ethernet frame into a net_pkt and delivers it to the
 * Zephyr network stack.  Returns -1 so firmware frees its buffer.
 */
int tcpip_stack_input(void *swdesc, uint8_t status, void *hwhdr, unsigned int msdu_offset,
		      struct wifi_pkt *pkt, uint8_t extra_status)
{
	struct hw_rxhdr *hw_rxhdr = (struct hw_rxhdr *)hwhdr;
	struct net_pkt *net_p;
	uint8_t *frame;
	uint16_t frame_len;

	if (!(status & RX_STAT_FORWARD)) {
		return -1;
	}

	if (hw_rxhdr->flags_is_80211_mpdu) {
		/* Raw 802.11 frame — not an ethernet frame, skip */
		return -1;
	}

	if (!rx_iface) {
		return -1;
	}

	/* First segment: ethernet frame starts at pkt->pkt[0] + msdu_offset */
	if (pkt->len[0] <= msdu_offset) {
		return -1;
	}

	frame = (uint8_t *)(pkt->pkt[0]) + msdu_offset;
	frame_len = pkt->len[0] - msdu_offset;

	if (frame_len < sizeof(struct net_eth_hdr)) {
		return -1;
	}

	/* Filter: only accept unicast (to us) or broadcast.
	 * Drop multicast (01:xx) that isn't broadcast to avoid
	 * exhausting the small RX packet/ARP entry pool. */
	bool is_mcast = (frame[0] & 0x01) &&
			!(frame[0] == 0xff && frame[1] == 0xff &&
			  frame[2] == 0xff && frame[3] == 0xff &&
			  frame[4] == 0xff && frame[5] == 0xff);

	if (is_mcast) {
		return -1;
	}

	/* Drop echoed TX frames — firmware mirrors our own transmissions
	 * back through the RX path.  Processing them wastes CPU and can
	 * trigger protocol side-effects (e.g. ARP table churn). */
	{
		const uint8_t *our_mac = net_if_get_link_addr(rx_iface)->addr;

		if (memcmp(frame + 6, our_mac, NET_ETH_ADDR_LEN) == 0) {
			return -1;
		}
	}

	/* Calculate total length including chained segments */
	uint16_t total_len = frame_len;
	for (int i = 1; i < 4 && pkt->len[i] > 0; i++) {
		total_len += pkt->len[i];
	}

	net_p = net_pkt_rx_alloc_with_buffer(rx_iface, total_len, AF_UNSPEC, 0, K_NO_WAIT);
	if (!net_p) {
		LOG_WRN("RX: no net_pkt (len=%u)", total_len);
		return -1;
	}

	/* Copy first segment */
	if (net_pkt_write(net_p, frame, frame_len) < 0) {
		LOG_WRN("RX: net_pkt_write failed");
		net_pkt_unref(net_p);
		return -1;
	}

	/* Copy chained segments */
	for (int i = 1; i < 4 && pkt->len[i] > 0; i++) {
		if (net_pkt_write(net_p, (uint8_t *)(pkt->pkt[i]), pkt->len[i]) < 0) {
			net_pkt_unref(net_p);
			return -1;
		}
	}

	/* Log ethertype + dst/src for debugging ARP/ICMP flow */
	uint16_t etype = (frame[12] << 8) | frame[13];

	LOG_DBG("RX: len=%u etype=0x%04x dst=%02x:%02x:%02x:%02x:%02x:%02x "
		"src=%02x:%02x:%02x:%02x:%02x:%02x",
		total_len, etype,
		frame[0], frame[1], frame[2], frame[3], frame[4], frame[5],
		frame[6], frame[7], frame[8], frame[9], frame[10], frame[11]);

	int rc = net_recv_data(rx_iface, net_p);
	if (rc < 0) {
		LOG_WRN("RX: net_recv_data failed: %d", rc);
		net_pkt_unref(net_p);
	}

	return -1;
}

uint8_t bl_radarind(void *pthis, void *hostid)
{
	os_printf("%s: Enter\r\n", __func__);
	return 0;
}

uint8_t bl_msgackind(void *pthis, void *hostid)
{
	struct bl_hw *bl_hw = (struct bl_hw *)pthis;

	os_printf("[IPC] MSG ACKED @%p\r\n", hostid);
	bl_hw->cmd_mgr.llind(&bl_hw->cmd_mgr, (struct bl_cmd *)hostid);
	return 0;
}

uint8_t bl_dbgind(void *pthis, void *hostid)
{
	os_printf("%s: Enter\r\n", __func__);
	return 0;
}

void bl_prim_tbtt_ind(void *pthis)
{
	os_printf("%s: Enter\r\n", __func__);
}

void bl_sec_tbtt_ind(void *pthis)
{
	os_printf("%s: Enter\r\n", __func__);
}

int bl_utils_idx_lookup(struct bl_hw *bl_hw, uint8_t *mac)
{
	int i;
	struct bl_sta *sta;

	for (i = 0; i < (int)ARRAY_SIZE(bl_hw->sta_table); i++) {
		sta = &bl_hw->sta_table[i];
		if (sta->is_used == 0) {
			continue;
		}
		if (memcmp(sta->sta_addr.array, mac, 6) == 0) {
			break;
		}
	}

	return (i == (int)ARRAY_SIZE(bl_hw->sta_table)) ? wifi_hw.ap_bcmc_idx : i;
}

static struct ipc_host_env_tag *ipc_env;

int bl_ipc_init(struct bl_hw *bl_hw, struct ipc_shared_env_tag *ipc_shared_mem)
{
	struct ipc_host_cb_tag cb;

	memset(&cb, 0, sizeof(cb));
	/* Initialize the API interface */
	cb.recv_data_ind = NULL;
	cb.recv_radar_ind = bl_radarind;
	cb.recv_msg_ind = NULL;
	cb.recv_msgack_ind = bl_msgackind;
	cb.recv_dbg_ind = bl_dbgind;
	cb.send_data_cfm = bl_txdatacfm;
	cb.prim_tbtt_ind = bl_prim_tbtt_ind;
	cb.sec_tbtt_ind = bl_sec_tbtt_ind;

	/* Set the IPC environment */
	bl_hw->ipc_env = (struct ipc_host_env_tag *)bl_os_malloc(sizeof(struct ipc_host_env_tag));
	if (!bl_hw->ipc_env) {
		LOG_ERR("ipc_env alloc failed");
		return -ENOMEM;
	}
	ipc_env = bl_hw->ipc_env;

	/* Call the initialization of the IPC */
	ipc_host_init(bl_hw->ipc_env, &cb, ipc_shared_mem, bl_hw);

	bl_cmd_mgr_init(&bl_hw->cmd_mgr);
	return 0;
}

void bl_utils_dump(void)
{
	bl_os_puts("---------- bl_utils_dump -----------\r\n");

	bl_os_printf("txdesc_free_idx: %lu(%lu)\r\n", ipc_env->txdesc_free_idx,
		     ipc_env->txdesc_free_idx & (NX_TXDESC_CNT0 - 1));
	bl_os_printf("txdesc_used_idx: %lu(%lu)\r\n", ipc_env->txdesc_used_idx,
		     ipc_env->txdesc_used_idx & (NX_TXDESC_CNT0 - 1));
	bl_os_printf("tx_host_id0 cnt: %d(used %ld)\r\n",
		     (int)(sizeof(ipc_env->tx_host_id0) / sizeof(ipc_env->tx_host_id0[0])),
		     (int32_t)ipc_env->txdesc_free_idx - (int32_t)ipc_env->txdesc_used_idx);

	bl_os_puts("========== bl_utils_dump End =======\r\n");
}
