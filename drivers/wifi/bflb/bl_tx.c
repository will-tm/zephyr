/*
 * Copyright (C) Bouffalo Lab 2016-2018
 * SPDX-License-Identifier: Apache-2.0
 *
 * TX path — follows SDK bl_output / bl_tx_push / bl_tx_try_flush pattern.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "bl_tx.h"
#include "bl_defs.h"
#include "bl_os_private.h"
#include "ipc_host.h"

LOG_MODULE_REGISTER(bl_tx, LOG_LEVEL_ERR);

#define HOSTDESC_PBUF_CHAINED_MAGIC 0x11111111
#define BIT_STA(sta)                (1U << (sta))

#ifndef PBUF_LINK_ENCAPSULATION_HLEN
#define PBUF_LINK_ENCAPSULATION_HLEN 48
#endif

#ifndef RWNX_HWTXHDR_ALIGN_PADS
#define RWNX_HWTXHDR_ALIGN_PADS(x) ((((x) + 7) & ~7) - (x))
#endif

extern struct bl_hw wifi_hw;
extern void bl_irq_handler(void);

uint32_t tx_cntrl_sta_trigger;
uint32_t tx_cntrl_sta_trigger_pending;

/* ---- bl_tx_push: fill IPC TX descriptor and push to firmware ---- */

static void bl_tx_push(struct bl_sta *sta, struct txdesc_host *txdesc_host,
		       struct txbuf_host *txbuf, struct bl_txhdr *txhdr)
{
	struct hostdesc *host;
	struct ethhdr *ethhdr;
	uint16_t link_offset_len = sizeof(struct ethhdr) + PBUF_LINK_ENCAPSULATION_HLEN;

	memset((void *)txdesc_host->pad_txdesc, 0, sizeof(txdesc_host->pad_txdesc));
	host = &(((struct txdesc_upper *)txdesc_host->pad_txdesc)->host);

	ethhdr = (struct ethhdr *)((uint8_t *)txbuf->buf + PBUF_LINK_ENCAPSULATION_HLEN);

	memcpy(&host->eth_dest_addr, ethhdr->h_dest, ETH_ALEN);
	memcpy(&host->eth_src_addr, ethhdr->h_source, ETH_ALEN);
	host->ethertype = ethhdr->h_proto;
	host->vif_type = txhdr->vif_type;
	host->packet_len = txhdr->len - link_offset_len;
	host->vif_idx = wifi_hw.vif_table[sta->vif_idx].vif_idx;
	host->staid = sta->sta_idx;
	host->tid = (sta->qos ? 0 : 0xFF);
	host->packet_addr = HOSTDESC_PBUF_CHAINED_MAGIC;
	host->flags = 0;

	host->pbuf_chained_ptr[0] = (uint32_t)((uint8_t *)txbuf->buf + link_offset_len);
	host->pbuf_chained_len[0] = txhdr->len - link_offset_len;

	struct bl_txhdr *new_txhdr =
		(struct bl_txhdr *)((uint8_t *)txbuf->buf +
				    RWNX_HWTXHDR_ALIGN_PADS((uint32_t)txbuf->buf));
	host->status_addr = (uint32_t)&new_txhdr->status;
	new_txhdr->status.value = 0;
	host->pbuf_addr = (uint32_t)txbuf;

	sys_cache_data_flush_all();
	ipc_host_txdesc_push(wifi_hw.ipc_env, (void *)txbuf);
}

/* ---- bl_tx_try_flush: move packets from waiting_list to IPC ---- */

void bl_tx_try_flush(int param, struct ke_tx_fc *tx_fc_field)
{
	struct txdesc_host *txdesc_host;
	struct bl_txhdr *txhdr;
	struct bl_sta *sta;
	uint32_t sta_trigger;

	ARG_UNUSED(param);
	ARG_UNUSED(tx_fc_field);

	bl_os_enter_critical();
	sta_trigger = tx_cntrl_sta_trigger | tx_cntrl_sta_trigger_pending;
	tx_cntrl_sta_trigger = 0;
	tx_cntrl_sta_trigger_pending = 0;
	bl_os_exit_critical();

	for (uint8_t i = 0; i < NX_REMOTE_STA_STORE_MAX && sta_trigger; i++) {
		sta = &wifi_hw.sta_table[i];
		if (!(sta_trigger & BIT_STA(i)) || !sta->is_used) {
			continue;
		}

		while (!utils_list_is_empty(&sta->waiting_list)) {
			txdesc_host = (struct txdesc_host *)ipc_host_txdesc_get(wifi_hw.ipc_env);
			if (!txdesc_host) {
				tx_cntrl_sta_trigger_pending |= BIT_STA(i);
				break;
			}

			txhdr = (struct bl_txhdr *)utils_list_pop_front(&sta->waiting_list);
			if (!txhdr) {
				break;
			}

			/* Find matching txbuf by descriptor index */
			uint32_t desc_idx = txdesc_host - ipc_shared_env.txdesc0;
			struct txbuf_host *txbuf = &ipc_shared_env.txbuf[desc_idx];

			LOG_INF("flush: sta=%d desc=%u", i, desc_idx);
			bl_tx_push(sta, txdesc_host, txbuf, txhdr);
		}
	}
}

/* ---- TX confirm and resend ---- */

int bl_txdatacfm(void *pthis, void *host_id)
{
	if (host_id) {
		struct txbuf_host *txbuf = (struct txbuf_host *)host_id;
		uint16_t align = RWNX_HWTXHDR_ALIGN_PADS((uint32_t)txbuf->buf);
		struct bl_txhdr *txhdr = (struct bl_txhdr *)((uint8_t *)txbuf->buf + align);

		if (txhdr->status.value == 0) {
			return -1;
		}
		LOG_DBG("TX cfm: status=0x%08x", txhdr->status.value);
		return 1;
	}
	return 0;
}

void bl_tx_resend(void)
{
	if (tx_cntrl_sta_trigger_pending) {
		bl_os_enter_critical();
		tx_cntrl_sta_trigger |= tx_cntrl_sta_trigger_pending;
		tx_cntrl_sta_trigger_pending = 0;
		bl_os_exit_critical();
	}
}

/* ---- bl_output_raw: SDK bl_output pattern ---- */

int bl_output_raw(const uint8_t *frame, uint16_t len, uint8_t vif_idx, uint8_t sta_idx)
{
	struct bl_sta *sta = NULL;
	int sta_id = -1;

	if (!frame || len < sizeof(struct ethhdr)) {
		return -EINVAL;
	}

	for (int i = 0; i < NX_REMOTE_STA_STORE_MAX; i++) {
		if (wifi_hw.sta_table[i].is_used) {
			sta_id = i;
			sta = &wifi_hw.sta_table[i];
			break;
		}
	}
	if (!sta) {
		LOG_ERR("TX: no active STA");
		return -ENOTCONN;
	}

	/* Get txbuf matching the descriptor that bl_tx_try_flush will pop.
	 * ipc_host_txdesc_get peeks at list_free front — use same index. */
	volatile struct txdesc_host *peek = ipc_host_txdesc_get(wifi_hw.ipc_env);
	if (!peek) {
		LOG_ERR("TX: no free descriptor");
		return -ENOMEM;
	}
	uint32_t desc_idx = peek - ipc_shared_env.txdesc0;
	struct txbuf_host *txbuf = &ipc_shared_env.txbuf[desc_idx];

	/* Layout in txbuf->buf: [headroom(48)] [eth_hdr(14)] [payload...] */
	uint16_t total = PBUF_LINK_ENCAPSULATION_HLEN + len;
	if (total > sizeof(txbuf->buf) + 4) {
		return -ENOMEM;
	}
	memset(txbuf->buf, 0, PBUF_LINK_ENCAPSULATION_HLEN);
	memcpy((uint8_t *)txbuf->buf + PBUF_LINK_ENCAPSULATION_HLEN, frame, len);

	uint16_t align = RWNX_HWTXHDR_ALIGN_PADS((uint32_t)txbuf->buf);
	struct bl_txhdr *txhdr = (struct bl_txhdr *)((uint8_t *)txbuf->buf + align);
	memset(txhdr, 0, sizeof(*txhdr));
	txhdr->p = (uint32_t *)txbuf;
	txhdr->len = total;
	txhdr->vif_type = 0;
	txhdr->sta_id = sta_id;

	LOG_INF("TX: len=%u etype=0x%04x sta=%d", len,
		sys_be16_to_cpu(((struct ethhdr *)frame)->h_proto), sta_id);

	bl_os_enter_critical();
	utils_list_push_back(&sta->waiting_list, &txhdr->item);
	tx_cntrl_sta_trigger |= BIT_STA(sta_id);
	bl_os_exit_critical();

	bl_irq_handler();

	return 0;
}
