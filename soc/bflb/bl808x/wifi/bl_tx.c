/*
 * Copyright (C) Bouffalo Lab 2016-2018
 * SPDX-License-Identifier: Apache-2.0
 *
 * TX path — minimal implementation for EAPOL frame transmission.
 * Full data TX path is milestone 4.
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

/* Sentinel value: packet data is in pbuf_chained, not at packet_addr */
#define HOSTDESC_PBUF_CHAINED_MAGIC 0x11111111

/* Firmware chan_env addresses (from nm output of libwifi.a) */
#define FW_CHAN_CURRENT_ADDR   0x2204a6d0
#define FW_CHAN_OPERATING_ADDR 0x2204a6d4
#define FW_CHAN_STATE_OFFSET   22
#define FW_CHAN_STATE_ACTIVE   6

extern struct bl_hw wifi_hw;
extern void bl_irq_handler(void);

/* ipc_shared_env (including txdesc->eth_packet) is now in WRAM where
 * MAC DMA can reach it.  No separate TX buffer needed — we copy payload
 * directly into txdesc->eth_packet and point pbuf_chained_ptr there,
 * matching the SDK bl_tx_push pattern. */

int bl_txdatacfm(void *pthis, void *host_id)
{
	if (host_id) {
		volatile struct txdesc_host *txdesc = (volatile struct txdesc_host *)host_id;
		uint32_t status = txdesc->pad_txdesc[0];

		if (status == 0) {
			return -1;
		}
		LOG_DBG("TX cfm: status=0x%08x", status);
		return 1;
	}
	return 0;
}

void bl_tx_try_flush(int param, struct ke_tx_fc *tx_fc_field)
{
	ARG_UNUSED(tx_fc_field);
	ARG_UNUSED(param);
}

void bl_tx_resend(void)
{
}

/*
 * bl_output_raw — send a raw Ethernet frame through the firmware.
 *
 * Follows the SDK bl_tx_push / bl_output pattern:
 * 1. Get a free txdesc_host from ipc_host_txdesc_get
 * 2. Copy payload into txdesc_host->eth_packet
 * 3. Fill txdesc_host->host (hostdesc)
 * 4. Push via ipc_host_txdesc_push
 */
int bl_output_raw(const uint8_t *frame, uint16_t len, uint8_t vif_idx, uint8_t sta_idx)
{
#if defined(CFG_CHIP_BL808) || defined(CFG_CHIP_BL606P)
	volatile struct txdesc_host *txdesc;
	volatile struct hostdesc *host;
	const struct ethhdr *eth = (const struct ethhdr *)frame;
	uint16_t payload_len;
	uint32_t copy_len;

	if (!frame || len < sizeof(struct ethhdr)) {
		return -EINVAL;
	}

	payload_len = len - sizeof(struct ethhdr);

	/* Get a free TX descriptor */
	txdesc = ipc_host_txdesc_get(wifi_hw.ipc_env);
	if (!txdesc) {
		LOG_ERR("no free txdesc");
		return -ENOMEM;
	}

	/* Copy payload into txdesc->eth_packet (now in WRAM, DMA-accessible).
	 * SDK bl_tx_push does the same: memcpy to eth_packet, then
	 * pbuf_chained_ptr[0] = &txdesc->eth_packet[0]. */
	copy_len = payload_len;
	if (copy_len > sizeof(txdesc->eth_packet)) {
		copy_len = sizeof(txdesc->eth_packet);
	}
	memcpy((void *)txdesc->eth_packet, frame + sizeof(struct ethhdr), copy_len);

	/* Fill the hostdesc — matches SDK bl_output + bl_tx_push pattern */
	host = &txdesc->host;
	memset((void *)host, 0, sizeof(*host));

	memcpy((void *)&host->eth_dest_addr, eth->h_dest, ETH_ALEN);
	memcpy((void *)&host->eth_src_addr, eth->h_source, ETH_ALEN);
	host->ethertype = eth->h_proto;
	host->packet_len = payload_len;
	/* M1s SDK convention: vif_idx is used as is_sta flag (1=STA, 0=AP).
	 * Firmware internally maps this to the actual VIF index ("fixed in ipc_emb").
	 * staid and tid are also filled by firmware. */
	host->vif_idx = 1; /* STA mode */
	host->flags = 0;
	host->packet_addr = HOSTDESC_PBUF_CHAINED_MAGIC;

	/* Point to payload in txdesc->eth_packet (WRAM, DMA-accessible).
	 * Matches SDK bl_tx_push: pbuf_chained_ptr[0] = &txdesc->eth_packet[0] */
	host->pbuf_chained_ptr[0] = (uint32_t)&txdesc->eth_packet[0];
	host->pbuf_chained_len[0] = copy_len;

	/* status_addr: firmware writes TX completion status here. */
	host->status_addr = (uint32_t)&txdesc->pad_txdesc[0];
	txdesc->pad_txdesc[0] = 0;

	/* pbuf_addr: returned as host_id in TX confirm. */
	host->pbuf_addr = (uint32_t)txdesc;

	/* Workaround: firmware's chan_switch_start doesn't set operating channel
	 * when current == target with single link (typical STA mode).
	 * This leaves chan_env[36] == NULL causing chan_is_tx_allowed() to
	 * block all data TX.  Set it here if needed.
	 * Addresses from firmware nm: chan_env.chan_ctxt_pool + offsets. */
	{
		volatile uint32_t *chan_current = (volatile uint32_t *)FW_CHAN_CURRENT_ADDR;
		volatile uint32_t *chan_operating = (volatile uint32_t *)FW_CHAN_OPERATING_ADDR;

		if (*chan_current && !*chan_operating) {
			*chan_operating = *chan_current;
			volatile uint8_t *chan_state =
				(volatile uint8_t *)(*chan_current + FW_CHAN_STATE_OFFSET);
			*chan_state = FW_CHAN_STATE_ACTIVE;
		}
	}

	LOG_DBG("TX: len=%u etype=0x%04x dst=%02x:%02x:%02x:%02x:%02x:%02x",
		len, sys_be16_to_cpu(eth->h_proto),
		eth->h_dest[0], eth->h_dest[1], eth->h_dest[2],
		eth->h_dest[3], eth->h_dest[4], eth->h_dest[5]);

	ipc_host_txdesc_push(wifi_hw.ipc_env, (void *)txdesc);
	bl_irq_handler();

	return 0;
#else
	return -ENOTSUP;
#endif
}
