/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BL606P/BL808 WiFi TX path.
 *
 * The BL606P firmware transmits through its native data path: the host
 * copies the frame payload into the shared-memory txbuf that pairs with a
 * free txdesc0 descriptor, fills the hostdesc, pushes the descriptor onto
 * list_ongoing and rings the A2E TXDESC doorbell.  The firmware's
 * ipc_emb_tx_evt drains list_ongoing, builds the 802.11 frame in the
 * txbuf headroom (HW crypto applies the per-STA key normally) and moves
 * the descriptor to list_cfm once the MAC HW confirms it.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <lmac_types.h>
#include <bl60x_fw_api.h>
#include <lmac_mac.h>
#include <utils_list.h>
#include <ipc_compat.h>
#include <ipc_shared.h>

#include "bflb_wifi.h"
#include "bflb_wifi_ipc.h"
#include "bflb_wifi_blob.h"

LOG_MODULE_DECLARE(bflb_wifi, CONFIG_WIFI_LOG_LEVEL);

/* Headroom the firmware uses to build the 802.11 MAC/QoS/SNAP/IV header
 * in front of the payload (vendor PBUF_LINK_ENCAPSULATION_HLEN).
 */
#define BFLB_TX_HEADROOM 48U

#define BFLB_TX_FREE_WAIT_MAX_MS  200U
#define BFLB_TX_FREE_WAIT_STEP_MS 1U
#define BFLB_TX_CFM_POLL_STEP_MS  2U
#define BFLB_TX_CFM_POLL_MAX_MS   200U

/* Status word bits the FW writes via hostdesc.status_addr. */
#define BFLB_TX_STATINFO_DONE BIT(31)

/* packet_addr == this sentinel means "use chained-pbuf pointers". */
#define BFLB_HOSTDESC_PBUF_CHAINED_MAGIC 0x11111111U

#define BFLB_TXDESC_READY_FILLED 0xFFFFFFFFU

#define BFLB_ETH_MAC_LEN  6U
#define BFLB_VIF_TYPE_STA 1U /* MM_STA per vendor bl_output */
#define BFLB_TID_BE       0U

/* Blob-ABI hostdesc offsets, asserted against the SDK header layout. */
#define BFLB_HOSTDESC_OFF_STATUS_ADDR   12U
#define BFLB_HOSTDESC_OFF_ETH_DEST_ADDR 16U

struct bflb_eth_frame_hdr {
	uint8_t dst[BFLB_ETH_MAC_LEN];
	uint8_t src[BFLB_ETH_MAC_LEN];
	uint16_t etype_be;
} __packed;

extern struct ipc_shared_env_tag ipc_shared_env;

extern void ipc_emb_tx_irq(void);
extern void ipc_emb_notify(void);

/* Status word the FW writes via hostdesc.status_addr.
 * Bits: 31 DESC_DONE_TX, 23 FRAME_SUCCESSFUL_TX.
 */
volatile uint32_t bflb_wifi_tx_status;

/* Serialise TX -- single shared status word, multiple callers. */
static K_MUTEX_DEFINE(bflb_tx_mutex);

static volatile struct txdesc_host *bflb_tx_alloc_desc(void);

/* Pop a free txdesc; wait briefly for a TXCFM to recycle one rather than
 * dropping the frame (losing EAPOL msg 4 times out the 4-way handshake).
 */
static volatile struct txdesc_host *bflb_tx_alloc_desc(void)
{
	struct txdesc_host *td;
	unsigned int key;

	for (uint32_t wait = 0; wait < BFLB_TX_FREE_WAIT_MAX_MS; wait++) {
		key = irq_lock();
		td = (struct txdesc_host *)utils_list_pop_front(
			(struct utils_list *)&ipc_shared_env.list_free);
		irq_unlock(key);
		if (td != NULL) {
			return td;
		}
		k_msleep(BFLB_TX_FREE_WAIT_STEP_MS);
	}
	return NULL;
}

int bflb_wifi_tx_eth(const uint8_t *frame, uint16_t len, uint8_t vif_idx, uint8_t sta_idx)
{
	const struct bflb_eth_frame_hdr *eth = (const struct bflb_eth_frame_hdr *)frame;
	volatile struct txdesc_host *td;
	struct txbuf_host *txbuf;
	struct hostdesc *host;
	uint16_t payload_len;
	uint32_t desc_idx;
	unsigned int key;
	bool done = false;

	BUILD_ASSERT(offsetof(struct hostdesc, status_addr) == BFLB_HOSTDESC_OFF_STATUS_ADDR,
		     "hostdesc.status_addr offset");
	BUILD_ASSERT(offsetof(struct hostdesc, eth_dest_addr) == BFLB_HOSTDESC_OFF_ETH_DEST_ADDR,
		     "hostdesc.eth_dest_addr offset");

	if ((frame == NULL) || (len < BFLB_WIFI_ETH_HDR_LEN) ||
	    (((uint32_t)len + BFLB_TX_HEADROOM) > sizeof(txbuf->buf))) {
		return -EINVAL;
	}

	k_mutex_lock(&bflb_tx_mutex, K_FOREVER);

	td = bflb_tx_alloc_desc();
	if (td == NULL) {
		LOG_WRN("tx: no free txdesc after %ums", BFLB_TX_FREE_WAIT_MAX_MS);
		k_mutex_unlock(&bflb_tx_mutex);
		return -ENOMEM;
	}

	desc_idx = (uint32_t)(td - ipc_shared_env.txdesc0);
	txbuf = &ipc_shared_env.txbuf[desc_idx];
	payload_len = len - BFLB_WIFI_ETH_HDR_LEN;

	/* Stage the frame in the shared txbuf: ethernet header at the
	 * headroom offset, payload right behind it.  WIFI_RAM is uncached,
	 * so the MAC HW DMA sees the writes without a cache flush.
	 */
	txbuf->flag = 0U;
	memset(txbuf->buf, 0, BFLB_TX_HEADROOM);
	memcpy((uint8_t *)txbuf->buf + BFLB_TX_HEADROOM, frame, len);

	memset((void *)(uintptr_t)td->pad_txdesc, 0, sizeof(td->pad_txdesc));
	host = &((struct txdesc_upper *)(uintptr_t)td->pad_txdesc)->host;

	memcpy(host->eth_dest_addr.array, eth->dst, BFLB_ETH_MAC_LEN);
	memcpy(host->eth_src_addr.array, eth->src, BFLB_ETH_MAC_LEN);
	host->ethertype = eth->etype_be;
	host->packet_addr = BFLB_HOSTDESC_PBUF_CHAINED_MAGIC;
	host->packet_len = payload_len;
	host->vif_idx = vif_idx;
	host->vif_type = BFLB_VIF_TYPE_STA;
	host->staid = sta_idx;
	host->tid = BFLB_TID_BE;
	host->pbuf_addr = (uint32_t)(uintptr_t)txbuf;
	host->pbuf_chained_ptr[0] = (uint32_t)(uintptr_t)((uint8_t *)txbuf->buf +
							  BFLB_TX_HEADROOM +
							  BFLB_WIFI_ETH_HDR_LEN);
	host->pbuf_chained_len[0] = payload_len;
	bflb_wifi_tx_status = 0;
	host->status_addr = (uint32_t)(uintptr_t)&bflb_wifi_tx_status;

	td->host_id = txbuf;
	td->ready = BFLB_TXDESC_READY_FILLED;

	key = irq_lock();
	utils_list_push_back((struct utils_list *)&ipc_shared_env.list_ongoing,
			     (struct utils_list_hdr *)&td->list_hdr);
	irq_unlock(key);

	/* Ring the doorbell and drive the firmware-side IRQ handler
	 * directly -- the A2E trigger does not interrupt this core.
	 */
	ipc_a2e_trigger(BFLB_IPC_A2E_TXDESC0);
	ipc_emb_tx_irq();
	ipc_emb_notify();

	for (uint32_t poll = 0; poll < BFLB_TX_CFM_POLL_MAX_MS; poll += BFLB_TX_CFM_POLL_STEP_MS) {
		if ((bflb_wifi_tx_status & BFLB_TX_STATINFO_DONE) != 0U) {
			done = true;
			break;
		}
		k_msleep(BFLB_TX_CFM_POLL_STEP_MS);
	}

	if (!done) {
		LOG_DBG("tx: cfm timeout %ums tx_status=0x%08x", BFLB_TX_CFM_POLL_MAX_MS,
			bflb_wifi_tx_status);
	}

	k_mutex_unlock(&bflb_tx_mutex);
	return 0;
}
