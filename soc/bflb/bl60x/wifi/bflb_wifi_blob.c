/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BL602 blob-specific IPC glue and firmware quirk workarounds.
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
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

/* A2E_STATUS bits dispatched to the firmware-side IPC IRQ chain. */
#define BFLB_A2E_MSG_BIT         BIT(1)
#define BFLB_A2E_RXDESC_BACK_BIT BIT(4)
#define BFLB_A2E_RXBUF_BACK_BIT  BIT(5)
#define BFLB_A2E_TXDESC_BITS     (BIT(8) | BIT(9) | BIT(10) | BIT(11) | BIT(12))

/* Submit/done lists in shared env.
 *
 * The blob's `struct ipc_shared_env_tag` is 0x6f4 bytes; its list pointers
 * live at fixed offsets addressed by raw pointer.  +0x6e4 is the submit
 * queue (host pushes, FW pops), +0x6ec the recycle queue.
 */
#define BFLB_IPC_LIST_SUBMIT_OFF 0x6E4U
#define BFLB_IPC_LIST_DONE_OFF   0x6ECU
#define BFLB_IPC_LIST_BYTES      16U /* 2 lists * sizeof(utils_list) */

#define BFLB_IPC_TXDESC0_OFF 516U /* msg_a2e_buf(512) + pattern_addr(4) */

/* status_addr lives at hostdesc word 7, which is td_words[3 + 4]. */
#define BFLB_TXDESC_WORD_STATUS_ADDR (3U + 4U)

/* RX-batch quirk: rxu_swdesc_upload_evt drops new RX descs once
 * `rxl_cntrl_env + 20` reaches 5; the per-VIF TBTT counter triggers a
 * null-data probe past 100 (see bflb_rx_batch_reset_handler).
 */
#define BFLB_RXL_CNTRL_BATCH_OFF 20U
#define BFLB_RX_BATCH_RESET_MS   20
#define BFLB_VIF_TBTT_CNT_OFF    0x74U
#define BFLB_VIF_MAX             2U

extern struct ipc_shared_env_tag ipc_shared_env;

extern void bl_irq_handler(void);
extern void ipc_emb_tx_irq(void);
extern void ipc_emb_msg_irq(void);
extern void ipc_emb_cfmback_irq(void);
extern int __real_rxu_cntrl_frame_handle(void *frame);

extern uint8_t rxl_cntrl_env[];
extern uint8_t vif_info_tab[];

static struct utils_list *bflb_ipc_list_done(void);
static void bflb_wifi_ipc_recycle_txcfm(void);
static void bflb_rx_batch_reset_handler(struct k_timer *t);

static K_TIMER_DEFINE(bflb_rx_batch_timer, bflb_rx_batch_reset_handler, NULL);

static struct utils_list *bflb_ipc_list_done(void)
{
	return (struct utils_list *)((uint8_t *)&ipc_shared_env + BFLB_IPC_LIST_DONE_OFF);
}

/* TXCFM handler: the blob has no list_cfm; txu_cntrl_cfm clears `ready` in
 * place.  Just drain the done list so it doesn't grow unbounded.
 */
static void bflb_wifi_ipc_recycle_txcfm(void)
{
	struct utils_list *done = bflb_ipc_list_done();
	struct utils_list_hdr *cur = done->first;

	while (cur != NULL) {
		struct utils_list_hdr *next = cur->next;

		cur->next = NULL;
		cur = next;
	}
	done->first = NULL;
	done->last = NULL;
}

/* RX-batch counter unstick: on a busy channel the batch counter pegs and
 * unicast RX (e.g. DHCP OFFER) gets dropped -- periodically reset it.
 * Also zero the per-VIF TBTT counter: once it exceeds 100 the FW sends a
 * null-data probe through a TX path the SW-CCMP wrap can't service, the
 * probe fails and the FW tears the link down.
 */
static void bflb_rx_batch_reset_handler(struct k_timer *t)
{
	ARG_UNUSED(t);
	*(volatile uint32_t *)&rxl_cntrl_env[BFLB_RXL_CNTRL_BATCH_OFF] = 0U;

	for (uint8_t i = 0; i < BFLB_VIF_MAX; i++) {
		uint8_t *vif = &vif_info_tab[i * BFLB_WIFI_VIF_INFO_STRIDE];

		vif[BFLB_VIF_TBTT_CNT_OFF] = 0U;
	}
}

volatile uint8_t *bflb_wifi_ipc_txdesc(uint32_t idx)
{
	return (volatile uint8_t *)&ipc_shared_env + BFLB_IPC_TXDESC0_OFF +
	       (idx * BFLB_WIFI_TXDESC_STRIDE);
}

void bflb_wifi_blob_ipc_seed(void)
{
	struct utils_list *submit =
		(struct utils_list *)((uint8_t *)&ipc_shared_env + BFLB_IPC_LIST_SUBMIT_OFF);

	memset(submit, 0, BFLB_IPC_LIST_BYTES);
	memset((void *)(uintptr_t)bflb_wifi_ipc_txdesc(0), 0,
	       BFLB_WIFI_TXDESC_COUNT * BFLB_WIFI_TXDESC_STRIDE);

	/* Pre-seed every descriptor's hostdesc.status_addr with a non-NULL
	 * host word.  The blob's txu_cntrl_cfm dereferences status_addr
	 * unconditionally and would fault on NULL.
	 */
	for (uint32_t i = 0; i < BFLB_WIFI_TXDESC_COUNT; i++) {
		volatile uint32_t *td = (volatile uint32_t *)bflb_wifi_ipc_txdesc(i);

		td[BFLB_TXDESC_WORD_STATUS_ADDR] = (uint32_t)(uintptr_t)&bflb_wifi_tx_status;
	}
}

void bflb_wifi_blob_ipc_isr(const void *arg)
{
	uint32_t status;
	uint32_t emb_status;

	ARG_UNUSED(arg);

	/* ACK the latched E2A bits -- the blob's bl_irq_handler only disables
	 * the E2A unmask and wakes the scheduler, it doesn't ACK.  Without an
	 * ACK the latched bits stay set and the IRQ re-fires.
	 */
	status = sys_read32(BFLB_IPC_E2A_RAWSTATUS);
	if (status != 0U) {
		ipc_e2a_ack(status);
	}

	/* Drive the FW-side IPC IRQ chain inline.  The blob's intc_init wires
	 * IRQs 61-63 to ipc_emb_msg_irq / ipc_emb_cfmback_irq / ipc_emb_tx_irq
	 * in its own intc table, but Zephyr routes the IRQ here first.
	 * Without these calls A2E_TXDESC triggers never wake ipc_emb_tx_evt.
	 */
	emb_status = sys_read32(BFLB_IPC_E2A_STATUS);
	if ((emb_status & BFLB_A2E_TXDESC_BITS) != 0U) {
		ipc_emb_tx_irq();
	}
	if ((emb_status & BFLB_A2E_MSG_BIT) != 0U) {
		ipc_emb_msg_irq();
	}
	if ((emb_status & (BFLB_A2E_RXDESC_BACK_BIT | BFLB_A2E_RXBUF_BACK_BIT)) != 0U) {
		ipc_emb_cfmback_irq();
	}

	bl_irq_handler();
	if ((status & IPC_IRQ_E2A_TXCFM_MASK) != 0U) {
		bflb_wifi_ipc_recycle_txcfm();
	}
}

/* On BL602 the A2E doorbell raises the shared IPC interrupt line, which
 * re-enters the ISR above and drives the firmware-side IRQ chain -- no
 * extra wake needed.
 */
void bflb_wifi_blob_msg_kick(void)
{
}

void bflb_wifi_blob_mac_init_done(struct bflb_wifi_dev *d)
{
	ARG_UNUSED(d);
	k_timer_start(&bflb_rx_batch_timer, K_MSEC(BFLB_RX_BATCH_RESET_MS),
		      K_MSEC(BFLB_RX_BATCH_RESET_MS));
}

/* RX control-path passthrough (diagnostics hook point). */
int __wrap_rxu_cntrl_frame_handle(void *frame)
{
	return __real_rxu_cntrl_frame_handle(frame);
}
