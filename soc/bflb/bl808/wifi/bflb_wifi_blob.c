/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BL606P/BL808 blob-specific IPC glue.
 *
 * The firmware runs on the same core as the host driver, and on BL808 the
 * APP2EMB doorbell does not raise a CPU interrupt.  After every A2E
 * trigger the firmware-side IRQ handler (ipc_emb_msg_irq / ipc_emb_tx_irq,
 * which reads the latched A2E status, sets the matching kernel event and
 * ACKs it) must be called directly, followed by ipc_emb_notify() to wake
 * the firmware scheduler thread.  EMB2APP (E2A) interrupts do fire and are
 * handled in bflb_wifi_blob_ipc_isr().
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

/* hostdesc sits behind the txdesc_upper co_list_hdr inside pad_txdesc;
 * status_addr is hostdesc word 3, i.e. pad_txdesc word 1 + 3.
 */
#define BFLB_TXDESC_WORD_STATUS_ADDR (1U + 3U)

#define BFLB_TXDESC_READY_FREE 0U

extern struct ipc_shared_env_tag ipc_shared_env;

extern void bl_irq_handler(void);
extern void ipc_emb_msg_irq(void);
extern void ipc_emb_notify(void);

static void bflb_wifi_ipc_recycle_txcfm(void);

/* TXCFM: the firmware moves transmitted descriptors from list_ongoing to
 * list_cfm; recycle them back onto list_free.
 */
static void bflb_wifi_ipc_recycle_txcfm(void)
{
	struct txdesc_host *td = (struct txdesc_host *)utils_list_pop_front(
		(struct utils_list *)&ipc_shared_env.list_cfm);

	while (td != NULL) {
		td->host_id = NULL;
		td->ready = BFLB_TXDESC_READY_FREE;
		utils_list_push_back((struct utils_list *)&ipc_shared_env.list_free,
				     &td->list_hdr);
		td = (struct txdesc_host *)utils_list_pop_front(
			(struct utils_list *)&ipc_shared_env.list_cfm);
	}
}

void bflb_wifi_blob_ipc_seed(void)
{
	utils_list_init((struct utils_list *)&ipc_shared_env.list_free);
	utils_list_init((struct utils_list *)&ipc_shared_env.list_ongoing);
	utils_list_init((struct utils_list *)&ipc_shared_env.list_cfm);

	memset((void *)ipc_shared_env.txdesc0, 0, sizeof(ipc_shared_env.txdesc0));

	for (uint32_t i = 0; i < NX_TXDESC_CNT0; i++) {
		volatile struct txdesc_host *td = &ipc_shared_env.txdesc0[i];

		/* Pre-seed status_addr: txu_cntrl_cfm dereferences it
		 * unconditionally and would fault on NULL.
		 */
		td->pad_txdesc[BFLB_TXDESC_WORD_STATUS_ADDR] =
			(uint32_t)(uintptr_t)&bflb_wifi_tx_status;

		utils_list_push_back((struct utils_list *)&ipc_shared_env.list_free,
				     (struct utils_list_hdr *)&td->list_hdr);
	}
}

void bflb_wifi_blob_ipc_isr(const void *arg)
{
	uint32_t status;

	ARG_UNUSED(arg);

	/* ACK the latched E2A bits -- the blob's bl_irq_handler only disables
	 * the E2A unmask and wakes the scheduler, it doesn't ACK.  Without an
	 * ACK the latched bits stay set and the IRQ re-fires.
	 */
	status = sys_read32(BFLB_IPC_E2A_RAWSTATUS);
	if (status != 0U) {
		ipc_e2a_ack(status);
	}

	bl_irq_handler();

	if ((status & IPC_IRQ_E2A_TXCFM_MASK) != 0U) {
		bflb_wifi_ipc_recycle_txcfm();
	}
}

void bflb_wifi_blob_msg_kick(void)
{
	ipc_emb_msg_irq();
	ipc_emb_notify();
}

void bflb_wifi_blob_mac_init_done(struct bflb_wifi_dev *d)
{
	ARG_UNUSED(d);
}
