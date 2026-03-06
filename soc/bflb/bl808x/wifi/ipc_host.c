/*
 * Copyright (C) Bouffalo Lab 2016-2018
 * SPDX-License-Identifier: Apache-2.0
 *
 * IPC host driver — ported from M1s SDK.
 * Removed lwIP/utils_list dependencies for Zephyr.
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/cache.h>

#include "ipc_host.h"
#include "reg_ipc_app.h"
#include "bl_cmds.h"
#include "bl_utils.h"
#include "bl_tx.h"
#include "bl_os_private.h"
#include <zephyr/kernel.h>

extern void bl_tx_resend(void);

/*
 * Software IPC notification — on BL808 the LMAC firmware runs on the same
 * core as the host driver, so the APP2EMB hardware trigger does NOT generate
 * a CPU interrupt.  We must wake the firmware thread explicitly.
 *
 * ke_evt_set() atomically ORs event bits into ke_env.evt_field.
 * ipc_emb_notify() signals the firmware's blocking semaphore.
 */
extern void ke_evt_set(uint32_t events);
extern void ipc_emb_notify(void);

/* Event bit 5 (from MSB) = ipc_emb_msg_evt handler in ke_evt_hdlr table.
 * __clzsi2(0x04000000) == 5, so bit 26 maps to handler index 5. */
#define KE_EVT_IPC_EMB_MSG 0x04000000

/* Event bits for ipc_emb_tx_evt: indices 19-23 from MSB = bits 12-8.
 * IPC_IRQ_A2E_TXDESC uses queue 0, which maps to handler index 23 = bit 8. */
#define KE_EVT_IPC_EMB_TX0 0x00000100

#define REG_SW_SET_PROFILING(env, value)                                                           \
	do {                                                                                       \
	} while (0)
#define REG_SW_CLEAR_PROFILING(env, value)                                                         \
	do {                                                                                       \
	} while (0)
#define REG_SW_SET_PROFILING_CHAN(env, bit)                                                        \
	do {                                                                                       \
	} while (0)
#define REG_SW_CLEAR_PROFILING_CHAN(env, bit)                                                      \
	do {                                                                                       \
	} while (0)

#undef os_printf
#define os_printf(...)                                                                             \
	do {                                                                                       \
	} while (0)

static const int nx_txdesc_cnt[] = {
	NX_TXDESC_CNT0, NX_TXDESC_CNT1, NX_TXDESC_CNT2, NX_TXDESC_CNT3,
#if NX_TXQ_CNT == 5
	NX_TXDESC_CNT4,
#endif
};

static const int nx_txdesc_cnt_msk[] = {
	NX_TXDESC_CNT0 - 1, NX_TXDESC_CNT1 - 1, NX_TXDESC_CNT2 - 1, NX_TXDESC_CNT3 - 1,
#if NX_TXQ_CNT == 5
	NX_TXDESC_CNT4 - 1,
#endif
};

void ipc_host_init(struct ipc_host_env_tag *env, struct ipc_host_cb_tag *cb,
		   struct ipc_shared_env_tag *shared_env_ptr, void *pthis)
{
	/* Reset the IPC Host environment */
	memset(env, 0, sizeof(struct ipc_host_env_tag));

	/* Initialize the shared environment pointer */
	env->shared = shared_env_ptr;

	/* Save the callbacks in our own environment */
	env->cb = *cb;

	/* Save the pointer to the register base */
	env->pthis = pthis;

	/* Initialize buffers numbers and buffers sizes needed for DMA Receptions */
	env->rx_bufnb = IPC_RXBUF_CNT;
	env->rx_bufsz = IPC_RXBUF_SIZE;
	env->rxdesc_nb = IPC_RXDESC_CNT;
	env->ipc_e2amsg_bufnb = IPC_MSGE2A_BUF_CNT;
	env->ipc_e2amsg_bufsz = sizeof(struct ipc_e2a_msg);

	/* Initialize the pointers to the hostid arrays */
	env->tx_host_id = env->tx_host_id0;

	/* Initialize the pointers to the TX descriptor arrays */
	env->txdesc = shared_env_ptr->txdesc0;
	memset((void *)&(shared_env_ptr->txdesc0), 0, sizeof(shared_env_ptr->txdesc0));
}

int ipc_host_msg_push(struct ipc_host_env_tag *env, void *msg_buf, uint16_t len)
{
	int i;
	uint32_t *src, *dst;

	ASSERT_ERR(!env->msga2e_hostid);
	ASSERT_ERR(round_up(len, 4) <= sizeof(env->shared->msg_a2e_buf.msg));

	/* Copy the message into the IPC MSG buffer */
	src = (uint32_t *)((struct bl_cmd *)msg_buf)->a2e_msg;
	dst = (uint32_t *)&(env->shared->msg_a2e_buf.msg);

	/* Copy the message in the IPC queue */
	for (i = 0; i < len; i += 4) {
		*dst++ = *src++;
	}

	env->msga2e_hostid = msg_buf;

	/* Ensure message is visible in PSRAM before waking firmware */
	sys_cache_data_flush_all();

	/* Trigger the irq to send the message to EMB */
	ipc_app2emb_trigger_set(IPC_IRQ_A2E_MSG);

	/* Wake firmware: HW IRQ doesn't fire on same core */
	ke_evt_set(KE_EVT_IPC_EMB_MSG);
	ipc_emb_notify();

	return 0;
}

void ipc_host_patt_addr_push(struct ipc_host_env_tag *env, uint32_t addr)
{
	struct ipc_shared_env_tag *shared_env_ptr = env->shared;

	/* Copy the address */
	shared_env_ptr->pattern_addr = addr;
}

uint32_t ipc_host_get_status(struct ipc_host_env_tag *env)
{
	return ipc_emb2app_status_get();
}

uint32_t ipc_host_get_rawstatus(struct ipc_host_env_tag *env)
{
	return ipc_emb2app_rawstatus_get();
}

static void ipc_host_msgack_handler(struct ipc_host_env_tag *env)
{
	void *hostid = env->msga2e_hostid;

	ASSERT_ERR(hostid);

	env->msga2e_hostid = NULL;
	env->msga2e_cnt++;
	env->cb.recv_msgack_ind(env->pthis, hostid);
}

static void ipc_host_tx_cfm_handler(struct ipc_host_env_tag *env, const int queue_idx,
				    const int user_pos)
{
	int ret;

	while (1) {
		uint32_t used_idx = env->txdesc_used_idx;
		void *host_id = env->tx_host_id[used_idx & nx_txdesc_cnt_msk[queue_idx]];

		if (host_id == NULL) {
			break;
		}

		ret = env->cb.send_data_cfm(env->pthis, host_id);
		if (ret < 0) {
			break;
		}
		/* Reset the host id in the array */
		env->tx_host_id[used_idx & nx_txdesc_cnt_msk[queue_idx]] = NULL;
		/* current txdesc is confirmed, so increase the idx now */
		env->txdesc_used_idx++;
	}
}

static void ipc_host_radar_handler(struct ipc_host_env_tag *env)
{
	/* empty */
}

static void ipc_host_dbg_handler(struct ipc_host_env_tag *env)
{
	while (env->cb.recv_dbg_ind(env->pthis,
				    env->ipc_host_dbgbuf_array[env->ipc_host_dbg_idx].hostid) == 0)
		;
}

volatile struct txdesc_host *ipc_host_txdesc_get(struct ipc_host_env_tag *env)
{
	volatile struct txdesc_host *txdesc_free;
	uint32_t used_idx = env->txdesc_used_idx;
	uint32_t free_idx = env->txdesc_free_idx;

	/* Check if a free descriptor is available */
	if (free_idx != (used_idx + nx_txdesc_cnt[0])) {
		txdesc_free = env->txdesc + (free_idx & nx_txdesc_cnt_msk[0]);
	} else {
		txdesc_free = NULL;
	}

	return txdesc_free;
}

int ipc_host_txdesc_left(struct ipc_host_env_tag *env, const int queue_idx, const int user_pos)
{
	uint32_t used_idx = env->txdesc_used_idx;
	uint32_t free_idx = env->txdesc_free_idx;

	return nx_txdesc_cnt[queue_idx] - (free_idx - used_idx);
}

void ipc_host_txdesc_push(struct ipc_host_env_tag *env, void *host_id)
{
	uint32_t free_idx = env->txdesc_free_idx & nx_txdesc_cnt_msk[0];
	volatile struct txdesc_host *txdesc_pushed = env->txdesc + free_idx;

	/* Descriptor is now ready */
	txdesc_pushed->ready = 0xFFFFFFFF;

	/* Save the host id in the environment */
	env->tx_host_id[free_idx] = host_id;

	/* Increment the index */
	env->txdesc_free_idx++;

	/* Ensure descriptor is visible in PSRAM before waking firmware */
	sys_cache_data_flush_all();

	/* Trigger interrupt */
	ipc_app2emb_trigger_setf(CO_BIT(IPC_IRQ_A2E_TXDESC_FIRSTBIT));

	/* Wake firmware: HW IRQ doesn't fire on same core */
	ke_evt_set(KE_EVT_IPC_EMB_TX0);
	ipc_emb_notify();
}

void ipc_host_irq(struct ipc_host_env_tag *env, uint32_t status)
{
	/* Acknowledge the pending interrupts */
	ipc_emb2app_ack_clear(status);
	/* Re-read to ensure acknowledgment is effective */
	status |= ipc_emb2app_status_get();

	if (status & IPC_IRQ_E2A_TXCFM) {
		int i;

		for (i = 0; i < IPC_TXQUEUE_CNT; i++) {
			uint32_t q_bit = CO_BIT(i + IPC_IRQ_E2A_TXCFM_POS);
			if (status & q_bit) {
				ipc_host_tx_cfm_handler(env, i, 0);
			}
		}
	}

	/* Resend pending TX frames */
	bl_tx_resend();

	if (status & IPC_IRQ_E2A_MSG) {
		if (env->cb.recv_msg_ind) {
			env->cb.recv_msg_ind(env->pthis, NULL);
		}
	}
	if (status & IPC_IRQ_E2A_MSG_ACK) {
		ipc_host_msgack_handler(env);
	}
	if (status & IPC_IRQ_E2A_RADAR) {
		ipc_host_radar_handler(env);
	}
	if (status & IPC_IRQ_E2A_DBG) {
		ipc_host_dbg_handler(env);
	}
	if (status & IPC_IRQ_E2A_TBTT_PRIM) {
		env->cb.prim_tbtt_ind(env->pthis);
	}
	if (status & IPC_IRQ_E2A_TBTT_SEC) {
		env->cb.sec_tbtt_ind(env->pthis);
	}
}

void ipc_host_enable_irq(struct ipc_host_env_tag *env, uint32_t value)
{
	ipc_emb2app_unmask_set(value);
}

void ipc_host_disable_irq(struct ipc_host_env_tag *env, uint32_t value)
{
	ipc_emb2app_unmask_clear(value);
}

void ipc_host_disable_irq_e2a(void)
{
	ipc_emb2app_unmask_clear(IPC_IRQ_E2A_ALL);
}
