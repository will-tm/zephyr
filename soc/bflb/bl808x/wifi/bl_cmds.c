/*
 * Copyright (C) Bouffalo Lab 2016-2018
 * SPDX-License-Identifier: Apache-2.0
 *
 * Command management — ported from M1s SDK.
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <bl_os_private.h>
#include "bl_cmds.h"
#include "bl_utils.h"
#include "bl_strs.h"

LOG_MODULE_REGISTER(bl_wifi_cmds, LOG_LEVEL_ERR);

#undef bl_os_log_printf
#define bl_os_log_printf(...)                                                                      \
	do {                                                                                       \
	} while (0)

static void cmd_dump(const struct bl_cmd *cmd)
{
	bl_os_log_debug("tkn[%d]  flags:%04x  result:%3d  cmd:%4d-%-24s - reqcfm(%4d-%-s)\n",
			cmd->tkn, cmd->flags, cmd->result, cmd->id, RWNX_ID2STR(cmd->id),
			cmd->reqid,
			cmd->reqid != (lmac_msg_id_t)-1 ? RWNX_ID2STR(cmd->reqid) : "none");
}

static void cmd_complete(struct bl_cmd_mgr *cmd_mgr, struct bl_cmd *cmd)
{
	cmd_mgr->queue_sz--;
	list_del(&cmd->list);
	cmd->flags |= RWNX_CMD_FLAG_DONE;
	if (cmd->flags & RWNX_CMD_FLAG_NONBLOCK) {
		bl_os_free(cmd);
	} else {
		if (RWNX_CMD_WAIT_COMPLETE(cmd->flags)) {
			cmd->result = 0;
			bl_os_event_group_send(cmd->complete, 0x1);
		}
	}
}

static int cmd_mgr_queue(struct bl_cmd_mgr *cmd_mgr, struct bl_cmd *cmd)
{
	struct bl_hw *bl_hw = container_of(cmd_mgr, struct bl_hw, cmd_mgr);
	struct bl_cmd *last;
	bool defer_push = false;
	uint32_t e;

	bl_os_mutex_lock(cmd_mgr->lock);

	if (cmd_mgr->state == RWNX_CMD_MGR_STATE_CRASHED) {
		cmd->result = EPIPE;
		bl_os_mutex_unlock(cmd_mgr->lock);
		return -EPIPE;
	}

	if (!list_empty(&cmd_mgr->cmds)) {
		if (cmd_mgr->queue_sz == cmd_mgr->max_queue_sz) {
			cmd->result = ENOMEM;
			bl_os_mutex_unlock(cmd_mgr->lock);
			return -ENOMEM;
		}
		last = list_entry(cmd_mgr->cmds.prev, struct bl_cmd, list);
		if (last->flags & (RWNX_CMD_FLAG_WAIT_ACK | RWNX_CMD_FLAG_WAIT_PUSH)) {
			cmd->flags |= RWNX_CMD_FLAG_WAIT_PUSH;
			defer_push = true;
		}
	}

	cmd->flags |= RWNX_CMD_FLAG_WAIT_ACK;
	if (cmd->flags & RWNX_CMD_FLAG_REQ_CFM) {
		cmd->flags |= RWNX_CMD_FLAG_WAIT_CFM;
	}

	cmd->tkn = cmd_mgr->next_tkn++;
	cmd->result = EINTR;

	if (!(cmd->flags & RWNX_CMD_FLAG_NONBLOCK)) {
		cmd->complete = bl_os_event_group_create();
	}

	list_add_tail(&cmd->list, &cmd_mgr->cmds);
	cmd_mgr->queue_sz++;
	bl_os_mutex_unlock(cmd_mgr->lock);

	LOG_DBG("queue id=0x%04x tkn=%u flags=0x%04x defer=%u", cmd->id, cmd->tkn, cmd->flags,
		defer_push);

	if (!defer_push) {
		ipc_host_msg_push(bl_hw->ipc_env, cmd,
				  sizeof(struct lmac_msg) + cmd->a2e_msg->param_len);
		bl_os_free(cmd->a2e_msg);
	}

	if (!(cmd->flags & RWNX_CMD_FLAG_NONBLOCK)) {
		e = bl_os_event_group_wait(cmd->complete, (1 << 0), BL_OS_TRUE, BL_OS_FALSE,
					   BL_OS_WAITING_FOREVER);
		if (e & (1 << 0)) {
			/* cmd OK */
		} else {
			cmd_dump(cmd);
			bl_os_mutex_lock(cmd_mgr->lock);
			cmd_mgr->state = RWNX_CMD_MGR_STATE_CRASHED;
			if (!(cmd->flags & RWNX_CMD_FLAG_DONE)) {
				cmd->result = ETIMEDOUT;
				cmd_complete(cmd_mgr, cmd);
			}
			bl_os_mutex_unlock(cmd_mgr->lock);
		}
		bl_os_event_group_delete(cmd->complete);
	} else {
		cmd->result = 0;
	}
	return 0;
}

static void cmd_mgr_print(struct bl_cmd_mgr *cmd_mgr)
{
	struct bl_cmd *cur;

	bl_os_mutex_lock(cmd_mgr->lock);
	list_for_each_entry(cur, &cmd_mgr->cmds, list)
	{
		cmd_dump(cur);
	}
	bl_os_mutex_unlock(cmd_mgr->lock);
}

static void cmd_mgr_drain(struct bl_cmd_mgr *cmd_mgr)
{
	struct bl_cmd *cur, *nxt;

	bl_os_mutex_lock(cmd_mgr->lock);
	list_for_each_entry_safe(cur, nxt, &cmd_mgr->cmds, list)
	{
		list_del(&cur->list);
		cmd_mgr->queue_sz--;
		if (!(cur->flags & RWNX_CMD_FLAG_NONBLOCK)) {
			bl_os_event_group_send(cur->complete, 0x1);
		}
	}
	bl_os_mutex_unlock(cmd_mgr->lock);
}

static int cmd_mgr_llind(struct bl_cmd_mgr *cmd_mgr, struct bl_cmd *cmd)
{
	struct bl_cmd *cur, *acked = NULL, *next = NULL;

	bl_os_mutex_lock(cmd_mgr->lock);
	list_for_each_entry(cur, &cmd_mgr->cmds, list)
	{
		if (!acked) {
			if (cur->tkn == cmd->tkn) {
				if (WARN_ON_ONCE(cur != cmd)) {
					cmd_dump(cmd);
				}
				acked = cur;
				continue;
			}
		}
		if (cur->flags & RWNX_CMD_FLAG_WAIT_PUSH) {
			next = cur;
			break;
		}
	}
	if (acked) {
		cmd->flags &= ~RWNX_CMD_FLAG_WAIT_ACK;
		if (RWNX_CMD_WAIT_COMPLETE(cmd->flags)) {
			cmd_complete(cmd_mgr, cmd);
		}
	}
	if (next) {
		struct bl_hw *bl_hw = container_of(cmd_mgr, struct bl_hw, cmd_mgr);
		next->flags &= ~RWNX_CMD_FLAG_WAIT_PUSH;
		ipc_host_msg_push(bl_hw->ipc_env, next,
				  sizeof(struct lmac_msg) + next->a2e_msg->param_len);
		bl_os_free(next->a2e_msg);
	}
	bl_os_mutex_unlock(cmd_mgr->lock);

	return 0;
}

static int cmd_mgr_msgind(struct bl_cmd_mgr *cmd_mgr, struct ipc_e2a_msg *msg, msg_cb_fct cb)
{
	struct bl_hw *bl_hw = container_of(cmd_mgr, struct bl_hw, cmd_mgr);
	struct bl_cmd *cmd;
	bool found = false;

	bl_os_mutex_lock(cmd_mgr->lock);
	list_for_each_entry(cmd, &cmd_mgr->cmds, list)
	{
		if (cmd->reqid == msg->id && (cmd->flags & RWNX_CMD_FLAG_WAIT_CFM)) {
			if (!cb || (cb && !cb(bl_hw, cmd, msg))) {
				found = true;
				cmd->flags &= ~RWNX_CMD_FLAG_WAIT_CFM;

				if (cmd->e2a_msg && msg->param_len) {
					memcpy(cmd->e2a_msg, &msg->param, msg->param_len);
				}

				if (RWNX_CMD_WAIT_COMPLETE(cmd->flags)) {
					cmd_complete(cmd_mgr, cmd);
				}
				break;
			}
		}
	}
	bl_os_mutex_unlock(cmd_mgr->lock);

	if (!found && cb) {
		cb(bl_hw, NULL, msg);
	}

	return 0;
}

void bl_cmd_mgr_init(struct bl_cmd_mgr *cmd_mgr)
{
	INIT_LIST_HEAD(&cmd_mgr->cmds);
	cmd_mgr->lock = bl_os_mutex_create();
	ASSERT_ERR(cmd_mgr->lock != NULL);

	cmd_mgr->max_queue_sz = RWNX_CMD_MAX_QUEUED;
	cmd_mgr->queue = &cmd_mgr_queue;
	cmd_mgr->print = &cmd_mgr_print;
	cmd_mgr->drain = &cmd_mgr_drain;
	cmd_mgr->llind = &cmd_mgr_llind;
	cmd_mgr->msgind = &cmd_mgr_msgind;
}
