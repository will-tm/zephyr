/*
 * Copyright (C) Bouffalo Lab 2016-2018
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform init — ported from M1s SDK.
 */

#include "bl_platform.h"
#include "bl_irqs.h"
#include "bl_utils.h"
#include "bl_os_private.h"
#include "reg_access.h"
#include "reg_ipc_app.h"

struct ipc_shared_env_tag *ipc_shenv;

int bl_platform_on(struct bl_hw *bl_hw)
{
	int ret;

	ipc_shenv = (struct ipc_shared_env_tag *)(&ipc_shared_env);
	ret = bl_ipc_init(bl_hw, ipc_shenv);
	if (ret) {
		return ret;
	}

	/* Clear any pending IRQ */
	ipc_emb2app_ack_clear(0xFFFFFFFF);

	return 0;
}

void bl_platform_off(struct bl_hw *bl_hw)
{
	ipc_host_disable_irq(bl_hw->ipc_env, IPC_IRQ_E2A_ALL);
}
