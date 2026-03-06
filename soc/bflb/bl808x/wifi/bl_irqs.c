/*
 * Copyright (C) Bouffalo Lab 2016-2018
 * SPDX-License-Identifier: Apache-2.0
 *
 * IRQ bottom-half handler — ported from M1s SDK.
 */

#include "bl_defs.h"
#include "bl_irqs.h"
#include "ipc_host.h"
#include "bl_os_private.h"

static struct bl_hw *wifi_hw;

int bl_irqs_init(struct bl_hw *bl_hw)
{
	wifi_hw = bl_hw;
	return 0;
}

int bl_irqs_enable(void)
{
	return 0;
}

int bl_irqs_disable(void)
{
	return 0;
}

void bl_irq_bottomhalf(struct bl_hw *bl_hw)
{
	uint32_t status;
	bool done = false;

	status = ipc_host_get_rawstatus(bl_hw->ipc_env);

	while (!done) {
		while (status != 0U) {
			/* All kinds of IRQs will be handled in one shot */
			ipc_host_irq(bl_hw->ipc_env, status);
			status = ipc_host_get_rawstatus(bl_hw->ipc_env);
		}

		ipc_host_enable_irq(bl_hw->ipc_env, IPC_IRQ_E2A_ALL);

		/* Check status again for race conditions */
		status = ipc_host_get_rawstatus(bl_hw->ipc_env);
		if (status == 0U) {
			done = true;
		}
	}
}
