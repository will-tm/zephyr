/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int coremark_run(void);

int main(void)
{
	printk("[D0] CoreMark starting\n");
	coremark_run();
	printk("[D0] CoreMark done\n");

	while (1) {
		k_sleep(K_FOREVER);
	}
	return 0;
}
