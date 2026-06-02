/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>

/*
 * XRAM console ring buffer for LP→M0 output forwarding.
 * Separate from D0's buffer — uses offset 0x40000200.
 */
#define XRAM_CONSOLE_BASE  0x40001200
#define XRAM_CONSOLE_MAGIC 0x1EC0FFEE
#define XRAM_CONSOLE_SIZE  2048

struct xram_console {
	volatile uint32_t magic;
	volatile uint32_t write_idx;
	volatile uint32_t read_idx;
	volatile uint32_t size;
	char data[];
};

#define XRAM_CONSOLE ((struct xram_console *)XRAM_CONSOLE_BASE)

int arch_printk_char_out(int c)
{
	struct xram_console *con = XRAM_CONSOLE;
	uint32_t next = (con->write_idx + 1) % con->size;

	if (next == con->read_idx) {
		return c;
	}

	con->data[con->write_idx] = (char)c;
	con->write_idx = next;

	return c;
}

static void xram_console_init(void)
{
	struct xram_console *con = XRAM_CONSOLE;

	con->write_idx = 0;
	con->read_idx = 0;
	con->size = XRAM_CONSOLE_SIZE;
	con->magic = XRAM_CONSOLE_MAGIC;
}

void soc_early_init_hook(void)
{
	xram_console_init();
}
