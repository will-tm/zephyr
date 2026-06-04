/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/cache.h>
#include <zephyr/sys/util.h>

/* T-Head C906 CSR definitions */
#define THEAD_MXSTATUS_THEADISAEE BIT(22)
#define THEAD_MXSTATUS_MM         BIT(15)

#define THEAD_MHCR_IE  BIT(0)
#define THEAD_MHCR_DE  BIT(1)
#define THEAD_MHCR_WA  BIT(2)
#define THEAD_MHCR_WB  BIT(3)
#define THEAD_MHCR_RS  BIT(4)
#define THEAD_MHCR_BPE BIT(5)
#define THEAD_MHCR_BTB BIT(6)

#define THEAD_MHINT_DPLD BIT(2)
#define THEAD_MHINT_IPLD BIT(8)

/*
 * Shared ring buffer in XRAM for D0->M0 console forwarding.
 * M0 bootloader polls this buffer and prints to UART0.
 *
 * Layout at XRAM_CONSOLE_BASE:
 *   [0x00] uint32_t magic    - 0xD0C0FFEE when active
 *   [0x04] uint32_t write_idx
 *   [0x08] uint32_t read_idx  (written by M0)
 *   [0x0C] uint32_t size      - data area size
 *   [0x10] char     data[size]
 */
#define XRAM_CONSOLE_BASE  0x40000100
#define XRAM_CONSOLE_MAGIC 0xD0C0FFEE
#define XRAM_CONSOLE_SIZE  4080

struct xram_console {
	volatile uint32_t magic;
	volatile uint32_t write_idx;
	volatile uint32_t read_idx;
	volatile uint32_t size;
	char data[];
};

#define XRAM_CONSOLE ((struct xram_console *)XRAM_CONSOLE_BASE)

static bool dcache_active;

/* T-Head dcache.cva (clean by virtual address) */
static inline void dcache_clean_addr(unsigned long addr)
{
	register unsigned long a __asm__("a3") = addr;

	__asm__ volatile(".insn 0x256800B" : : "r"(a));
}

/* T-Head dcache.iva (invalidate by virtual address) */
static inline void dcache_inv_addr(unsigned long addr)
{
	register unsigned long a __asm__("a3") = addr;

	__asm__ volatile(".insn 0x266800B" : : "r"(a));
}

int arch_printk_char_out(int c)
{
	struct xram_console *con = XRAM_CONSOLE;

	if (dcache_active) {
		dcache_inv_addr((unsigned long)&con->read_idx);
	}

	uint32_t next = (con->write_idx + 1) % con->size;

	/* Drop character if buffer full (M0 not draining fast enough) */
	if (next == con->read_idx) {
		return c;
	}

	con->data[con->write_idx] = (char)c;
	con->write_idx = next;

	if (dcache_active) {
		dcache_clean_addr((unsigned long)&con->data[con->write_idx]);
		dcache_clean_addr((unsigned long)&con->write_idx);
	}

	return c;
}

static void xram_console_init(void)
{
	struct xram_console *con = XRAM_CONSOLE;

	con->write_idx = 0;
	con->read_idx = 0;
	con->size = XRAM_CONSOLE_SIZE;
	/* Magic last - signals M0 that the buffer is ready */
	con->magic = XRAM_CONSOLE_MAGIC;
}

static void c906_enable_thead_isa(void)
{
	unsigned long val;

	__asm__ volatile("csrr %0, 0x7C0" : "=r"(val));
	val |= THEAD_MXSTATUS_THEADISAEE | THEAD_MXSTATUS_MM;
	__asm__ volatile("csrw 0x7C0, %0" : : "r"(val));
}

/*
 * soc_reset_hook: runs from __start BEFORE stack/BSS/C-runtime.
 * Must be naked - no stack available.
 * Write a marker to XRAM to prove D0 reached this point.
 */
void __attribute__((naked)) soc_reset_hook(void)
{
	__asm__ volatile("li t0, 0x40000100\n"
			 "li t1, 0xD0A11E00\n"
			 "sw t1, 0(t0)\n"
			 "csrr t1, 0xfc1\n"
			 "sw t1, 8(t0)\n"
			 "ret\n");
}

/* MM_MISC_CPU_RTC: D0 mtimer clock gate + divider */
#define MM_MISC_CPU_RTC 0x30000018

#define CPU_FREQ   DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)
#define TIMER_FREQ DT_PROP(DT_PATH(cpus), timebase_frequency)

static void c906_mtimer_init(void)
{
	uint32_t div = (CPU_FREQ / TIMER_FREQ) - 1;
	uint32_t tmp;

	tmp = sys_read32(MM_MISC_CPU_RTC);
	tmp &= ~(1U << 31);
	sys_write32(tmp, MM_MISC_CPU_RTC);
	tmp &= ~0x3FF;
	tmp |= (div & 0x3FF);
	sys_write32(tmp, MM_MISC_CPU_RTC);
	tmp |= (1U << 31);
	sys_write32(tmp, MM_MISC_CPU_RTC);
}

void soc_early_init_hook(void)
{
	unsigned long val;

	xram_console_init();

	__asm__ volatile("sfence.vma x0, x0");
	c906_enable_thead_isa();

	/* I-cache: invalidate all, then enable */
	__asm__ volatile("fence" ::: "memory");
	__asm__ volatile("fence.i" ::: "memory");
	__asm__ volatile(".insn 0x100000B" ::: "memory"); /* icache.iall */
	__asm__ volatile("csrr %0, 0x7C1" : "=r"(val));
	val |= THEAD_MHCR_IE;
	__asm__ volatile("csrw 0x7C1, %0" : : "r"(val));
	__asm__ volatile("fence" ::: "memory");
	__asm__ volatile("fence.i" ::: "memory");

	/* D-cache: invalidate all, then enable */
	__asm__ volatile(".insn 0x20000B" ::: "memory"); /* dcache.iall */
	__asm__ volatile("csrr %0, 0x7C1" : "=r"(val));
	val |= THEAD_MHCR_DE | THEAD_MHCR_WB | THEAD_MHCR_WA |
	       THEAD_MHCR_RS | THEAD_MHCR_BPE | THEAD_MHCR_BTB;
	__asm__ volatile("csrw 0x7C1, %0" : : "r"(val));
	__asm__ volatile("fence" ::: "memory");
	__asm__ volatile("fence.i" ::: "memory");

	/* Prefetch */
	__asm__ volatile("csrr %0, 0x7C5" : "=r"(val));
	val |= THEAD_MHINT_DPLD | THEAD_MHINT_IPLD;
	__asm__ volatile("csrw 0x7C5, %0" : : "r"(val));

	dcache_active = true;
	c906_mtimer_init();
}
