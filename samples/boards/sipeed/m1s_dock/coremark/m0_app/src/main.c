/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <bflb_soc.h>
#include <glb_reg.h>
#include <mm_glb_reg.h>
#include <mm_misc_reg.h>
#include <pds_reg.h>

#define D0_FLASH_OFFSET  0x100000
#define D0_L2SRAM_BASE   0x3ef80000
#define D0_L2SRAM_SIZE   (512 * 1024)
#define D0_BOOT_ADDR     D0_L2SRAM_BASE
#define LP_FLASH_OFFSET  0x020000
#define LP_SRAM_BASE     0x2202c000
#define LP_SRAM_SIZE     (48 * 1024)
#define LP_BOOT_ADDR     LP_SRAM_BASE
#define SF_CTRL          0x2000b000
#define SF_ID1_OFF       0xA4

#define IPC_SYNC_ADDR1 0x40000000
#define IPC_SYNC_ADDR2 0x40000004
#define IPC_SYNC_FLAG  0x12345678

struct xram_console {
	volatile uint32_t magic;
	volatile uint32_t write_idx;
	volatile uint32_t read_idx;
	volatile uint32_t size;
	char data[];
};

#define D0_XRAM_BASE  0x40000100
#define D0_XRAM_MAGIC 0xD0C0FFEE
#define LP_XRAM_BASE  0x40001200
#define LP_XRAM_MAGIC 0x1EC0FFEE

static void xram_drain(uint32_t base, uint32_t magic)
{
	volatile struct xram_console *con = (volatile struct xram_console *)base;

	if (con->magic != magic) {
		return;
	}

	while (con->read_idx != con->write_idx) {
		printk("%c", con->data[con->read_idx]);
		con->read_idx = (con->read_idx + 1) % con->size;
	}
}

static void release_d0_core(void)
{
	uint32_t tmp;

	/* Copy D0 firmware from flash to L2SRAM */
	volatile uint32_t *src = (volatile uint32_t *)(0x58000000 + D0_FLASH_OFFSET);
	volatile uint32_t *dst = (volatile uint32_t *)D0_L2SRAM_BASE;

	for (uint32_t i = 0; i < D0_L2SRAM_SIZE / 4; i++) {
		dst[i] = src[i];
	}

	tmp = sys_read32(0x20005300);
	tmp |= BIT(0);
	sys_write32(tmp, 0x20005300);

	sys_write32(D0_BOOT_ADDR, MM_MISC_BASE + MM_MISC_CPU0_BOOT_OFFSET);

	tmp = sys_read32(MM_GLB_BASE + MM_GLB_MM_CLK_CTRL_CPU_OFFSET);
	tmp |= MM_GLB_REG_MMCPU0_CLK_EN_MSK;
	sys_write32(tmp, MM_GLB_BASE + MM_GLB_MM_CLK_CTRL_CPU_OFFSET);
	k_busy_wait(1);
	tmp = sys_read32(MM_GLB_BASE + MM_GLB_MM_SW_SYS_RESET_OFFSET);
	tmp &= ~MM_GLB_REG_CTRL_MMCPU0_RESET_MSK;
	sys_write32(tmp, MM_GLB_BASE + MM_GLB_MM_SW_SYS_RESET_OFFSET);

	sys_write32(IPC_SYNC_FLAG, IPC_SYNC_ADDR1);
	sys_write32(IPC_SYNC_FLAG, IPC_SYNC_ADDR2);
	sys_cache_data_flush_all();
}

static void release_lp_core(void)
{
	uint32_t tmp;

	/* Copy LP firmware from flash to SRAM */
	volatile uint32_t *src = (volatile uint32_t *)(0x58000000 + LP_FLASH_OFFSET);
	volatile uint32_t *dst = (volatile uint32_t *)LP_SRAM_BASE;

	for (uint32_t i = 0; i < LP_SRAM_SIZE / 4; i++) {
		dst[i] = src[i];
	}

	/* LP mtimer clock: PBCLK=80MHz, div=79 -> 1MHz tick */
	tmp = sys_read32(PDS_BASE + PDS_CPU_CORE_CFG8_OFFSET);
	tmp &= ~(1U << 31);
	sys_write32(tmp, PDS_BASE + PDS_CPU_CORE_CFG8_OFFSET);
	tmp &= ~0x3FFU;
	tmp |= 79;
	sys_write32(tmp, PDS_BASE + PDS_CPU_CORE_CFG8_OFFSET);
	tmp |= (1U << 31);
	sys_write32(tmp, PDS_BASE + PDS_CPU_CORE_CFG8_OFFSET);

	sys_write32(LP_BOOT_ADDR, PDS_BASE + PDS_CPU_CORE_CFG13_OFFSET);

	tmp = sys_read32(PDS_BASE + PDS_CPU_CORE_CFG0_OFFSET);
	tmp |= PDS_REG_PICO_CLK_EN_MSK;
	sys_write32(tmp, PDS_BASE + PDS_CPU_CORE_CFG0_OFFSET);
	k_busy_wait(1);
	tmp = sys_read32(GLB_BASE + GLB_SWRST_CFG2_OFFSET);
	tmp &= ~GLB_REG_CTRL_PICO_RESET_MSK;
	sys_write32(tmp, GLB_BASE + GLB_SWRST_CFG2_OFFSET);
}

#define DRAIN_STACK_SIZE 1024
#define DRAIN_PRIORITY   -1

static void drain_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		xram_drain(D0_XRAM_BASE, D0_XRAM_MAGIC);
		xram_drain(LP_XRAM_BASE, LP_XRAM_MAGIC);
		k_msleep(10);
	}
}

K_THREAD_DEFINE(drain_tid, DRAIN_STACK_SIZE, drain_thread, NULL, NULL, NULL, DRAIN_PRIORITY, 0, 0);

int coremark_run(void);

int main(void)
{
	printk("[M0] Releasing D0 + LP...\n");
	release_d0_core();
	release_lp_core();
	printk("[M0] All cores released\n");

	printk("[M0] CoreMark starting\n");
	coremark_run();
	printk("[M0] CoreMark done\n");

	while (1) {
		k_sleep(K_FOREVER);
	}
	return 0;
}
