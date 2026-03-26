/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware port for the BL606P native BLE controller blob on BL808.
 *
 * The BL808 and BL606P are the same silicon. The BLE controller is a
 * precompiled RivieraWaves-based blob that communicates with the
 * BLE baseband hardware through Exchange Memory (EM) at bus
 * address 0x28000000.
 *
 * Exchange Memory (EM) — 0x28000000
 *   EM is a hardware bus bridge that maps a portion of WRAM into the BLE
 *   baseband's DMA address space. GLB_SRAM_CFG3[7:0] (EM_SEL) controls
 *   which 8KB WRAM banks are repurposed as EM. The BLE hardware DMA
 *   reads/writes descriptors, scheduling tables, and packet buffers here.
 *   The CPU accesses the same physical memory through the bus bridge.
 *
 *   Critical: CPU writes to 0x28000000 require a Bufferable sysmap
 *   attribute. Strongly Ordered (SO) silently corrupts writes because
 *   the bus bridge does not support non-buffered write transactions.
 *   EM_SEL must be configured BEFORE the blob runs, as btble_ke_mem_init
 *   (which initialises the EM heap) executes before ble_rf_init (which
 *   would otherwise set EM_SEL).
 *
 * This file provides:
 *   - bl808_rf_init()  — PHY/RF calibration via BL808-native wl_init()
 *   - printf/puts stubs — silence phyrf calibration output (picolibc
 *     stdout is NULL; the blob calls printf directly)
 *   - bl_flash_read/write/erase — XIP flash read, writes stubbed
 *   - bl_irq_register/enable/disable — mapped to Zephyr IRQ APIs
 *   - bl_efuse_read_mac_smart() — MAC address from efuse via hwinfo
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/hwinfo.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#include <bflb_soc.h>
#include <glb_reg.h>
#include <pds_reg.h>
#include <aon_reg.h>
#include <wl_api.h>

static uint8_t wl_rmem_buf[CONFIG_BFLB_BL808_WL_RMEM_SIZE] __aligned(4);

#define XTAL_FREQ        DT_PROP(DT_NODELABEL(clk_crystal), clock_frequency)
#define EFUSE_TRIM_UNSET 0x80 /* phyrf uses internal cal when trim not loaded */
#define TX_PWR_DBM       10   /* default TX power target for all radio modes */

int bl808_rf_init(void)
{
	struct wl_cfg_t *cfg;

	cfg = wl_cfg_get(wl_rmem_buf);
	cfg->mode = WL_API_MODE_BZ;
	cfg->en_param_load = 0;
	cfg->en_full_cal = 1;
	cfg->capcode_get = NULL;
	cfg->capcode_set = NULL;
	cfg->param_load = NULL;
	cfg->log_level = WL_LOG_LEVEL_NONE;
	cfg->log_printf = NULL;
	cfg->param.xtalfreq_hz = XTAL_FREQ;
	cfg->param.ef.dcdc_vout_trim_aon = EFUSE_TRIM_UNSET;
	cfg->param.ef.icx_code = EFUSE_TRIM_UNSET;
	cfg->param.ef.iptat_code = EFUSE_TRIM_UNSET;
	cfg->param.pwrtarget.pwr_ble = TX_PWR_DBM;
	cfg->param.pwrtarget.pwr_bt[0] = TX_PWR_DBM;
	cfg->param.pwrtarget.pwr_bt[1] = TX_PWR_DBM;
	cfg->param.pwrtarget.pwr_bt[2] = TX_PWR_DBM;
	cfg->param.pwrtarget.pwr_zigbee = TX_PWR_DBM;

	return wl_init();
}

int printf(const char *fmt, ...)
{
	return 0;
}

int vprintf(const char *fmt, va_list ap)
{
	return 0;
}

#undef putchar
int putchar(int c)
{
	return c;
}

#undef puts
int puts(const char *s)
{
	return 0;
}

int bl_flash_read(uint32_t addr, uint8_t *dst, int len)
{
	memcpy(dst, (const uint8_t *)(0x58000000 + addr), len);
	return 0;
}

int bl_flash_erase(uint32_t addr, int len)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(len);
	return -1;
}

int bl_flash_write(uint32_t addr, const uint8_t *src, int len)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(src);
	ARG_UNUSED(len);
	return -1;
}

int bl_efuse_read_mac_smart(uint8_t smart, uint8_t mac[6], uint8_t slot)
{
	ARG_UNUSED(smart);
	ARG_UNUSED(slot);

	return (hwinfo_get_device_id(mac, 6) >= 6) ? 0 : -1;
}

int EF_Ctrl_Read_MAC_Address(uint8_t mac[6])
{
	return (hwinfo_get_device_id(mac, 6) >= 6) ? 0 : -1;
}

void bl_irq_register(int irqnum, void *handler)
{
	irq_connect_dynamic(irqnum, 0, (void (*)(const void *))handler, NULL, 0);
}

void bl_irq_enable(unsigned int source)
{
	irq_enable(source);
}

void bl_irq_disable(unsigned int source)
{
	irq_disable(source);
}

void bl_irq_pending_clear(unsigned int source)
{
	ARG_UNUSED(source);
}
