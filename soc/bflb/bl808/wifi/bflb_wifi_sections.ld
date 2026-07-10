/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * BL606P/BL808 WiFi firmware shared memory must live in WIFI_RAM (the
 * uncached WRAM view at 0x22040000): the MAC HW DMA engine reads its
 * descriptor pools, TX buffers and the IPC environment there and cannot
 * reach the cacheable alias or PSRAM.
 *
 *   .fw.SHRAMIPC -- ipc_shared_env (host/FW mailbox + txbuf), 4 KB-aligned
 *   .fw.SHRAM    -- RX/TX descriptor pools, scan buffers
 *
 * Zephyr's arch_data_copy() / arch_bss_zero() do not touch custom
 * memory-regions, so soc_early_init_hook() (in soc/bflb/bl808/soc.c)
 * zeroes WIFI_RAM at boot.  These sections use (NOLOAD) so no flash
 * image is allocated for them (the blob's copies are all-zero).
 */

SECTION_DATA_PROLOGUE(_bl_static_cfg_entry,,)
{
	_ld_bl_static_cfg_entry_start = .;
	KEEP(*(.bl_static_cfg_entry))
	_ld_bl_static_cfg_entry_end = .;
} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

SECTION_DATA_PROLOGUE(_bflb_wifi_shramipc, (NOLOAD), ALIGN(4096))
{
	_ld_bflb_wifi_shramipc_start = .;
	KEEP(*(.fw.SHRAMIPC))
	_ld_bflb_wifi_shramipc_end = .;
} GROUP_LINK_IN(WIFI_RAM)

SECTION_DATA_PROLOGUE(_bflb_wifi_shram, (NOLOAD),)
{
	. = ALIGN(8);
	_ld_bflb_wifi_shram_start = .;
	KEEP(*(.fw.SHRAM))
	. = ALIGN(4);
	_ld_bflb_wifi_shram_end = .;
} GROUP_LINK_IN(WIFI_RAM)
