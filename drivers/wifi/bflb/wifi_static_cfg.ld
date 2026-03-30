/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Linker sections for WiFi firmware shared memory and static config.
 */

SECTION_DATA_PROLOGUE(_bl_static_cfg,,)
{
	_ld_bl_static_cfg_entry_start = .;
	KEEP(*(.bl_static_cfg_entry))
	_ld_bl_static_cfg_entry_end = .;
} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

/* WiFi firmware shared RAM — IPC environment, TX/RX descriptors, scan buffers.
 * Must be in WRAM where both CPU and WiFi MAC DMA can access.
 */
SECTION_DATA_PROLOGUE(_fw_shramipc,,ALIGN(4))
{
	KEEP(*(.fw.SHRAMIPC))
} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

SECTION_DATA_PROLOGUE(_fw_shram,,ALIGN(4))
{
	KEEP(*(.fw.SHRAM))
} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)
