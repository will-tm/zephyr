/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Linker section for WiFi static config entries required by libwifi.a
 */

SECTION_DATA_PROLOGUE(_bl_static_cfg,,)
{
	_ld_bl_static_cfg_entry_start = .;
	KEEP(*(.bl_static_cfg_entry))
	_ld_bl_static_cfg_entry_end = .;
} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)
