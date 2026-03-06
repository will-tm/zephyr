/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * WiFi firmware data structures.
 *
 * These are "common" (C) symbols from libwifi.a that would otherwise
 * land in SRAM BSS.  By defining them here with explicit section
 * attributes we place them in PSRAM, matching the SDK's BSS layout.
 */

#include <stdint.h>

/*
 * Sizes and alignments from: riscv64-zephyr-elf-nm libwifi.a | grep " C "
 * These must match exactly or the firmware will corrupt memory.
 */

/* IPC shared env — MUST be in WRAM (default BSS).  The firmware builds
 * TX Hardware Descriptors (THDs) and 802.11 headers inside txdesc_host,
 * and the MAC DMA engine reads from those addresses.  MAC DMA cannot
 * reach PSRAM (0x50000000+), only WRAM (0x22030000+). */
__attribute__((aligned(4))) uint8_t ipc_shared_env[0x25c4];

/* WiFi MAC DMA targets — must be in WRAM (DMA engine cannot reach PSRAM) */
__attribute__((aligned(8))) uint8_t rx_payload_desc_buffer[0x11608];
__attribute__((aligned(8))) uint8_t rx_dma_hdrdesc[0x1004];
__attribute__((aligned(8))) uint8_t rx_payload_desc[0x854];
__attribute__((aligned(4))) uint8_t rx_swdesc_tab[0x3d8];
