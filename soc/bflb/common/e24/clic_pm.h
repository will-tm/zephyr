/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_BFLB_COMMON_E24_CLIC_PM_H_
#define ZEPHYR_SOC_BFLB_COMMON_E24_CLIC_PM_H_

#include <stdint.h>
#include <zephyr/sys/util.h>
#include <bflb_soc.h>

/* Backup arrays must be sized to this many uint32_t words. The actual
 * extern symbol bflb_clic_pm_save/restore reference is sized via DIV
 * here so callers can declare static storage of the right size without
 * pulling in the implementation file's macro.
 */
#define BFLB_CLIC_PM_BACKUP_WORDS DIV_ROUND_UP(IRQn_LAST + 1U, 4U)

/**
 * @brief Snapshot the CLIC INTIE + INTCFG arrays into caller storage.
 *
 * @param intie_dst   Destination for INTIE bytes, packed as 4-bytes-per-word.
 * @param intcfg_dst  Destination for INTCFG bytes, same packing.
 */
void bflb_clic_pm_save(uint32_t intie_dst[BFLB_CLIC_PM_BACKUP_WORDS],
		       uint32_t intcfg_dst[BFLB_CLIC_PM_BACKUP_WORDS]);

/**
 * @brief Zero INTIE and INTIP for every IRQ.
 *
 * Used right after a wake-from-reset to put the CLIC into a known
 * quiescent state before any peripheral driver re-installs its IRQ.
 */
void bflb_clic_pm_clear_all(void);

/**
 * @brief Restore CLIC INTCFG then INTIE arrays from a previous snapshot.
 *
 * INTCFG must be valid before any interrupt is unmasked, so the order
 * matters -- this helper restores INTCFG first, then INTIE.
 */
void bflb_clic_pm_restore(const uint32_t intie_src[BFLB_CLIC_PM_BACKUP_WORDS],
			  const uint32_t intcfg_src[BFLB_CLIC_PM_BACKUP_WORDS]);

#endif /* ZEPHYR_SOC_BFLB_COMMON_E24_CLIC_PM_H_ */
