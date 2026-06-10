/*
 * Copyright (c) 2021 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief interrupt management code for riscv SOCs supporting the SiFive clic
 */
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <bflb_soc.h>
#include <soc.h>
#include "clic.h"
#include "clic_pm.h"

#define CLIC_INTCFG_PADDING_BITS 4

void riscv_clic_irq_enable(unsigned int irq)
{
	*(volatile uint8_t *)(CLIC_HART0_ADDR + CLIC_INTIE + irq) = 1;
}

void riscv_clic_irq_disable(unsigned int irq)
{
	*(volatile uint8_t *)(CLIC_HART0_ADDR + CLIC_INTIE + irq) = 0;
}

void riscv_clic_irq_priority_set(unsigned int irq, unsigned int prio, uint32_t flags)
{
	*(volatile uint8_t *)(CLIC_HART0_ADDR + CLIC_INTCFG + irq) =
		(prio & 0xF) << CLIC_INTCFG_PADDING_BITS;
	ARG_UNUSED(flags);
}

int riscv_clic_irq_is_enabled(unsigned int irq)
{
	return *(volatile uint8_t *)(CLIC_HART0_ADDR + CLIC_INTIE + irq);
}

#ifdef CONFIG_PM

/* CLIC INTIE/INTCFG save/restore for deep-sleep paths that reset the CPU. */

static inline volatile uint32_t *bflb_clic_intie_ptr(void)
{
	return (volatile uint32_t *)(CLIC_HART0_ADDR + CLIC_INTIE);
}

static inline volatile uint32_t *bflb_clic_intip_ptr(void)
{
	return (volatile uint32_t *)(CLIC_HART0_ADDR + CLIC_INTIP);
}

static inline volatile uint32_t *bflb_clic_intcfg_ptr(void)
{
	return (volatile uint32_t *)(CLIC_HART0_ADDR + CLIC_INTCFG);
}

void bflb_clic_pm_save(uint32_t intie_dst[BFLB_CLIC_PM_BACKUP_WORDS],
		       uint32_t intcfg_dst[BFLB_CLIC_PM_BACKUP_WORDS])
{
	const volatile uint32_t *ie_src = bflb_clic_intie_ptr();
	const volatile uint32_t *cfg_src = bflb_clic_intcfg_ptr();

	for (uint32_t i = 0U; i < BFLB_CLIC_PM_BACKUP_WORDS; i++) {
		intie_dst[i] = ie_src[i];
		intcfg_dst[i] = cfg_src[i];
	}
}

void bflb_clic_pm_clear_all(void)
{
	volatile uint32_t *ie = bflb_clic_intie_ptr();
	volatile uint32_t *ip = bflb_clic_intip_ptr();

	for (uint32_t i = 0U; i < BFLB_CLIC_PM_BACKUP_WORDS; i++) {
		ie[i] = 0U;
		ip[i] = 0U;
	}
}

void bflb_clic_pm_restore(const uint32_t intie_src[BFLB_CLIC_PM_BACKUP_WORDS],
			  const uint32_t intcfg_src[BFLB_CLIC_PM_BACKUP_WORDS])
{
	volatile uint32_t *cfg = bflb_clic_intcfg_ptr();
	volatile uint32_t *ie = bflb_clic_intie_ptr();

	/* INTCFG before INTIE: priority/level must be valid before any
	 * interrupt is unmasked, otherwise the CLIC vectors through
	 * zeroed INTCFG entries.
	 */
	for (uint32_t i = 0U; i < BFLB_CLIC_PM_BACKUP_WORDS; i++) {
		cfg[i] = intcfg_src[i];
	}
	for (uint32_t i = 0U; i < BFLB_CLIC_PM_BACKUP_WORDS; i++) {
		ie[i] = intie_src[i];
	}
}

#endif /* CONFIG_PM */
