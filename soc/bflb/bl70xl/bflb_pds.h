/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_BFLB_BL70XL_BFLB_PDS_H_
#define SOC_BFLB_BL70XL_BFLB_PDS_H_

#include <stdint.h>

#include <bflb_soc.h>
#include <hbn_reg.h>
#include <pds_reg.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>

#define PDS_SOC_HBN_RSV0_ADDR     (HBN_BASE + HBN_RSV0_OFFSET)
#define PDS_SOC_STATUS_ENTER_FLAG 0x4E424845U /* "ENBE" ASCII */
#define PDS_SOC_XTAL_TYPE         1U          /* 32 MHz crystal */
#define PDS_SOC_IRQN_MTIMER       7U
#define PDS_SOC_IRQN_PDS_WAKEUP   66U
#define PDS_SOC_IRQN_HBN_OUT0     67U

#if defined(CONFIG_BT_BFLB_BL70XL) || defined(CONFIG_RISCV_GP)
#define PDS_SOC_FASTBOOT_NEEDS_GP 1
#endif

#define PDS_SOC_UART_BASE UART0_BASE

/* HBN-domain wakeup pins (AON pads routed through HBN, not PDS GPIO) */
#define PDS_PIN_IS_HBN(pin) (((pin) >= 9U && (pin) <= 13U) || ((pin) >= 30U && (pin) <= 31U))

#define PDS_HBN_WAKEUP_GPIO_9  BIT(0)
#define PDS_HBN_GPIO_TRIG_ASYNC_FALLING 4U
#define PDS_HBN_WORKAROUND_MASK 0xFC00U
#define PDS_HBN_IRQ_GPIO_MASK  0x7FU

/* HBN pad ↔ GPIO bit mapping (shared between bflb_pds.c and bflb_pds_itcm.c) */
#define HBN_PAD_LOW_BASE  9U
#define HBN_PAD_LOW_WIDTH 5U
#define HBN_PAD_HIGH_BASE 30U
#define HBN_PAD_HIGH_BIT  5U

/* PDS GPIO group lookup for non-HBN pins */
#define PDS_GPIO_GROUP(pin)                                                                        \
	(((pin) <= 3)    ? 0U                                                                      \
	 : ((pin) == 7)  ? 1U                                                                      \
	 : ((pin) == 8)  ? 2U                                                                      \
	 : ((pin) <= 15) ? 3U                                                                      \
	 : ((pin) <= 19) ? 4U                                                                      \
	 : ((pin) <= 23) ? 5U                                                                      \
	 : ((pin) <= 27) ? 6U                                                                      \
			 : 7U)

#define PDS_GPIO_GROUP_BITS    4U
#define PDS_GPIO_INT_BOTH_EDGE 4U

#define PDS_INT_WAKEUP_SRC_PDS_IO  BIT(PDS_CR_PDS_WAKEUP_SRC_EN_POS + 3U)
#define PDS_GPIO_INT_CLR_ALL       (0xFFU << PDS_CR_PDS_GPIO0_SET_INT_CLR_POS)

/* HBN SRAM power control */
#define HBN_SRAM_ACTIVE    (BIT(0) | BIT(1) | BIT(3))
#define HBN_SRAM_RETENTION (BIT(0) | BIT(1) | BIT(3) | BIT(6))

void pds_gpio_wakeup_cfg(uint8_t pin);
void pds_gpio_wakeup_clear(void);

void pds_flash_init(void);
void flash_bflb_pm_suspend(void);
void flash_bflb_pm_resume(void);

#ifdef CONFIG_BL70XL_PDS_S2RAM
int bflb_pds_system_off(void);
void pds_fastboot_entry(void);
void bflb_pds_enter_s2ram(uint32_t sleep_cycles);
#endif

#endif /* SOC_BFLB_BL70XL_BFLB_PDS_H_ */
