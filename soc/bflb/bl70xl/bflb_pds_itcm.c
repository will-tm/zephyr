/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>
#include <zephyr/arch/common/pm_s2ram.h>

#include <clic.h>
#include <clic_pm.h>
#include <bflb_soc.h>
#include <glb_reg.h>
#include <pds_reg.h>
#include <hbn_reg.h>
#include <aon_reg.h>
#include <bouffalolab/common/uart_reg.h>

#include "bflb_pds.h"
#include "bl70xl_rom_api.h"

#define MTIMECMP_BASE DT_REG_ADDR_BY_NAME(DT_NODELABEL(mtimer), mtimecmp)
#define MTIMECMP_LO   (MTIMECMP_BASE)
#define MTIMECMP_HI   (MTIMECMP_BASE + 4U)

#define PDS_CTL_PDS_START_POS        0U
#define PDS_CTL_SW_PU_FLASH_POS      10U
#define PDS_CTL_LDO11_OFF_POS        22U
#define PDS_CTL_ABORT_PDS_PWR_ON_POS 27U
#define PDS_CTL_ABORT_VALUE (BIT(PDS_CTL_SW_PU_FLASH_POS) | BIT(PDS_CTL_ABORT_PDS_PWR_ON_POS))

#define PDS_CFG_GPIO_KEEP(n)                                                                       \
	(((uint32_t)(n) << PDS_CR_PDS_GPIO_KEEP_EN_POS) &                                          \
	 GENMASK(PDS_CR_PDS_GPIO_KEEP_EN_POS + 2U, PDS_CR_PDS_GPIO_KEEP_EN_POS))

#define PDS_CFG_LEVEL31_GPIO_KEEP 4U

#define PDS_CFG_LEVEL31_CTL                                                                        \
	(BIT(PDS_CTL_PDS_START_POS) | BIT(PDS_CR_PDS_PD_AVDD14_POS) |                              \
	 BIT(PDS_CR_PDS_PD_BG_SYS_POS) | BIT(PDS_CR_PDS_GATE_CLK_POS) |                            \
	 BIT(PDS_CR_PDS_MEM_STBY_POS) | BIT(PDS_CTL_SW_PU_FLASH_POS) |                             \
	 BIT(PDS_CR_PDS_ISO_EN_POS) | BIT(PDS_CR_PDS_PWR_OFF_POS) | BIT(PDS_CR_PDS_PD_XTAL_POS) |  \
	 BIT(PDS_CR_PDS_CTRL_SOC_ENB_POS) | BIT(PDS_CR_PDS_RST_SOC_EN_POS) |                       \
	 BIT(PDS_CR_PDS_LDO_VSEL_EN_POS) | BIT(PDS_CTL_LDO11_OFF_POS) |                            \
	 BIT(PDS_CR_PDS_CTRL_RF_POS) | PDS_CFG_GPIO_KEEP(PDS_CFG_LEVEL31_GPIO_KEEP))

#define PDS_CFG_LEVEL31_CTL3 BIT(PDS_CR_PDS_MISC_ISO_EN_POS)

#define PDS_CFG_LEVEL31_CTL4                                                                       \
	(BIT(PDS_CR_PDS_NP_RESET_POS) | BIT(PDS_CR_PDS_NP_GATE_CLK_POS) |                          \
	 BIT(PDS_CR_PDS_BZ_RESET_POS) | BIT(PDS_CR_PDS_BZ_GATE_CLK_POS) |                          \
	 BIT(PDS_CR_PDS_MISC_PWR_OFF_POS) | BIT(PDS_CR_PDS_MISC_RESET_POS) |                       \
	 BIT(PDS_CR_PDS_MISC_GATE_CLK_POS))

static const uint32_t bflb_pds_level31_cfg[4] = {
	PDS_CFG_LEVEL31_CTL,
	0U,
	PDS_CFG_LEVEL31_CTL3,
	PDS_CFG_LEVEL31_CTL4,
};

#define GLB_SYS_CLK_RC32M 0U

#define AON_RF_TOP_POWER_MASK (BIT(0) | BIT(1) | BIT(2))
#define AON_RF_TOP_XTAL_PU    (BIT(5) | BIT(6))

#define GPIO_PIN_COUNT         32U
#define GPIO_PINS_PER_CFG_WORD 2U
#define GPIO_CFG_BITS_PER_PIN  16U
#define GPIO_CFG_WORD_SIZE     4U
#define GPIO_FUNC_SEL_WIDTH    5U
#define GPIO_FUNC_SEL_GPIO     11U

/* Not in pm_s2ram.h; only ARM ports declare this publicly. */
extern int arch_pm_s2ram_resume(void);

extern uint32_t clic_intie_backup[];
extern uint32_t clic_intcfg_backup[];
extern uint32_t bflb_pds_sleep_cycles_32k;

static inline uint32_t gpio_to_hbn_bits(uint32_t gpio_mask)
{
	uint32_t hbn = (gpio_mask >> HBN_PAD_LOW_BASE) & BIT_MASK(HBN_PAD_LOW_WIDTH);

	hbn |= (gpio_mask >> HBN_PAD_HIGH_BASE) << HBN_PAD_HIGH_BIT;
	return hbn;
}

static void pds_uart_drain(void)
{
	while ((sys_read32(PDS_SOC_UART_BASE + UART_STATUS_OFFSET) & UART_STS_UTX_BUS_BUSY) != 0U) {
	}
}

static void pds_mtimer_park(void)
{
	sys_write8(0U, CLIC_HART0_ADDR + CLIC_INTIE + PDS_SOC_IRQN_MTIMER);
	sys_write32(UINT32_MAX, MTIMECMP_LO);
	sys_write32(UINT32_MAX, MTIMECMP_HI);
	sys_write8(0U, CLIC_HART0_ADDR + CLIC_INTIP + PDS_SOC_IRQN_MTIMER);
}

/* Match PDS/HBN pulls to GPIO output state so pins don't float in sleep. */
static void pds_gpio_set_pulls(void)
{
	uint32_t output_en = sys_read32(GLB_BASE + GLB_GPIO_CFGCTL34_OFFSET);
	uint32_t output_val = sys_read32(GLB_BASE + GLB_GPIO_CFGCTL32_OFFSET);
	uint32_t pu = 0U;
	uint32_t pd = 0U;
	uint32_t hbn_pu;
	uint32_t hbn_pd;
	uint32_t tmp;

	for (uint32_t pin = 0U; pin < GPIO_PIN_COUNT; pin++) {
		uint32_t cfg_off;
		uint32_t cfg;
		uint32_t shift;
		uint32_t func_sel;

		if ((output_en & BIT(pin)) == 0U) {
			continue;
		}

		cfg_off = GLB_GPIO_CFGCTL0_OFFSET +
			  (pin / GPIO_PINS_PER_CFG_WORD) * GPIO_CFG_WORD_SIZE;
		cfg = sys_read32(GLB_BASE + cfg_off);
		shift = (pin % GPIO_PINS_PER_CFG_WORD) * GPIO_CFG_BITS_PER_PIN;
		func_sel = (cfg >> (shift + GLB_REG_GPIO_0_FUNC_SEL_POS)) &
			   BIT_MASK(GPIO_FUNC_SEL_WIDTH);

		if (func_sel != GPIO_FUNC_SEL_GPIO) {
			continue;
		}
		if ((output_val & BIT(pin)) != 0U) {
			pu |= BIT(pin);
		} else {
			pd |= BIT(pin);
		}
	}

	sys_write32(sys_read32(PDS_BASE + PDS_GPIO_PU_SET_OFFSET) | pu,
		    PDS_BASE + PDS_GPIO_PU_SET_OFFSET);
	sys_write32(sys_read32(PDS_BASE + PDS_GPIO_PD_SET_OFFSET) | pd,
		    PDS_BASE + PDS_GPIO_PD_SET_OFFSET);

	hbn_pu = gpio_to_hbn_bits(pu);
	hbn_pd = gpio_to_hbn_bits(pd);
	if ((hbn_pu | hbn_pd) != 0U) {
		tmp = sys_read32(HBN_BASE + HBN_PAD_CTRL_1_OFFSET);
		tmp |= hbn_pu << HBN_REG_AON_GPIO_PU_POS;
		tmp |= hbn_pd << HBN_REG_AON_GPIO_PD_POS;
		sys_write32(tmp, HBN_BASE + HBN_PAD_CTRL_1_OFFSET);

		tmp = sys_read32(HBN_BASE + HBN_PAD_CTRL_0_OFFSET);
		tmp |= (hbn_pu | hbn_pd) << HBN_REG_EN_AON_CTRL_GPIO_POS;
		sys_write32(tmp, HBN_BASE + HBN_PAD_CTRL_0_OFFSET);
	}
}

int bflb_pds_system_off(void)
{
	bflb_clic_pm_save(clic_intie_backup, clic_intcfg_backup);
	bflb_clic_pm_clear_all();

	pds_mtimer_park();

	pds_uart_drain();
	flash_bflb_pm_suspend();
	GLB_Set_System_CLK(PDS_SOC_XTAL_TYPE, GLB_SYS_CLK_RC32M);
	GLB_Power_Off_DLL();
	AON_Power_Off_XTAL();

	sys_write32(UINT32_MAX, HBN_BASE + HBN_IRQ_CLR_OFFSET);
	sys_write32(HBN_SRAM_RETENTION, HBN_BASE + HBN_SRAM_OFFSET);

	sys_write8(1U, CLIC_HART0_ADDR + CLIC_INTIE + PDS_SOC_IRQN_HBN_OUT0);

	HBN_Set_Status_Flag(PDS_SOC_STATUS_ENTER_FLAG);

	pds_gpio_set_pulls();
	pds_gpio_wakeup_clear();

	PDS_Default_Level_Config(bflb_pds_level31_cfg, bflb_pds_sleep_cycles_32k);

	__asm__ volatile("wfi");

	/* Abort: undo PDS hardware state and restore flash XIP.
	 * pm_state_set_s2ram handles the rest (GPIO, CLIC, clocks).
	 */
	sys_write32(PDS_CTL_ABORT_VALUE, PDS_BASE + PDS_CTL_OFFSET);
	HBN_Set_Status_Flag(0U);
	flash_bflb_pm_resume();

	return -EBUSY;
}

void __attribute__((naked, used)) pds_fastboot_entry(void)
{
	__asm__ volatile(
		/* Clear fastboot flag so a crash won't loop back here. */
		"li	t0, %0\n\t"
		"sw	zero, 0(t0)\n\t"

		/* Enable FPU. */
		"li	t0, %1\n\t"
		"csrs	mstatus, t0\n\t"
		"fssr	x0\n\t"

#ifdef PDS_SOC_FASTBOOT_NEEDS_GP
		".option push\n\t"
		".option norelax\n\t"
		"la	gp, __global_pointer$\n\t"
		".option pop\n\t"
#endif

		/* Borrow the ISR stack. */
		"la	sp, z_interrupt_stacks\n\t"
		"li	t0, %2\n\t"
		"add	sp, sp, t0\n\t"

		"call	pds_restore\n\t"
		:
		: "i"(PDS_SOC_HBN_RSV0_ADDR), "i"(MSTATUS_FS_DIRTY), "i"(CONFIG_ISR_STACK_SIZE));
}

static void pds_restore_rf_and_xtal(void)
{
	volatile uint32_t *rf_top = (volatile uint32_t *)(AON_BASE + AON_RF_TOP_AON_OFFSET);
	volatile uint32_t *pds_ctl = (volatile uint32_t *)(PDS_BASE + PDS_CTL_OFFSET);

	*rf_top &= ~AON_RF_TOP_POWER_MASK;
	*rf_top |= AON_RF_TOP_XTAL_PU;
	*pds_ctl &= ~BIT(PDS_CR_PDS_GPIO_ISO_MODE_POS);
}

/* Restore flash XIP, then resume CPU context into pm_state_set_s2ram(). */
void __used pds_restore(void)
{
	HBN_Set_Status_Flag(0U);
	pds_restore_rf_and_xtal();
	flash_bflb_pm_resume();

	(void)arch_pm_s2ram_resume();

	CODE_UNREACHABLE;
}
