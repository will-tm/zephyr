/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_pds

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/common/pm_s2ram.h>
#include <clic.h>
#include <clic_pm.h>
#include <bflb_soc.h>
#include <glb_reg.h>
#include <pds_reg.h>
#include <hbn_reg.h>

#include "bflb_pds.h"
#include "bl70xl_rom_api.h"
#include <pds_bflb_time.h>

#define MTIMECMP_BASE DT_REG_ADDR_BY_NAME(DT_NODELABEL(mtimer), mtimecmp)
#define MTIMECMP_LO   (MTIMECMP_BASE)
#define MTIMECMP_HI   (MTIMECMP_BASE + 4U)

#if defined(CONFIG_BT_BFLB_BL70XL)
#include <bouffalolab/bl70xl/ble/btble_lib_api.h>
extern void btble_pds_enable(uint8_t enable);
extern void btble_pds_sleep_cycle(uint32_t expected_idle_ms);
#endif

#define PDS_INT_WAKEUP_SRC_HBN BIT(PDS_CR_PDS_WAKEUP_SRC_EN_POS)

#define PDS_WAKEUP_GPIO_COUNT DT_INST_PROP_LEN_OR(0, wakeup_gpios, 0)
#define PDS_WAKEUP_GPIO_PIN(idx, _) DT_GPIO_PIN_BY_IDX(DT_DRV_INST(0), wakeup_gpios, idx)

#if PDS_WAKEUP_GPIO_COUNT > 0
static const uint8_t pds_wakeup_pins[] = {
	LISTIFY(PDS_WAKEUP_GPIO_COUNT, PDS_WAKEUP_GPIO_PIN, (,))
};
#endif

#define GLB_EM_SEL_WIDTH 4U
#define GLB_EM_SEL_MASK  BIT_MASK(GLB_EM_SEL_WIDTH)

#define GLB_ROOT_CLK_SEL_WIDTH 2U
#define GLB_PLL_SEL_WIDTH      2U
#define GLB_CLK_DIV_WIDTH      8U
#define GLB_SYS_CLK_DLL_BASE   2U

#define AHB_MCU_SW_SEC_ENG 36U

#define LDO11_SOFT_START_DELAY 2U
#define MTIME_COMPARE_MAX      UINT32_MAX
#define PDS_INIT_PRIORITY      90
#define PDS_MTIMER_CLK_DIV     31U

struct pds_clock_snapshot {
	uint32_t cgen_cfg1;
	uint32_t clk_cfg0;
	uint32_t clk_cfg2;
	uint32_t cpu_clk_cfg;
	uint8_t sysclk;
};

struct pds_state {
	uint32_t wakeup_src;
	uint8_t em_sel;
	bool initialized;
};

static struct pds_state pds_state;

uint32_t bflb_pds_wakeup_pins;

void pds_gpio_wakeup_cfg(uint8_t pin)
{
	uint32_t group;
	uint32_t tmp;

	if (PDS_PIN_IS_HBN(pin)) {
		uint16_t hbn_mask;

		if (pin >= 9U && pin <= 13U) {
			hbn_mask = PDS_HBN_WAKEUP_GPIO_9 << (pin - 9U);
		} else {
			hbn_mask = PDS_HBN_WAKEUP_GPIO_9 << (5U + pin - 30U);
		}
		hbn_mask |= PDS_HBN_WORKAROUND_MASK;
		HBN_GPIO_Wakeup_Set(hbn_mask, PDS_HBN_GPIO_TRIG_ASYNC_FALLING);
		return;
	}

	group = PDS_GPIO_GROUP(pin);

	/* Set edge mode for wake group (both edges) */
	tmp = sys_read32(PDS_BASE + PDS_GPIO_INT_SET_OFFSET);
	tmp &= ~(BIT_MASK(PDS_GPIO_GROUP_BITS) << (group * PDS_GPIO_GROUP_BITS));
	tmp |= (PDS_GPIO_INT_BOTH_EDGE << (group * PDS_GPIO_GROUP_BITS));
	sys_write32(tmp, PDS_BASE + PDS_GPIO_INT_SET_OFFSET);

	/* Unmask wake group interrupt */
	tmp = sys_read32(PDS_BASE + PDS_CFG_PDS_KEY_SCAN_OFFSET);
	tmp &= ~BIT(PDS_CR_PDS_GPIO_SET_INT_MASK_POS + group);
	sys_write32(tmp, PDS_BASE + PDS_CFG_PDS_KEY_SCAN_OFFSET);

	/* Enable input for wake pin */
	tmp = sys_read32(PDS_BASE + PDS_GPIO_IE_SET_OFFSET);
	tmp |= BIT(pin);
	sys_write32(tmp, PDS_BASE + PDS_GPIO_IE_SET_OFFSET);

	/* Enable PDS IO wake source */
	pds_state.wakeup_src |= PDS_INT_WAKEUP_SRC_PDS_IO;
	sys_write32(pds_state.wakeup_src, PDS_BASE + PDS_INT_OFFSET);
}

void pds_gpio_wakeup_clear(void)
{
	uint32_t tmp;

	/* Pulse the per-group int-clear bits: assert then release, so the GPIO
	 * status latch is cleared but free to capture the next wake edge during
	 * sleep (leaving the bits asserted holds STAT at zero).
	 */
	tmp = sys_read32(PDS_BASE + PDS_CFG_PDS_KEY_SCAN_OFFSET);
	sys_write32(tmp | PDS_GPIO_INT_CLR_ALL, PDS_BASE + PDS_CFG_PDS_KEY_SCAN_OFFSET);
	sys_write32(tmp & ~PDS_GPIO_INT_CLR_ALL, PDS_BASE + PDS_CFG_PDS_KEY_SCAN_OFFSET);
}

uint32_t clic_intie_backup[BFLB_CLIC_PM_BACKUP_WORDS];
uint32_t clic_intcfg_backup[BFLB_CLIC_PM_BACKUP_WORDS];
uint32_t bflb_pds_sleep_cycles_32k;

static void pds_clock_save(struct pds_clock_snapshot *snap)
{
	uint8_t root_sel;
	uint8_t pll_sel;

	snap->cgen_cfg1 = sys_read32(GLB_BASE + GLB_CGEN_CFG1_OFFSET);
	snap->clk_cfg2 = sys_read32(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	snap->cpu_clk_cfg = sys_read32(GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);
	snap->clk_cfg0 = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);

	root_sel = (uint8_t)((snap->clk_cfg0 >> GLB_HBN_ROOT_CLK_SEL_POS) &
			     BIT_MASK(GLB_ROOT_CLK_SEL_WIDTH));
	pll_sel = (uint8_t)((snap->clk_cfg0 >> GLB_REG_PLL_SEL_POS) &
			    BIT_MASK(GLB_PLL_SEL_WIDTH));

	snap->sysclk = (root_sel >= GLB_SYS_CLK_DLL_BASE)
			       ? (uint8_t)(GLB_SYS_CLK_DLL_BASE + pll_sel)
			       : root_sel;
}

static void pds_clock_restore_root(const struct pds_clock_snapshot *snap)
{
	uint8_t hclk_div;
	uint8_t bclk_div;

	AON_Power_On_XTAL();
	HBN_Set_ROOT_CLK_Sel(1U);

	GLB_Set_System_CLK(PDS_SOC_XTAL_TYPE, snap->sysclk);

	hclk_div =
		(uint8_t)((snap->clk_cfg0 >> GLB_REG_HCLK_DIV_POS) & BIT_MASK(GLB_CLK_DIV_WIDTH));
	bclk_div =
		(uint8_t)((snap->clk_cfg0 >> GLB_REG_BCLK_DIV_POS) & BIT_MASK(GLB_CLK_DIV_WIDTH));
	GLB_Set_System_CLK_Div(hclk_div, bclk_div);
}

static void pds_gpio_clear_pulls(void)
{
	uint32_t tmp;

	sys_write32(0U, PDS_BASE + PDS_GPIO_PU_SET_OFFSET);
	sys_write32(0U, PDS_BASE + PDS_GPIO_PD_SET_OFFSET);

	tmp = sys_read32(HBN_BASE + HBN_PAD_CTRL_1_OFFSET);
	tmp &= ~(HBN_REG_AON_GPIO_PU_MSK | HBN_REG_AON_GPIO_PD_MSK);
	sys_write32(tmp, HBN_BASE + HBN_PAD_CTRL_1_OFFSET);

	tmp = sys_read32(HBN_BASE + HBN_PAD_CTRL_0_OFFSET);
	tmp &= ~HBN_REG_EN_AON_CTRL_GPIO_MSK;
	sys_write32(tmp, HBN_BASE + HBN_PAD_CTRL_0_OFFSET);
}

static int pds_init(const struct device *dev)
{
	uint32_t hbn_glb;
	uint32_t f32k_sel;
	uint32_t seam_misc;
	const struct device *devs;
	size_t dev_count;

	ARG_UNUSED(dev);

	hbn_glb = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	f32k_sel = (hbn_glb >> HBN_F32K_SEL_POS) & BIT_MASK(HBN_F32K_SEL_LEN);
	if (f32k_sel != 0U) {
		hbn_glb &= ~(BIT_MASK(HBN_F32K_SEL_LEN) << HBN_F32K_SEL_POS);
		sys_write32(hbn_glb, HBN_BASE + HBN_GLB_OFFSET);
	}

	AON_Set_LDO11_SOC_Sstart_Delay(LDO11_SOFT_START_DELAY);

	PDS_IntClear();
	pds_state.wakeup_src = PDS_INT_WAKEUP_SRC_HBN;
	sys_write32(pds_state.wakeup_src, PDS_BASE + PDS_INT_OFFSET);

	/* Apply factory RC32K trim so the HBN RTC runs near 32768 Hz. */
	(void)HBN_Trim_RC32K();

	HBN_Enable_RTC_Counter();

#if defined(CONFIG_BT_BFLB_BL70XL)
	bl_rtc_frequency = HBN_RTC_FREQ;
#endif

	seam_misc = sys_read32(GLB_BASE + GLB_SEAM_MISC_OFFSET);
	pds_state.em_sel = (uint8_t)((seam_misc >> GLB_EM_SEL_POS) & GLB_EM_SEL_MASK);

	HBN_Set_Wakeup_Addr((uint32_t)pds_fastboot_entry);

#ifdef CONFIG_BL70XL_PDS_S2RAM
	pds_flash_init();
#endif

#if PDS_WAKEUP_GPIO_COUNT > 0
	for (uint32_t i = 0U; i < ARRAY_SIZE(pds_wakeup_pins); i++) {
		pds_gpio_wakeup_cfg(pds_wakeup_pins[i]);
	}
#endif

	/* Keep wakeup-source devices out of system suspend so their state (and
	 * the GPIO wake callback re-fired on resume) survives PDS.
	 */
	dev_count = z_device_get_all_static(&devs);
	for (size_t i = 0U; i < dev_count; i++) {
		(void)pm_device_wakeup_enable(&devs[i], true);
	}

	pds_state.initialized = true;
	return 0;
}

DEVICE_DT_INST_DEFINE(0, pds_init, NULL, NULL, NULL, POST_KERNEL, PDS_INIT_PRIORITY, NULL);

static void pm_state_set_light(void)
{
	__asm__ volatile("wfi");
}

#ifdef CONFIG_BL70XL_PDS_S2RAM

void bflb_pds_enter_s2ram(uint32_t sleep_cycles)
{
	struct pds_clock_snapshot snap;
	uint64_t rtc_before;
	uint64_t mtime_before;
	uint32_t hbn_irq;
	uint32_t hbn_gpio;
	uint32_t wake;
	uint32_t tmp;

	pds_clock_save(&snap);

	rtc_before = bflb_pds_hbn_rtc_read();
	mtime_before = bflb_pds_mtime_read();

	bflb_pds_sleep_cycles_32k = sleep_cycles;
	(void)arch_pm_s2ram_suspend(bflb_pds_system_off);

	hbn_irq = sys_read32(HBN_BASE + HBN_IRQ_STAT_OFFSET);

	/* Post-wake hardware restore */
	pds_gpio_clear_pulls();
	sys_write32(HBN_SRAM_ACTIVE, HBN_BASE + HBN_SRAM_OFFSET);

	tmp = sys_read32(GLB_BASE + GLB_SEAM_MISC_OFFSET);
	tmp &= ~((uint32_t)GLB_EM_SEL_MASK << GLB_EM_SEL_POS);
	tmp |= ((uint32_t)pds_state.em_sel << GLB_EM_SEL_POS);
	sys_write32(tmp, GLB_BASE + GLB_SEAM_MISC_OFFSET);

	GLB_AHB_MCU_Software_Reset(AHB_MCU_SW_SEC_ENG);
	bflb_clic_pm_clear_all();

	PDS_IntClear();
	pds_gpio_wakeup_clear();

	sys_write32(UINT32_MAX, HBN_BASE + HBN_TIME_L_OFFSET);
	sys_write32(HBN_RTC_TIME_H_MASK, HBN_BASE + HBN_TIME_H_OFFSET);
	sys_write32(UINT32_MAX, HBN_BASE + HBN_IRQ_CLR_OFFSET);
	sys_write32(pds_state.wakeup_src, PDS_BASE + PDS_INT_OFFSET);
	bflb_clic_pm_restore(clic_intie_backup, clic_intcfg_backup);

	pds_clock_restore_root(&snap);
	sys_write32(snap.clk_cfg2, GLB_BASE + GLB_CLK_CFG2_OFFSET);
	sys_write32(snap.cgen_cfg1, GLB_BASE + GLB_CGEN_CFG1_OFFSET);
	sys_write32(snap.cpu_clk_cfg, GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);
	GLB_Set_MTimer_CLK(1U, 0U, PDS_MTIMER_CLK_DIV);

	bflb_pds_time_compensate(rtc_before, mtime_before);

	hbn_gpio = hbn_irq & PDS_HBN_IRQ_GPIO_MASK;
	wake = (hbn_gpio & BIT_MASK(HBN_PAD_LOW_WIDTH)) << HBN_PAD_LOW_BASE;
	wake |= (hbn_gpio >> HBN_PAD_HIGH_BIT) << HBN_PAD_HIGH_BASE;
	bflb_pds_wakeup_pins = wake;

	sys_write8(1U, CLIC_HART0_ADDR + CLIC_INTIE + PDS_SOC_IRQN_MTIMER);
}

static void pm_state_set_s2ram(void)
{
#if defined(CONFIG_BT_BFLB_BL70XL)
	static bool pds_ble_inited;
	uint64_t mtimecmp_saved;
	uint64_t mtime_now;
	int64_t remaining_cycles;
	uint32_t idle_ms;

	if (!pds_ble_inited) {
		btble_pds_enable(1);
		pds_ble_inited = true;
	}

	mtimecmp_saved =
		(uint64_t)sys_read32(MTIMECMP_LO) | ((uint64_t)sys_read32(MTIMECMP_HI) << 32);

	mtime_now = bflb_pds_mtime_read();
	remaining_cycles = (int64_t)(mtimecmp_saved - mtime_now);
	if (remaining_cycles <= 0) {
		return;
	}
	idle_ms = (uint32_t)((uint64_t)remaining_cycles * 1000U / sys_clock_hw_cycles_per_sec());

	irq_unlock(MSTATUS_IEN);
	btble_pds_sleep_cycle(idle_ms);
	irq_lock();

	sys_write32(UINT32_MAX, MTIMECMP_LO);
	sys_write32((uint32_t)(mtimecmp_saved >> 32), MTIMECMP_HI);
	sys_write32((uint32_t)mtimecmp_saved, MTIMECMP_LO);
#else
	uint64_t now;
	uint32_t sleep_32k;

	sleep_32k = bflb_pds_calc_sleep_ticks();
	if (sleep_32k == 0U) {
		__asm__ volatile("wfi");
		return;
	}

	bflb_pds_enter_s2ram(sleep_32k);

	/* Force mtimer ISR so the kernel learns elapsed time after wake. */
	now = bflb_pds_mtime_read();

	sys_write32(MTIME_COMPARE_MAX, MTIMECMP_LO);
	sys_write32((uint32_t)(now >> 32U), MTIMECMP_HI);
	sys_write32((uint32_t)now, MTIMECMP_LO);
#endif
}
#endif /* CONFIG_BL70XL_PDS_S2RAM */

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	if (!pds_state.initialized) {
		return;
	}

	switch (state) {
#ifdef CONFIG_BL70XL_PDS_S2RAM
	case PM_STATE_SUSPEND_TO_RAM:
		pm_state_set_s2ram();
		break;
#endif
	case PM_STATE_SUSPEND_TO_IDLE:
	default:
		pm_state_set_light();
		break;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);

	irq_unlock(MSTATUS_IEN);
}
