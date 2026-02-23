/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_bl808x_clock_controller

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/clock/bflb_bl808x_clock.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_bl808x, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

#include <bouffalolab/bl808x/bflb_soc.h>
#include <bouffalolab/bl808x/aon_reg.h>
#include <bouffalolab/bl808x/glb_reg.h>
#include <bouffalolab/bl808x/hbn_reg.h>
#include <bouffalolab/bl808x/mcu_misc_reg.h>
#include <bouffalolab/bl808x/pds_reg.h>
#include <bouffalolab/bl808x/sf_ctrl_reg.h>
#include <zephyr/drivers/clock_control/clock_control_bflb_common.h>
#include "clock_control_bl808x_pll.h"

/*
 * CLK_SRC_IS(clk, src): true if the named clock 'clk' in the clock controller's
 * clocks list is sourced from the same clock provider node as 'src'.
 */
#define CLK_SRC_IS(clk, src)                                                                       \
	DT_SAME_NODE(DT_CLOCKS_CTLR_BY_IDX(DT_INST_CLOCKS_CTLR_BY_NAME(0, clk), 0),                \
		     DT_INST_CLOCKS_CTLR_BY_NAME(0, src))

#define CLOCK_TIMEOUT        1024 /* polling iterations */
/* CRYSTAL_VALUES_CNT defined in clock_control_bl808x_pll.h */
#define ROOT_CLK_RANGE_DELIM DT_FREQ_M(480)

/*
 * MCU_E907_RTC register CLK_SEL field (bit 29).
 * Selects mtimer clock source: 0 = XCLK, 1 = FCLK.
 * Not defined in the vendor header for BL808.
 */
#define MCU_MISC_REG_MCU_RTC_CLK_SEL_POS  (29U)
#define MCU_MISC_REG_MCU_RTC_CLK_SEL_MSK  (1U << MCU_MISC_REG_MCU_RTC_CLK_SEL_POS)
#define MCU_MISC_REG_MCU_RTC_CLK_SEL_UMSK (~MCU_MISC_REG_MCU_RTC_CLK_SEL_MSK)

/* MCU_E907_RTC DIV field is 10 bits wide */
#define MCU_MISC_MCU_RTC_DIV_MAX 0x3FFU

#define EFUSE_RC32M_TRIM_OFFSET        0x7C
#define EFUSE_RC32M_TRIM_EP_OFFSET     0x78
#define EFUSE_RC32M_TRIM_EP_EN_POS     1
#define EFUSE_RC32M_TRIM_EP_PARITY_POS 0
#define EFUSE_RC32M_TRIM_POS           4
#define EFUSE_RC32M_TRIM_MSK           0xFF0
/* crystal_id values - index into PLL config arrays */
#define CRYSTAL_ID_FREQ_32000000       0
#define CRYSTAL_ID_FREQ_24000000       1
#define CRYSTAL_ID_FREQ_38400000       2
#define CRYSTAL_ID_FREQ_40000000       3
#define CRYSTAL_ID_FREQ_26000000       4

#define CRYSTAL_FREQ_TO_ID(freq) CONCAT(CRYSTAL_ID_FREQ_, freq)

/*
 * CLK_AT_LEAST_MUL: minimum CPU cycles per microsecond, used for busy-wait loops.
 * Scales with PLL top_frequency from DTS via BFLB_MUL_CLK.
 */
#if CLK_SRC_IS(root, wifipll_top)
#define CLK_AT_LEAST_MUL                                                                           \
	BFLB_MUL_CLK(32, DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, wifipll_top), top_frequency),      \
		     BL808X_WIFIPLL_TOP_FREQ)
#else
#define CLK_AT_LEAST_MUL 32
#endif

/*
 * HBN_RSV3 crystal type flag (Bouffalo SDK).
 * Bits [7:0]  = GLB_XTAL_* enum value
 * Bits [15:8] = 0x58 validity marker
 */
#define HBN_XTAL_FLAG_VALUE 0x5800U
#define HBN_XTAL_FLAG_MASK  0x0000ff00U

enum bl808x_clkid {
	bl808x_clkid_clk_root = BL808X_CLKID_CLK_ROOT,
	bl808x_clkid_clk_rc32m = BL808X_CLKID_CLK_RC32M,
	bl808x_clkid_clk_crystal = BL808X_CLKID_CLK_CRYSTAL,
	bl808x_clkid_clk_bclk = BL808X_CLKID_CLK_BCLK,
	bl808x_clkid_clk_f32k = BL808X_CLKID_CLK_F32K,
	bl808x_clkid_clk_xtal32k = BL808X_CLKID_CLK_XTAL32K,
	bl808x_clkid_clk_rc32k = BL808X_CLKID_CLK_RC32K,
	bl808x_clkid_clk_wifipll = BL808X_CLKID_CLK_WIFIPLL,
	bl808x_clkid_clk_aupll = BL808X_CLKID_CLK_AUPLL,
	bl808x_clkid_clk_160mux = BL808X_CLKID_CLK_160M,
};

struct clock_control_bl808x_pll_config {
	enum bl808x_clkid source;
	uint32_t top_frequency;
	bool enabled;
};

struct clock_control_bl808x_root_config {
	enum bl808x_clkid source;
	uint8_t pll_select;
	uint8_t divider;
};

struct clock_control_bl808x_bclk_config {
	uint8_t divider;
};

struct clock_control_bl808x_f32k_config {
	enum bl808x_clkid source;
	bool xtal_enabled;
};

struct clock_control_bl808x_config {
	uint32_t crystal_id;
};

struct clock_control_bl808x_data {
	bool crystal_enabled;
	struct clock_control_bl808x_pll_config wifipll;
	struct clock_control_bl808x_pll_config aupll;
	struct clock_control_bl808x_root_config root;
	struct clock_control_bl808x_bclk_config bclk;
	struct clock_control_bl808x_f32k_config f32k;
};

static void clock_control_bl808x_clock_at_least_us(uint32_t us)
{
	for (uint32_t i = 0; i < us * CLK_AT_LEAST_MUL; i++) {
		clock_bflb_settle();
	}
}

static int clock_control_bl808x_deinit_crystal(void)
{
	uint32_t tmp;

	tmp = sys_read32(AON_BASE + AON_RF_TOP_AON_OFFSET);
	tmp &= AON_PU_XTAL_AON_UMSK;
	tmp &= AON_PU_XTAL_BUF_AON_UMSK;
	sys_write32(tmp, AON_BASE + AON_RF_TOP_AON_OFFSET);
	clock_bflb_settle();
	return 0;
}

static int clock_control_bl808x_init_crystal(void)
{
	uint32_t tmp;
	int count = CLOCK_TIMEOUT;

	/* power crystal */
	tmp = sys_read32(AON_BASE + AON_RF_TOP_AON_OFFSET);
	tmp = (tmp & AON_PU_XTAL_AON_UMSK) | (1U << AON_PU_XTAL_AON_POS);
	tmp = (tmp & AON_PU_XTAL_BUF_AON_UMSK) | (1U << AON_PU_XTAL_BUF_AON_POS);
	sys_write32(tmp, AON_BASE + AON_RF_TOP_AON_OFFSET);

	/* wait for crystal to be powered on */
	do {
		clock_bflb_settle();
		tmp = sys_read32(AON_BASE + AON_TSEN_OFFSET);
		count--;
	} while (!(tmp & AON_XTAL_RDY_MSK) && count > 0);

	clock_bflb_settle();

	if (count < 1) {
		return -ETIMEDOUT;
	}
	return 0;
}

/*
 * Update HCLK and BCLK dividers using the hardware BCLK protection handshake.
 * Must be called with root clock on RC32M or XCLK to be safe.
 */
static int clock_bflb_set_root_clock_dividers(uint32_t hclk_div, uint32_t bclk_div)
{
	uint32_t tmp;
	uint32_t old_rootclk;
	int count = CLOCK_TIMEOUT;

	old_rootclk = clock_bflb_get_root_clock();

	/* Must drop to XCLK/RC32M before changing dividers if running from PLL */
	if (old_rootclk > BFLB_MAIN_CLOCK_XTAL) {
		clock_bflb_set_root_clock(BFLB_MAIN_CLOCK_RC32M);
	}

	tmp = sys_read32(GLB_BASE + GLB_SYS_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_HCLK_DIV_UMSK) | (hclk_div << GLB_REG_HCLK_DIV_POS);
	tmp = (tmp & GLB_REG_BCLK_DIV_UMSK) | (bclk_div << GLB_REG_BCLK_DIV_POS);
	sys_write32(tmp, GLB_BASE + GLB_SYS_CFG0_OFFSET);

	/* Trigger divider update and wait for BCLK protection to confirm */
	tmp = sys_read32(GLB_BASE + GLB_SYS_CFG1_OFFSET);
	tmp = (tmp & GLB_REG_BCLK_DIV_ACT_PULSE_UMSK) | (1U << GLB_REG_BCLK_DIV_ACT_PULSE_POS);
	sys_write32(tmp, GLB_BASE + GLB_SYS_CFG1_OFFSET);

	do {
		tmp = sys_read32(GLB_BASE + GLB_SYS_CFG1_OFFSET);
		tmp = (tmp & GLB_STS_BCLK_PROT_DONE_MSK) >> GLB_STS_BCLK_PROT_DONE_POS;
		count--;
	} while (count > 0 && tmp == 0);

	clock_bflb_set_root_clock(old_rootclk);
	clock_bflb_settle();

	if (count < 1) {
		return -EIO;
	}
	return 0;
}

/*
 * Configure the machine timer clock.
 * BL808 M0 core mtimer is clocked from FCLK (core clock), not XCLK.
 * SDK: CPU_Set_MTimer_CLK(ENABLE, CPU_Get_MTimer_Source_Clock() / 1e6 - 1)
 * divider: fclk_Hz / 1000000 - 1  -> gives 1 MHz output
 */
static void clock_control_bl808x_set_machine_timer_clock_enable(bool enable)
{
	uint32_t tmp;

	tmp = sys_read32(MCU_MISC_BASE + MCU_MISC_MCU_E907_RTC_OFFSET);
	if (enable) {
		tmp = (tmp & MCU_MISC_REG_MCU_RTC_EN_UMSK) | (1U << MCU_MISC_REG_MCU_RTC_EN_POS);
	} else {
		tmp &= MCU_MISC_REG_MCU_RTC_EN_UMSK;
	}
	sys_write32(tmp, MCU_MISC_BASE + MCU_MISC_MCU_E907_RTC_OFFSET);
}

static void clock_control_bl808x_deinit_wifipll(void)
{
	uint32_t tmp;

	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmp &= GLB_PU_WIFIPLL_UMSK;
	tmp &= GLB_PU_WIFIPLL_SFREG_UMSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
}

/* source: 0 = RC32M, 1 = crystal */
static void clock_control_bl808x_set_wifipll_source(uint32_t source)
{
	uint32_t tmp;

	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG1_OFFSET);
	if (source == 1) {
		tmp = (tmp & GLB_WIFIPLL_REFCLK_SEL_UMSK) | (1U << GLB_WIFIPLL_REFCLK_SEL_POS);
	} else {
		tmp = (tmp & GLB_WIFIPLL_REFCLK_SEL_UMSK) | (3U << GLB_WIFIPLL_REFCLK_SEL_POS);
	}
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG1_OFFSET);
}

/*
 * WiFi PLL (WAC PLL) setup for BL808.
 *
 * PLL must be powered down (deinit) before calling this function.
 * Takes pre-extracted config field values (not a struct pointer) so that
 * no data/struct access occurs during the PLL-off window. Register reads
 * happen here (post-deinit) to get correct base values.
 */
static void clock_control_bl808x_init_wifipll_setup(uint32_t refdiv_ratio, uint32_t int_frac_sw,
						    uint32_t icp_1u, uint32_t icp_5u, uint32_t rz,
						    uint32_t cz, uint32_t c3, uint32_t r4_short,
						    uint32_t c4_en, uint32_t sel_sample_clk,
						    uint32_t vco_speed, uint32_t sdm_ctrl_hw,
						    uint32_t sdm_bypass, uint32_t sdmin)
{
	uint32_t tmp;

	/* CFG1 */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG1_OFFSET);
	tmp = (tmp & GLB_WIFIPLL_REFDIV_RATIO_UMSK) |
	      (refdiv_ratio << GLB_WIFIPLL_REFDIV_RATIO_POS);
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG1_OFFSET);

	/* CFG2 */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG2_OFFSET);
	tmp = (tmp & GLB_WIFIPLL_INT_FRAC_SW_UMSK) | (int_frac_sw << GLB_WIFIPLL_INT_FRAC_SW_POS);
	tmp = (tmp & GLB_WIFIPLL_ICP_1U_UMSK) | (icp_1u << GLB_WIFIPLL_ICP_1U_POS);
	tmp = (tmp & GLB_WIFIPLL_ICP_5U_UMSK) | (icp_5u << GLB_WIFIPLL_ICP_5U_POS);
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG2_OFFSET);

	/* CFG3 */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG3_OFFSET);
	tmp = (tmp & GLB_WIFIPLL_RZ_UMSK) | (rz << GLB_WIFIPLL_RZ_POS);
	tmp = (tmp & GLB_WIFIPLL_CZ_UMSK) | (cz << GLB_WIFIPLL_CZ_POS);
	tmp = (tmp & GLB_WIFIPLL_C3_UMSK) | (c3 << GLB_WIFIPLL_C3_POS);
	tmp = (tmp & GLB_WIFIPLL_R4_SHORT_UMSK) | (r4_short << GLB_WIFIPLL_R4_SHORT_POS);
	tmp = (tmp & GLB_WIFIPLL_C4_EN_UMSK) | (c4_en << GLB_WIFIPLL_C4_EN_POS);
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG3_OFFSET);

	/* CFG4 */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG4_OFFSET);
	tmp = (tmp & GLB_WIFIPLL_SEL_SAMPLE_CLK_UMSK) |
	      (sel_sample_clk << GLB_WIFIPLL_SEL_SAMPLE_CLK_POS);
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG4_OFFSET);

	/* CFG5 */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG5_OFFSET);
	tmp = (tmp & GLB_WIFIPLL_VCO_SPEED_UMSK) | (vco_speed << GLB_WIFIPLL_VCO_SPEED_POS);
	tmp |= GLB_WIFIPLL_VCO_DIV3_EN_MSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG5_OFFSET);

	/* CFG6 */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG6_OFFSET);
	tmp = (tmp & GLB_WIFIPLL_SDM_CTRL_HW_UMSK) | (sdm_ctrl_hw << GLB_WIFIPLL_SDM_CTRL_HW_POS);
	tmp = (tmp & GLB_WIFIPLL_SDM_BYPASS_UMSK) | (sdm_bypass << GLB_WIFIPLL_SDM_BYPASS_POS);
	tmp = (tmp & GLB_WIFIPLL_SDMIN_UMSK) | (sdmin << GLB_WIFIPLL_SDMIN_POS);
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG6_OFFSET);

	/* Power on PLL switching regulator (SFREG) — provides clean analog
	 * supply to the PLL core. Must come before PU_WIFIPLL. */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmp = (tmp & GLB_PU_WIFIPLL_SFREG_UMSK) | (1U << GLB_PU_WIFIPLL_SFREG_POS);
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);

	clock_control_bl808x_clock_at_least_us(8);

	/* Power on the PLL itself (VCO, charge pump, dividers) */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmp = (tmp & GLB_PU_WIFIPLL_UMSK) | (1U << GLB_PU_WIFIPLL_POS);
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);

	clock_control_bl808x_clock_at_least_us(8);

	/* SDM reset toggle (1→0→1): resets the sigma-delta modulator that
	 * generates the fractional-N divider control. Clears any accumulated
	 * quantization error from a previous configuration. */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmp = (tmp & GLB_WIFIPLL_SDM_RSTB_UMSK) | (1U << GLB_WIFIPLL_SDM_RSTB_POS);
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	clock_control_bl808x_clock_at_least_us(8);
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmp &= GLB_WIFIPLL_SDM_RSTB_UMSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	clock_control_bl808x_clock_at_least_us(8);
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmp = (tmp & GLB_WIFIPLL_SDM_RSTB_UMSK) | (1U << GLB_WIFIPLL_SDM_RSTB_POS);
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);

	/* FBDV reset toggle (1→0→1): resets the feedback divider that closes
	 * the PLL loop. Forces the divider to a known state so the PLL can
	 * reacquire lock cleanly. */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmp = (tmp & GLB_WIFIPLL_FBDV_RSTB_UMSK) | (1U << GLB_WIFIPLL_FBDV_RSTB_POS);
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	clock_control_bl808x_clock_at_least_us(8);
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmp &= GLB_WIFIPLL_FBDV_RSTB_UMSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	clock_control_bl808x_clock_at_least_us(8);
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmp = (tmp & GLB_WIFIPLL_FBDV_RSTB_UMSK) | (1U << GLB_WIFIPLL_FBDV_RSTB_POS);
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);

	/* Enable PLL output clock dividers.
	 * BL808 SDK GLB_Power_On_WAC_PLL enables:
	 *   DIV4  = 960/4  = 240MHz (WIFIPLL_240M)
	 *   DIV5  = 960/5  = 192MHz (WIFIPLL_192M)
	 *   DIV6  = 960/6  = 160MHz (WIFIPLL_160M)
	 *   DIV8  = 960/8  = 120MHz (WIFIPLL_120M)
	 *   DIV10 = 960/10 =  96MHz (WIFIPLL_96M)
	 * EN_CTRL_HW lets hardware gate unused outputs automatically.
	 * BL808x CFG8 has no EN_DIV3 (BL61x-only). */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG8_OFFSET);
	tmp = (tmp & GLB_WIFIPLL_EN_DIV4_UMSK) | (1U << GLB_WIFIPLL_EN_DIV4_POS);
	tmp = (tmp & GLB_WIFIPLL_EN_DIV5_UMSK) | (1U << GLB_WIFIPLL_EN_DIV5_POS);
	tmp = (tmp & GLB_WIFIPLL_EN_DIV6_UMSK) | (1U << GLB_WIFIPLL_EN_DIV6_POS);
	tmp = (tmp & GLB_WIFIPLL_EN_DIV8_UMSK) | (1U << GLB_WIFIPLL_EN_DIV8_POS);
	tmp = (tmp & GLB_WIFIPLL_EN_DIV10_UMSK) | (1U << GLB_WIFIPLL_EN_DIV10_POS);
	tmp = (tmp & GLB_WIFIPLL_EN_CTRL_HW_UMSK) | (1U << GLB_WIFIPLL_EN_CTRL_HW_POS);
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG8_OFFSET);

	/* Wait for PLL to lock (~50µs per SDK) */
	clock_control_bl808x_clock_at_least_us(50);
}

/* cfg MUST point to SRAM (stack copy), not XIP flash — see update_clocks(). */
static void clock_control_bl808x_init_wifipll(const bl808x_pll_config *cfg,
					      enum bl808x_clkid source)
{
	uint32_t tmp;
	uint32_t old_rootclk;

	old_rootclk = clock_bflb_get_root_clock();

	/* Must drop to XCLK/RC32M before reconfiguring PLL */
	if (old_rootclk > BFLB_MAIN_CLOCK_XTAL) {
		clock_bflb_set_root_clock(BFLB_MAIN_CLOCK_RC32M);
	}

	clock_control_bl808x_deinit_wifipll();

	if (source == bl808x_clkid_clk_crystal) {
		clock_control_bl808x_set_wifipll_source(1);
	} else {
		clock_control_bl808x_set_wifipll_source(0);
	}

	clock_control_bl808x_init_wifipll_setup(
		cfg->pllRefdivRatio, cfg->pllIntFracSw, cfg->pllIcp1u, cfg->pllIcp5u, cfg->pllRz,
		cfg->pllCz, cfg->pllC3, cfg->pllR4Short, cfg->pllC4En, cfg->pllSelSampleClk,
		cfg->pllVcoSpeed, cfg->pllSdmCtrlHw, cfg->pllSdmBypass, cfg->pllSdmin);

	/* enable PLL clock */
	tmp = sys_read32(GLB_BASE + GLB_SYS_CFG0_OFFSET);
	tmp |= GLB_REG_PLL_EN_MSK;
	sys_write32(tmp, GLB_BASE + GLB_SYS_CFG0_OFFSET);

	clock_bflb_set_root_clock(old_rootclk);
	clock_bflb_settle();
}

/*
 * Ungate a PLL output clock in GLB_CGEN_CFG3.
 * Bit positions (from glb_reg.h GLB_CGEN_* defines):
 *   2=ISP_WIFIPLL_80M  3=ISP_AUPLL_DIV5   4=ISP_AUPLL_DIV6
 *   5=TOP_AUPLL_DIV5   6=TOP_AUPLL_DIV6   7=EMI_WIFIPLL_320M
 *   8=EMI_AUPLL_DIV1  13=TOP_WIFIPLL_240M 14=TOP_WIFIPLL_320M
 *  15=TOP_AUPLL_DIV2  16=TOP_AUPLL_DIV1
 */
static void clock_control_bl808x_ungate_pll(uint8_t pll)
{
	uint32_t tmp;

	tmp = sys_read32(GLB_BASE + GLB_CGEN_CFG3_OFFSET);
	tmp |= (1U << pll);
	sys_write32(tmp, GLB_BASE + GLB_CGEN_CFG3_OFFSET);
}

/* Setup WiFi PLL (separate from root mux selection).
 * cfg points to a SRAM copy of the PLL config (not XIP flash) — see
 * update_clocks() for why flash reads are forbidden after RC32M switch. */
static void clock_control_bl808x_setup_wifipll(const struct device *dev,
					       const bl808x_pll_config *cfg)
{
	struct clock_control_bl808x_data *data = dev->data;

	clock_control_bl808x_init_wifipll(cfg, data->wifipll.source);

	clock_control_bl808x_ungate_pll(GLB_CGEN_TOP_WIFIPLL_320M_POS);
	clock_control_bl808x_ungate_pll(GLB_CGEN_TOP_WIFIPLL_240M_POS);
	clock_control_bl808x_ungate_pll(GLB_CGEN_EMI_WIFIPLL_320M_POS);
	clock_control_bl808x_ungate_pll(GLB_CGEN_MM_WIFIPLL_160M_POS);
	clock_control_bl808x_ungate_pll(GLB_CGEN_MM_WIFIPLL_240M_POS);
	clock_control_bl808x_ungate_pll(GLB_CGEN_MM_WIFIPLL_320M_POS);
}

/*
 * Select the PLL output mux for the CPU core clock (PDS_REG_PLL_SEL).
 *   0 = AUPLL DIV2,  1 = AUPLL DIV1,
 *   2 = WIFIPLL DIV3/4 (240M), 3 = WIFIPLL DIV1 (320M)
 */
static void clock_control_bl808x_select_pll(uint8_t pll)
{
	uint32_t tmp;

	tmp = sys_read32(PDS_BASE + PDS_CPU_CORE_CFG1_OFFSET);
	tmp = (tmp & PDS_REG_PLL_SEL_UMSK) | (pll << PDS_REG_PLL_SEL_POS);
	sys_write32(tmp, PDS_BASE + PDS_CPU_CORE_CFG1_OFFSET);
}

/* Gate (disable) a PLL output clock in GLB_CGEN_CFG3. */
static void clock_control_bl808x_gate_pll(uint8_t pll)
{
	uint32_t tmp;

	tmp = sys_read32(GLB_BASE + GLB_CGEN_CFG3_OFFSET);
	tmp &= ~(1U << pll);
	sys_write32(tmp, GLB_BASE + GLB_CGEN_CFG3_OFFSET);
}

/* Get XCLK frequency: RC32M when root_clk_sel[0]=0, crystal otherwise. */
static uint32_t clock_control_bl808x_get_xclk(const struct device *dev)
{
	uint32_t tmp;

	tmp = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmp = (tmp & HBN_ROOT_CLK_SEL_MSK) >> HBN_ROOT_CLK_SEL_POS;
	if ((tmp & 1U) == 0U) {
		return BFLB_RC32M_FREQUENCY;
	}
	return DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, crystal), clock_frequency);
}

/* Get FCLK (CPU core clock): XCLK when root_clk_sel[1]=0, PLL output otherwise. */
static uint32_t clock_control_bl808x_get_fclk(const struct device *dev)
{
	struct clock_control_bl808x_data *data = dev->data;
	uint32_t tmp;

	tmp = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmp &= HBN_ROOT_CLK_SEL_MSK;
	tmp = (tmp >> HBN_ROOT_CLK_SEL_POS) >> 1;
	tmp &= 1;

	if (tmp == 0) {
		return clock_control_bl808x_get_xclk(dev);
	}
	tmp = sys_read32(PDS_BASE + PDS_CPU_CORE_CFG1_OFFSET);
	tmp = (tmp & PDS_REG_PLL_SEL_MSK) >> PDS_REG_PLL_SEL_POS;
	if (tmp == BL808X_WIFIPLL_ID_DIV1) {
		return data->wifipll.top_frequency;
	} else if (tmp == BL808X_WIFIPLL_ID_DIV3_4) {
		return BFLB_MUL_CLK(data->wifipll.top_frequency, 3, 4);
	} else {
		/* AUPLL paths (DIV1/DIV2) — unreachable while BUILD_ASSERT
		 * prevents AUPLL from being enabled. Returns 0 as a safe
		 * fallback if AUPLL support is added later.
		 */
	}
	return 0;
}

static uint32_t clock_control_bl808x_mtimer_get_fclk_src_div(const struct device *dev)
{
	return (clock_control_bl808x_get_fclk(dev) / 1000000 - 1);
}

/* Get HCLK (AHB bus clock): FCLK divided by (hclk_div + 1). */
static uint32_t clock_control_bl808x_get_hclk(const struct device *dev)
{
	uint32_t tmp;
	uint32_t clock_f;

	tmp = sys_read32(GLB_BASE + GLB_SYS_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_HCLK_DIV_MSK) >> GLB_REG_HCLK_DIV_POS;
	clock_f = clock_control_bl808x_get_fclk(dev);
	return clock_f / (tmp + 1);
}

/* Get BCLK (APB peripheral bus clock): HCLK divided by (bclk_div + 1). */
static uint32_t clock_control_bl808x_get_bclk(const struct device *dev)
{
	uint32_t tmp;
	uint32_t source_clock;

	tmp = sys_read32(GLB_BASE + GLB_SYS_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_BCLK_DIV_MSK) >> GLB_REG_BCLK_DIV_POS;
	source_clock = clock_control_bl808x_get_hclk(dev);
	return source_clock / (tmp + 1);
}

/* Get 160M mux clock: alternative clock for SPI, DBI, UART, PKA peripherals. */
static uint32_t clock_control_bl808x_get_160m(const struct device *dev)
{
	uint32_t tmp;
	uint32_t source_clock;

	tmp = sys_read32(GLB_BASE + GLB_DIG_CLK_CFG1_OFFSET);
	tmp = (tmp & GLB_REG_TOP_MUXPLL_160M_SEL_MSK) >> GLB_REG_TOP_MUXPLL_160M_SEL_POS;
	source_clock = clock_control_bl808x_get_fclk(dev);
	switch (tmp) {
	case 1:
		return source_clock / 3;
	case 3:
		return source_clock * 2 / 5;
	default:
		return source_clock / 2;
	}
}

static void clock_control_bl808x_init_root_as_wifipll(const struct device *dev)
{
	struct clock_control_bl808x_data *data = dev->data;

	clock_control_bl808x_select_pll(data->root.pll_select);

	/* Ungate the selected PLL output */
	if (data->root.pll_select == BL808X_WIFIPLL_ID_DIV1) {
		clock_control_bl808x_ungate_pll(GLB_CGEN_TOP_WIFIPLL_320M_POS);
	} else if (data->root.pll_select == BL808X_WIFIPLL_ID_DIV3_4) {
		clock_control_bl808x_ungate_pll(GLB_CGEN_TOP_WIFIPLL_240M_POS);
	}

	if (data->wifipll.source == bl808x_clkid_clk_crystal) {
		clock_bflb_set_root_clock(BFLB_MAIN_CLOCK_PLL_XTAL);
	} else {
		clock_bflb_set_root_clock(BFLB_MAIN_CLOCK_PLL_RC32M);
	}
}

static void clock_control_bl808x_init_root_as_crystal(const struct device *dev)
{
	clock_bflb_set_root_clock(BFLB_MAIN_CLOCK_XTAL);
}

static uint32_t clock_control_bl808x_mtimer_get_xclk_src_div(const struct device *dev)
{
	return (clock_control_bl808x_get_xclk(dev) / 1000 / 1000 - 1);
}

/* Set F32K source mux in HBN: 0 = RC32K, 1 = XTAL32K. */
static void clock_control_bl808x_set_f32k_src(uint32_t src)
{
	uint32_t tmp = sys_read32(HBN_BASE + HBN_GLB_OFFSET);

	tmp = (tmp & HBN_F32K_SEL_UMSK) | (src << HBN_F32K_SEL_POS);
	sys_write32(tmp, HBN_BASE + HBN_GLB_OFFSET);
}

static int clock_control_bl808x_clock_trim_32M(void)
{
	uint32_t tmp;
	uint32_t trim, trim_ep;
	int err;
	const struct device *efuse = DEVICE_DT_GET_ONE(bflb_efuse);

	err = syscon_read_reg(efuse, EFUSE_RC32M_TRIM_OFFSET, &trim);
	if (err < 0) {
		LOG_ERR("Failed to read RC32M trim efuse (err %d)", err);
		return err;
	}
	err = syscon_read_reg(efuse, EFUSE_RC32M_TRIM_EP_OFFSET, &trim_ep);
	if (err < 0) {
		LOG_ERR("Failed to read RC32M trim enable efuse (err %d)", err);
		return err;
	}
	if (((trim_ep >> EFUSE_RC32M_TRIM_EP_EN_POS) & 1U) == 0U) {
		LOG_ERR("RC32M trim not enabled in efuse");
		return -EINVAL;
	}

	trim = (trim & EFUSE_RC32M_TRIM_MSK) >> EFUSE_RC32M_TRIM_POS;

	if (((trim_ep >> EFUSE_RC32M_TRIM_EP_PARITY_POS) & 1U) != (POPCOUNT(trim) & 1U)) {
		LOG_ERR("RC32M trim parity check failed");
		return -EINVAL;
	}

	/* BL808x: enable external trim code and write value, both in CTRL0 */
	tmp = sys_read32(PDS_BASE + PDS_RC32M_CTRL0_OFFSET);
	tmp = (tmp & PDS_RC32M_EXT_CODE_EN_UMSK) | (1U << PDS_RC32M_EXT_CODE_EN_POS);
	tmp = (tmp & PDS_RC32M_CODE_FR_EXT_UMSK) | (trim << PDS_RC32M_CODE_FR_EXT_POS);
	sys_write32(tmp, PDS_BASE + PDS_RC32M_CTRL0_OFFSET);
	clock_bflb_settle();

	return 0;
}

/*
 * Configure the machine timer clock source and divider.
 * source_clock: 0 = XCLK (RC32M or XTAL), 1 = FCLK (RC32M, XTAL, or PLL)
 * divider: 10-bit value, output = source / (divider + 1)
 */
static void clock_control_bl808x_set_machine_timer_clock(bool enable, uint32_t source_clock,
							 uint32_t divider)
{
	uint32_t tmp;

	/* CLK_SEL is a 1-bit field: clamp invalid values to XCLK (0) */
	if (source_clock > 1U) {
		source_clock = 0U;
	}

	tmp = sys_read32(MCU_MISC_BASE + MCU_MISC_MCU_E907_RTC_OFFSET);
	tmp = (tmp & MCU_MISC_REG_MCU_RTC_CLK_SEL_UMSK) |
	      (source_clock << MCU_MISC_REG_MCU_RTC_CLK_SEL_POS);
	sys_write32(tmp, MCU_MISC_BASE + MCU_MISC_MCU_E907_RTC_OFFSET);

	/* disable first, then set div */
	clock_control_bl808x_set_machine_timer_clock_enable(false);

	tmp = sys_read32(MCU_MISC_BASE + MCU_MISC_MCU_E907_RTC_OFFSET);
	tmp = (tmp & MCU_MISC_REG_MCU_RTC_DIV_UMSK) |
	      ((divider & MCU_MISC_MCU_RTC_DIV_MAX) << MCU_MISC_REG_MCU_RTC_DIV_POS);
	sys_write32(tmp, MCU_MISC_BASE + MCU_MISC_MCU_E907_RTC_OFFSET);

	clock_control_bl808x_set_machine_timer_clock_enable(enable);
}

static int clock_control_bl808x_update_f32k(const struct device *dev)
{
	struct clock_control_bl808x_data *data = dev->data;
	uint32_t tmp, tmpold;

	if (data->f32k.source != bl808x_clkid_clk_xtal32k &&
	    data->f32k.source != bl808x_clkid_clk_rc32k) {
		return -EINVAL;
	}

	if (data->f32k.xtal_enabled) {
		tmp = sys_read32(HBN_BASE + HBN_XTAL32K_OFFSET);
		tmpold = tmp;
		tmp &= HBN_XTAL32K_HIZ_EN_UMSK;
		tmp &= HBN_XTAL32K_INV_STRE_UMSK;
		tmp &= HBN_XTAL32K_REG_UMSK;
		/* SDK default: max inverter strength and regulator level */
		tmp |= 3U << HBN_XTAL32K_INV_STRE_POS;
		tmp |= 3U << HBN_XTAL32K_REG_POS;
		tmp |= HBN_PU_XTAL32K_MSK;
		tmp |= HBN_PU_XTAL32K_BUF_MSK;
		if (tmpold != tmp) {
			sys_write32(tmp, HBN_BASE + HBN_XTAL32K_OFFSET);
			clock_control_bl808x_clock_at_least_us(1000);
		}
	} else {
		tmp = sys_read32(HBN_BASE + HBN_XTAL32K_OFFSET);
		tmp |= HBN_XTAL32K_HIZ_EN_MSK;
		tmp &= HBN_PU_XTAL32K_UMSK;
		tmp &= HBN_PU_XTAL32K_BUF_UMSK;
		sys_write32(tmp, HBN_BASE + HBN_XTAL32K_OFFSET);
	}

	if (data->f32k.source == bl808x_clkid_clk_rc32k) {
		clock_control_bl808x_set_f32k_src(0);
	} else {
		clock_control_bl808x_set_f32k_src(1);
	}

	return 0;
}

static int clock_control_bl808x_update_clocks(const struct device *dev)
{
	struct clock_control_bl808x_data *data = dev->data;
	const struct clock_control_bl808x_config *config = dev->config;
	bl808x_pll_config cached_pll_cfg = {0};
	int ret = 0;

	/*
	 * CRITICAL: Pre-load PLL config from XIP flash BEFORE the RC32M switch.
	 *
	 * The bootloader configures the flash controller (SF_CTRL) timing
	 * parameters (read-delay, clock-invert, etc.) for its own clock speed
	 * (e.g. BCLK=80MHz from PLL → flash SPI=40MHz). When we switch root
	 * clock to RC32M below, BCLK drops to 32MHz and the flash SPI clock
	 * changes accordingly — but the SF_CTRL timing parameters are NOT
	 * updated. Any XIP data read after the switch hangs or returns
	 * corrupt data (instruction fetch is fine since we run from ITCM).
	 *
	 * The PLL config structs live in flash .rodata (separate compilation
	 * unit, intentionally NOT relocated to ITCM — ITCM rodata >~200B
	 * causes a separate boot failure). So we must copy them to the stack
	 * while flash timing is still valid, then use the stack copy for PLL
	 * programming.
	 *
	 * This was proven empirically: hardcoded PLL values (no flash read)
	 * always work; reading even one struct field from flash after RC32M
	 * switch causes boot failure; reading the same struct before the
	 * switch and using the cached copy works.
	 */
	if (data->wifipll.enabled) {
		const bl808x_pll_config *const *pll_cfgs = bl808x_pll_configs;
		const bl808x_pll_config *src;

		if (data->wifipll.top_frequency >= ROOT_CLK_RANGE_DELIM) {
			pll_cfgs = bl808x_pll_configs_500M;
		}
		if (data->wifipll.source == bl808x_clkid_clk_crystal) {
			src = pll_cfgs[config->crystal_id];
		} else {
			src = pll_cfgs[CRYSTAL_ID_FREQ_32000000];
		}
		cached_pll_cfg = *src;
	}

	/* set root clock to internal 32MHz Oscillator as failsafe */
	clock_bflb_set_root_clock(BFLB_MAIN_CLOCK_RC32M);
	if (clock_bflb_set_root_clock_dividers(0, 0) != 0) {
		return -EIO;
	}

	ret = clock_control_bl808x_update_f32k(dev);
	if (ret < 0) {
		return ret;
	}

	if (data->crystal_enabled) {
		if (clock_control_bl808x_init_crystal() < 0) {
			return -EIO;
		}
	} else {
		clock_control_bl808x_deinit_crystal();
	}

	ret = clock_bflb_set_root_clock_dividers(data->root.divider - 1, data->bclk.divider - 1);
	if (ret < 0) {
		return ret;
	}

	clock_control_bl808x_gate_pll(GLB_CGEN_TOP_AUPLL_DIV1_POS);
	clock_control_bl808x_gate_pll(GLB_CGEN_TOP_AUPLL_DIV2_POS);
	clock_control_bl808x_gate_pll(GLB_CGEN_TOP_WIFIPLL_320M_POS);
	clock_control_bl808x_gate_pll(GLB_CGEN_TOP_WIFIPLL_240M_POS);
	clock_control_bl808x_gate_pll(GLB_CGEN_EMI_AUPLL_DIV1_POS);
	clock_control_bl808x_gate_pll(GLB_CGEN_EMI_WIFIPLL_320M_POS);
	clock_control_bl808x_gate_pll(GLB_CGEN_MM_AUPLL_DIV1_POS);
	clock_control_bl808x_gate_pll(GLB_CGEN_MM_AUPLL_DIV2_POS);
	clock_control_bl808x_gate_pll(GLB_CGEN_MM_WIFIPLL_160M_POS);
	clock_control_bl808x_gate_pll(GLB_CGEN_MM_WIFIPLL_240M_POS);
	clock_control_bl808x_gate_pll(GLB_CGEN_MM_WIFIPLL_320M_POS);

	/* PLL setup */
	if (data->wifipll.enabled) {
		clock_control_bl808x_setup_wifipll(dev, &cached_pll_cfg);
	} else {
		clock_control_bl808x_deinit_wifipll();
	}

	if (data->root.source == bl808x_clkid_clk_wifipll) {
		if (!data->wifipll.enabled) {
			return -EINVAL;
		}
		clock_control_bl808x_init_root_as_wifipll(dev);
	} else if (data->root.source == bl808x_clkid_clk_crystal) {
		if (!data->crystal_enabled) {
			return -EINVAL;
		}
		clock_control_bl808x_init_root_as_crystal(dev);
	} else {
		/* Root clock already set to RC32M */
	}

	ret = clock_control_bl808x_clock_trim_32M();
	if (ret < 0) {
		LOG_WRN("RC32M trim failed (err %d), continuing without trim", ret);
		ret = 0;
	}

	if (data->root.source == bl808x_clkid_clk_wifipll ||
	    data->root.source == bl808x_clkid_clk_aupll) {
		clock_control_bl808x_set_machine_timer_clock(
			1, 1, clock_control_bl808x_mtimer_get_fclk_src_div(dev));
	} else {
		clock_control_bl808x_set_machine_timer_clock(
			1, 0, clock_control_bl808x_mtimer_get_xclk_src_div(dev));
	}

	clock_bflb_settle();

	return ret;
}

static void clock_control_bl808x_uart_set_clock_enable(bool enable)
{
	uint32_t tmp;

	tmp = sys_read32(GLB_BASE + GLB_UART_CFG0_OFFSET);
	if (enable) {
		tmp = (tmp & GLB_UART_CLK_EN_UMSK) | (1U << GLB_UART_CLK_EN_POS);
	} else {
		tmp &= GLB_UART_CLK_EN_UMSK;
	}
	sys_write32(tmp, GLB_BASE + GLB_UART_CFG0_OFFSET);
}

/* Clock source_clock parameter (matches SDK HBN_UART_CLK_Type):
 * 0: MCU_PBCLK/BCLK (HBN_UART_CLK_SEL=0, SEL2=0) = 80 MHz on PLL
 * 1: 160 MHz PLL     (HBN_UART_CLK_SEL=1, SEL2=0)
 * 2: XCLK            (HBN_UART_CLK_SEL2=1)         = 40 MHz crystal
 */
static void clock_control_bl808x_uart_set_clock(bool enable, uint32_t source_clock,
						uint32_t divider)
{
	uint32_t tmp;

	/* UART_CLK_DIV is a 3-bit field */
	if (divider > 0x7U) {
		divider = 0x7U;
	}
	/* HBN_UART_CLK: 0=BCLK, 1=160M PLL, 2=XCLK */
	if (source_clock > 2U) {
		source_clock = 2U;
	}
	/* disable uart clock */
	clock_control_bl808x_uart_set_clock_enable(false);

	tmp = sys_read32(GLB_BASE + GLB_UART_CFG0_OFFSET);
	tmp = (tmp & GLB_UART_CLK_DIV_UMSK) | (divider << GLB_UART_CLK_DIV_POS);
	sys_write32(tmp, GLB_BASE + GLB_UART_CFG0_OFFSET);

	tmp = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	if (source_clock < 2) {
		tmp = (tmp & HBN_UART_CLK_SEL_UMSK) | (source_clock << HBN_UART_CLK_SEL_POS);
		tmp &= HBN_UART_CLK_SEL2_UMSK;
	} else {
		tmp &= HBN_UART_CLK_SEL_UMSK;
		tmp = (tmp & HBN_UART_CLK_SEL2_UMSK) | (1U << HBN_UART_CLK_SEL2_POS);
	}
	sys_write32(tmp, HBN_BASE + HBN_GLB_OFFSET);

	clock_control_bl808x_uart_set_clock_enable(enable);
}

/* Ungate all peripheral clocks in CGEN_CFG1/CFG2 and configure UART clock. */
static void clock_control_bl808x_peripheral_clock_init(void)
{
	uint32_t regval;

	/* GLB_CGEN_CFG1: ungate all peripheral clocks */
	regval = sys_read32(GLB_BASE + GLB_CGEN_CFG1_OFFSET);
	regval |= (1U << GLB_CGEN_S1_GPIP_POS);    /* ADC/DAC */
	regval |= (1U << GLB_CGEN_S1_SEC_DBG_POS); /* SEC debug */
	regval |= (1U << GLB_CGEN_S1_SEC_ENG_POS); /* SEC engine */
	regval |= (1U << GLB_CGEN_S1_DMA_POS);     /* DMA0 */
	regval |= (1U << GLB_CGEN_S1_RSVD13_POS);  /* USB */
	regval |= (1U << GLB_CGEN_S1A_UART0_POS);  /* UART0 */
	regval |= (1U << GLB_CGEN_S1A_UART1_POS);  /* UART1 */
	regval |= (1U << GLB_CGEN_S1A_SPI_POS);    /* SPI */
	regval |= (1U << GLB_CGEN_S1A_I2C_POS);    /* I2C0 */
	regval |= (1U << GLB_CGEN_S1A_PWM_POS);    /* PWM */
	regval |= (1U << GLB_CGEN_S1A_TIMER_POS);  /* TIMER/WDG */
	regval |= (1U << GLB_CGEN_S1A_IR_POS);     /* IR */
	regval |= (1U << GLB_CGEN_S1A_RSVD11_POS); /* I2S */
	regval |= (1U << GLB_CGEN_S1A_UART2_POS);  /* CAN/UART2 */
	sys_write32(regval, GLB_BASE + GLB_CGEN_CFG1_OFFSET);

	/* GLB_CGEN_CFG2 (0x588): ungate USB clock */
	regval = sys_read32(GLB_BASE + GLB_CGEN_CFG2_OFFSET);
	regval |= (1U << GLB_CGEN_S1_EXT_USB_POS); /* USB */
	regval |= (1U << GLB_CGEN_S1_EXT_SDH_POS); /* SDH/eMMC */
	sys_write32(regval, GLB_BASE + GLB_CGEN_CFG2_OFFSET);

	/* MCU_PBCLK (BCLK=80MHz), div=0 (UART driver assumes BCLK) */
	clock_control_bl808x_uart_set_clock(1, 0, 0);
}

static int clock_control_bl808x_on(const struct device *dev, clock_control_subsys_t sys)
{
	struct clock_control_bl808x_data *data = dev->data;
	int ret = -EINVAL;
	uint32_t key;
	enum bl808x_clkid oldroot;

	key = irq_lock();

	if ((enum bl808x_clkid)sys == bl808x_clkid_clk_crystal) {
		if (data->crystal_enabled) {
			ret = 0;
		} else {
			data->crystal_enabled = true;
			ret = clock_control_bl808x_update_clocks(dev);
			if (ret < 0) {
				data->crystal_enabled = false;
			}
		}
	} else if ((enum bl808x_clkid)sys == bl808x_clkid_clk_wifipll) {
		if (data->wifipll.enabled) {
			ret = 0;
		} else {
			data->wifipll.enabled = true;
			ret = clock_control_bl808x_update_clocks(dev);
			if (ret < 0) {
				data->wifipll.enabled = false;
			}
		}
	} else if ((int)sys == BFLB_FORCE_ROOT_RC32M) {
		if (data->root.source == bl808x_clkid_clk_rc32m) {
			ret = 0;
		} else {
			/* Cannot fail to set root to rc32m */
			data->root.source = bl808x_clkid_clk_rc32m;
			ret = clock_control_bl808x_update_clocks(dev);
		}
	} else if ((int)sys == BFLB_FORCE_ROOT_CRYSTAL) {
		if (data->root.source == bl808x_clkid_clk_crystal) {
			ret = 0;
		} else {
			oldroot = data->root.source;
			data->root.source = bl808x_clkid_clk_crystal;
			ret = clock_control_bl808x_update_clocks(dev);
			if (ret < 0) {
				data->root.source = oldroot;
			}
		}
	} else if ((int)sys == BFLB_FORCE_ROOT_PLL) {
		if (data->root.source == bl808x_clkid_clk_wifipll) {
			ret = 0;
		} else {
			oldroot = data->root.source;
			data->root.source = bl808x_clkid_clk_wifipll;
			ret = clock_control_bl808x_update_clocks(dev);
			if (ret < 0) {
				data->root.source = oldroot;
			}
		}
	}

	irq_unlock(key);
	return ret;
}

static int clock_control_bl808x_off(const struct device *dev, clock_control_subsys_t sys)
{
	struct clock_control_bl808x_data *data = dev->data;
	int ret = -EINVAL;
	uint32_t key;

	key = irq_lock();

	if ((enum bl808x_clkid)sys == bl808x_clkid_clk_crystal) {
		if (!data->crystal_enabled) {
			ret = 0;
		} else {
			data->crystal_enabled = false;
			ret = clock_control_bl808x_update_clocks(dev);
			if (ret < 0) {
				data->crystal_enabled = true;
			}
		}
	} else if ((enum bl808x_clkid)sys == bl808x_clkid_clk_wifipll) {
		if (!data->wifipll.enabled) {
			ret = 0;
		} else {
			data->wifipll.enabled = false;
			ret = clock_control_bl808x_update_clocks(dev);
			if (ret < 0) {
				data->wifipll.enabled = true;
			}
		}
	}

	irq_unlock(key);
	return ret;
}

static enum clock_control_status clock_control_bl808x_get_status(const struct device *dev,
								 clock_control_subsys_t sys)
{
	struct clock_control_bl808x_data *data = dev->data;

	if ((enum bl808x_clkid)sys == bl808x_clkid_clk_root) {
		return CLOCK_CONTROL_STATUS_ON;
	} else if ((enum bl808x_clkid)sys == bl808x_clkid_clk_bclk) {
		return CLOCK_CONTROL_STATUS_ON;
	} else if ((enum bl808x_clkid)sys == bl808x_clkid_clk_crystal) {
		if (data->crystal_enabled) {
			return CLOCK_CONTROL_STATUS_ON;
		} else {
			return CLOCK_CONTROL_STATUS_OFF;
		}
	} else if ((enum bl808x_clkid)sys == bl808x_clkid_clk_rc32m) {
		return CLOCK_CONTROL_STATUS_ON;
	} else if ((enum bl808x_clkid)sys == bl808x_clkid_clk_wifipll) {
		if (data->wifipll.enabled) {
			return CLOCK_CONTROL_STATUS_ON;
		} else {
			return CLOCK_CONTROL_STATUS_OFF;
		}
	} else if ((enum bl808x_clkid)sys == bl808x_clkid_clk_aupll) {
		if (data->aupll.enabled) {
			return CLOCK_CONTROL_STATUS_ON;
		} else {
			return CLOCK_CONTROL_STATUS_OFF;
		}
	} else {
		/* Unknown clock ID */
	}
	return CLOCK_CONTROL_STATUS_UNKNOWN;
}

static int clock_control_bl808x_get_rate(const struct device *dev, clock_control_subsys_t sys,
					 uint32_t *rate)
{
	struct clock_control_bl808x_data *data = dev->data;

	if ((enum bl808x_clkid)sys == bl808x_clkid_clk_root) {
		*rate = clock_control_bl808x_get_hclk(dev);
	} else if ((enum bl808x_clkid)sys == bl808x_clkid_clk_bclk) {
		*rate = clock_control_bl808x_get_bclk(dev);
	} else if ((enum bl808x_clkid)sys == bl808x_clkid_clk_crystal) {
		*rate = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, crystal), clock_frequency);
	} else if ((enum bl808x_clkid)sys == bl808x_clkid_clk_160mux) {
		if (data->wifipll.enabled || data->aupll.enabled) {
			*rate = clock_control_bl808x_get_160m(dev);
		} else {
			return -EINVAL;
		}
	} else if ((enum bl808x_clkid)sys == bl808x_clkid_clk_rc32m) {
		*rate = BFLB_RC32M_FREQUENCY;
	} else {
		return -EINVAL;
	}
	return 0;
}

static int clock_control_bl808x_init(const struct device *dev)
{
	int ret;
	uint32_t key;

	key = irq_lock();

	ret = clock_control_bl808x_update_clocks(dev);
	if (ret < 0) {
		irq_unlock(key);
		return ret;
	}

	clock_control_bl808x_peripheral_clock_init();

	clock_bflb_settle();

	irq_unlock(key);

	return 0;
}

static DEVICE_API(clock_control, clock_control_bl808x_api) = {
	.on = clock_control_bl808x_on,
	.off = clock_control_bl808x_off,
	.get_rate = clock_control_bl808x_get_rate,
	.get_status = clock_control_bl808x_get_status,
};

static const struct clock_control_bl808x_config clock_control_bl808x_config = {
	.crystal_id = CRYSTAL_FREQ_TO_ID(
		DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, crystal), clock_frequency)),
};

static struct clock_control_bl808x_data clock_control_bl808x_data = {
	.crystal_enabled = DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, crystal)),

	.wifipll =
		{
#if CLK_SRC_IS(wifipll_top, crystal)
			.source = bl808x_clkid_clk_crystal,
#else
			.source = bl808x_clkid_clk_rc32m,
#endif
			.top_frequency =
				DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, wifipll_top), top_frequency),
			.enabled = DT_NODE_HAS_STATUS_OKAY(
				DT_INST_CLOCKS_CTLR_BY_NAME(0, wifipll_top)),
		},

	.aupll =
		{
#if CLK_SRC_IS(aupll_top, crystal)
			.source = bl808x_clkid_clk_crystal,
#else
			.source = bl808x_clkid_clk_rc32m,
#endif
			.top_frequency =
				DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, aupll_top), top_frequency),
			.enabled =
				DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, aupll_top)),
		},

	.root =
		{
#if CLK_SRC_IS(root, wifipll_top)
			.pll_select =
				DT_CLOCKS_CELL(DT_INST_CLOCKS_CTLR_BY_NAME(0, root), select) & 0xF,
			.source = bl808x_clkid_clk_wifipll,
#elif CLK_SRC_IS(root, aupll_top)
			.pll_select =
				DT_CLOCKS_CELL(DT_INST_CLOCKS_CTLR_BY_NAME(0, root), select) & 0xF,
			.source = bl808x_clkid_clk_aupll,
#elif CLK_SRC_IS(root, crystal)
			.source = bl808x_clkid_clk_crystal,
#else
			.source = bl808x_clkid_clk_rc32m,
#endif
			.divider = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, root), divider),
		},

	.bclk =
		{
			.divider = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, bclk), divider),
		},

	.f32k =
		{
#if DT_SAME_NODE(DT_CLOCKS_CTLR(DT_INST_CLOCKS_CTLR_BY_NAME(0, f32k)),                             \
		 DT_INST_CLOCKS_CTLR_BY_NAME(0, xtal32k))
			.source = bl808x_clkid_clk_xtal32k,
#else
			.source = bl808x_clkid_clk_rc32k,
#endif
			.xtal_enabled =
				DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, xtal32k)),
		},
};

BUILD_ASSERT(CLK_SRC_IS(aupll_top, crystal) || CLK_SRC_IS(wifipll_top, crystal) ||
			     CLK_SRC_IS(root, crystal)
		     ? DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, crystal))
		     : 1,
	     "Crystal must be enabled to use it");

BUILD_ASSERT(CLK_SRC_IS(root, wifipll_top)
		     ? DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, wifipll_top))
		     : 1,
	     "Wifi PLL must be enabled to use it");

BUILD_ASSERT(CLK_SRC_IS(root, aupll_top)
		     ? DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, aupll_top))
		     : 1,
	     "Audio PLL must be enabled to use it");

BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, rc32m)), "RC32M is always on");

BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, rc32k)), "RC32K is always on");

BUILD_ASSERT(CLK_SRC_IS(f32k, xtal32k)
		     ? DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, xtal32k))
		     : 1,
	     "XTAL32K must be enabled to use it as F32K source");

BUILD_ASSERT(!DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, aupll_top)),
	     "Audio PLL is unsupported");

BUILD_ASSERT(DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, rc32m), clock_frequency) ==
		     BFLB_RC32M_FREQUENCY,
	     "RC32M must be 32M");

DEVICE_DT_INST_DEFINE(0, clock_control_bl808x_init, NULL, &clock_control_bl808x_data,
		      &clock_control_bl808x_config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_bl808x_api);
