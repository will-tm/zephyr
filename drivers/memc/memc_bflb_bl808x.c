/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * UHS PSRAM controller driver for BL808.
 *
 * BL808 uses a UHS (Ultra High Speed) PSRAM controller at 0x3000F000
 * (MM subsystem), NOT the regular PSRAM controller at 0x20052000.
 * The PSRAM is memory-mapped at 0x50000000.
 *
 * Based on bouffalo_sdk: bl808_psram_uhs.c, bl808_uhs_phy.c, board.c
 */

#define DT_DRV_COMPAT bflb_bl808x_psram

#include <zephyr/device.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(memc_bflb_bl808x, CONFIG_MEMC_LOG_LEVEL);

#include <bouffalolab/bl808x/bflb_soc.h>
#include <bouffalolab/bl808x/cci_reg.h>
#include <bouffalolab/bl808x/glb_reg.h>
#include <bouffalolab/bl808x/tzc_sec_reg.h>

/* UHS PSRAM controller base address (MM subsystem) */
#define PSRAM_UHS_BASE 0x3000F000U

/* PSRAM memory-mapped base address */
#define PSRAM_MEM_BASE 0x50000000U

/* Efuse fields */
#define EFUSE_DEV_INFOS_OFFSET 0x18
#define EFUSE_PSRAM_SIZE_POS   24
#define EFUSE_PSRAM_SIZE_MSK   GENMASK(1, 0)

/* Efuse PSRAM info values (from SDK board.c) */
#define UHS_32MB_PSRAM 2
#define UHS_64MB_PSRAM 3

/* UHS controller register offsets */
#define UHS_BASIC_OFF        0x00
#define UHS_CMD_OFF          0x04
#define UHS_MANUAL_OFF       0x0C
#define UHS_AUTO_FRESH_1_OFF 0x10
#define UHS_AUTO_FRESH_2_OFF 0x14
#define UHS_AUTO_FRESH_4_OFF 0x1C
#define UHS_TIMING_CTRL_OFF  0x30

/* PHY register offsets */
#define PHY_CFG_00_OFF 0x100
#define PHY_CFG_04_OFF 0x104
#define PHY_CFG_08_OFF 0x108
#define PHY_CFG_0C_OFF 0x10C
#define PHY_CFG_10_OFF 0x110
#define PHY_CFG_14_OFF 0x114
#define PHY_CFG_18_OFF 0x118
#define PHY_CFG_1C_OFF 0x11C
#define PHY_CFG_20_OFF 0x120
#define PHY_CFG_24_OFF 0x124
#define PHY_CFG_28_OFF 0x128
#define PHY_CFG_2C_OFF 0x12C
#define PHY_CFG_30_OFF 0x130
#define PHY_CFG_34_OFF 0x134
#define PHY_CFG_38_OFF 0x138
#define PHY_CFG_3C_OFF 0x13C
#define PHY_CFG_40_OFF 0x140
#define PHY_CFG_44_OFF 0x144
#define PHY_CFG_48_OFF 0x148
#define PHY_CFG_4C_OFF 0x14C
#define PHY_CFG_50_OFF 0x150

/* UHS_BASIC bit fields */
#define UHS_INIT_EN      BIT(0)
#define UHS_AF_EN        BIT(1)
#define UHS_CONFIG_REQ   BIT(2)
#define UHS_CONFIG_GNT   BIT(3)
#define UHS_MODE_REG_POS 8
#define UHS_MODE_REG_MSK GENMASK(15, 8)
#define UHS_ADDRMB_POS   16
#define UHS_ADDRMB_MSK   GENMASK(23, 16)
#define UHS_LINEAR_POS   28
#define UHS_LINEAR_MSK   GENMASK(31, 28)

/* UHS_CMD bit fields */
#define UHS_REGR_PULSE BIT(4)
#define UHS_REGR_DONE  BIT(12)

/* UHS_MANUAL bit fields */
#define UHS_PCK_T_DIV_POS 24
#define UHS_PCK_T_DIV_MSK GENMASK(31, 24)

/* UHS_AUTO_FRESH_2 bit fields */
#define UHS_REFI_CYCLE_POS 0
#define UHS_REFI_CYCLE_MSK GENMASK(15, 0)

/* UHS_AUTO_FRESH_4 bit fields */
#define UHS_BUST_CYCLE_POS 0
#define UHS_BUST_CYCLE_MSK GENMASK(6, 0)

/* PHY_CFG_30 bit fields (latency) */
#define PHY_WL_DQ_DIG_POS 0
#define PHY_WL_DQ_DIG_MSK GENMASK(2, 0)
#define PHY_WL_DQ_ANA_POS 4
#define PHY_WL_DQ_ANA_MSK GENMASK(6, 4)
#define PHY_WL_DIG_POS    8
#define PHY_WL_DIG_MSK    GENMASK(10, 8)
#define PHY_WL_ANA_POS    12
#define PHY_WL_ANA_MSK    GENMASK(14, 12)
#define PHY_RL_DIG_POS    16
#define PHY_RL_DIG_MSK    GENMASK(19, 16)
#define PHY_RL_ANA_POS    20
#define PHY_RL_ANA_MSK    GENMASK(22, 20)

/* PHY_CFG_00 bit fields */
#define PHY_CK_DLY_DRV_POS  16
#define PHY_CK_DLY_DRV_MSK  GENMASK(19, 16)
#define PHY_CEN_DLY_DRV_POS 28
#define PHY_CEN_DLY_DRV_MSK GENMASK(31, 28)

/* PHY_CFG_04 bit fields */
#define PHY_DM1_DLY_DRV_POS 12
#define PHY_DM1_DLY_DRV_MSK GENMASK(15, 12)
#define PHY_DM0_DLY_DRV_POS 28
#define PHY_DM0_DLY_DRV_MSK GENMASK(31, 28)

/* PHY_CFG_28 bit fields */
#define PHY_DQS0_DLY_DRV_POS     24
#define PHY_DQS0_DLY_DRV_MSK     GENMASK(27, 24)
#define PHY_DQS0_DIFF_DLY_RX_POS 28
#define PHY_DQS0_DIFF_DLY_RX_MSK GENMASK(31, 28)

/* PHY_CFG_2C bit fields */
#define PHY_DQS1_DLY_DRV_POS     24
#define PHY_DQS1_DLY_DRV_MSK     GENMASK(27, 24)
#define PHY_DQS1_DIFF_DLY_RX_POS 28
#define PHY_DQS1_DIFF_DLY_RX_MSK GENMASK(31, 28)

/* PHY_CFG_50 bit fields */
#define PHY_DQ_OE_MID_P_POS 8
#define PHY_DQ_OE_MID_P_MSK GENMASK(10, 8)
#define PHY_DQ_OE_MID_N_POS 12
#define PHY_DQ_OE_MID_N_MSK GENMASK(14, 12)
#define PHY_WL_CEN_ANA_POS  24
#define PHY_WL_CEN_ANA_MSK  GENMASK(26, 24)

/* PHY_CFG_40 bit fields */
#define PHY_UHS_DMY0_POS 8
#define PHY_UHS_DMY0_MSK GENMASK(11, 8)

/* DQ lane DLY_DRV/DLY_RX: PHY_CFG_08 through PHY_CFG_24, two DQ lanes per register.
 * Per register: [31:28]=DQhi_DLY_DRV, [23:20]=DQhi_DLY_RX,
 *               [15:12]=DQlo_DLY_DRV, [7:4]=DQlo_DLY_RX
 */

/* GLB LDO12UHS register */
#define GLB_LDO12UHS_OFF      0x6D0
#define GLB_PU_LDO12UHS_BIT   BIT(0)
#define GLB_LDO12UHS_VOUT_POS 20
#define GLB_LDO12UHS_VOUT_MSK GENMASK(23, 20)

/* GLB UHS PLL registers */
#define GLB_UHS_PLL_CFG0_OFF 0x7D0

/* Memory size codes for PSRAM_UHS (ADDRMB_MSK field) */
#define PSRAM_SIZE_32MB 0x1FU
#define PSRAM_SIZE_64MB 0x3FU

/* Page size codes (LINEAR_BND_B field) */
#define PSRAM_PAGE_2KB 0x0BU

#define UHS_RW_TIMEOUT 0xFFFFFU

struct memc_bflb_bl808x_data {
	uint32_t psram_size;
};

struct memc_bflb_bl808x_config {
	uint8_t unused;
};

static inline uint32_t uhs_read(uint32_t off)
{
	return sys_read32(PSRAM_UHS_BASE + off);
}

static inline void uhs_write(uint32_t off, uint32_t val)
{
	sys_write32(val, PSRAM_UHS_BASE + off);
}

static inline void uhs_set_bits(uint32_t off, uint32_t mask, uint32_t val, uint32_t pos)
{
	uint32_t tmp = uhs_read(off);

	tmp &= ~mask;
	tmp |= (val << pos) & mask;
	uhs_write(off, tmp);
}

/*
 * Initialize UHS PLL for 2000MHz.
 * The PSRAM_UHS controller at 0x3000F000 requires UHS PLL to be running
 * before its registers can be accessed. Without it, reads fault.
 *
 * Ported from SDK: GLB_Config_UHS_PLL(GLB_XTAL_40M, uhsPllCfg_2000M)
 * which calls GLB_Power_Off_MU_PLL + GLB_MU_PLL_Ref_Clk_Sel + GLB_Power_On_MU_PLL.
 *
 * UHS PLL registers are at GLB_BASE + 0x7D0 (CFG0 through CFG9).
 * Bit fields use CCI_AUPLL_* names from cci_reg.h (shared MU PLL layout).
 *
 * For 40MHz XTAL → 2000MHz:
 *   refdiv_ratio=2, sel_sample_clk=2, vco_speed=7
 *   even_div_en=1, even_div_ratio=40 (2000/50)
 *   sdmin=0x32000
 *   refclk_sel=0 (XTAL)
 */
static void uhs_pll_init_2000m(void)
{
	uint32_t pll = GLB_BASE + GLB_UHS_PLL_CFG0_OFF;
	uint32_t tmp;

	/* Power off UHS PLL first */
	tmp = sys_read32(pll + 0x00); /* CFG0 */
	tmp &= ~CCI_PU_AUPLL_MSK;
	tmp &= ~CCI_PU_AUPLL_SFREG_MSK;
	sys_write32(tmp, pll + 0x00);

	/* CFG1: reference clock = XTAL (refclk_sel=0), refdiv_ratio=2 */
	tmp = sys_read32(pll + 0x04);
	tmp &= ~CCI_AUPLL_REFCLK_SEL_MSK;
	tmp &= ~CCI_AUPLL_REFDIV_RATIO_MSK;
	tmp |= (2U << CCI_AUPLL_REFDIV_RATIO_POS);
	sys_write32(tmp, pll + 0x04);

	/* CFG4: sel_sample_clk=2 */
	tmp = sys_read32(pll + 0x10);
	tmp &= ~CCI_AUPLL_SEL_SAMPLE_CLK_MSK;
	tmp |= (2U << CCI_AUPLL_SEL_SAMPLE_CLK_POS);
	sys_write32(tmp, pll + 0x10);

	/* CFG5: vco_speed=7 */
	tmp = sys_read32(pll + 0x14);
	tmp &= ~CCI_AUPLL_VCO_SPEED_MSK;
	tmp |= (7U << CCI_AUPLL_VCO_SPEED_POS);
	sys_write32(tmp, pll + 0x14);

	/* CFG1: even_div_en=1, even_div_ratio=40 (UHS PLL specific) */
	tmp = sys_read32(pll + 0x04);
	tmp |= GLB_UHSPLL_EVEN_DIV_EN_MSK;
	tmp &= ~GLB_UHSPLL_EVEN_DIV_RATIO_MSK;
	tmp |= (40U << GLB_UHSPLL_EVEN_DIV_RATIO_POS);
	sys_write32(tmp, pll + 0x04);

	/* CFG6: sdmin=0x32000 */
	tmp = sys_read32(pll + 0x18);
	tmp &= ~CCI_AUPLL_SDMIN_MSK;
	tmp |= (0x32000U << CCI_AUPLL_SDMIN_POS);
	sys_write32(tmp, pll + 0x18);

	/* Power up: pu_sfreg=1 */
	tmp = sys_read32(pll + 0x00);
	tmp |= CCI_PU_AUPLL_SFREG_MSK;
	sys_write32(tmp, pll + 0x00);
	k_busy_wait(3);

	/* pu_aupll=1 */
	tmp = sys_read32(pll + 0x00);
	tmp |= CCI_PU_AUPLL_MSK;
	sys_write32(tmp, pll + 0x00);
	k_busy_wait(3);

	/* Toggle SDM reset: 1→0→1 */
	tmp = sys_read32(pll + 0x00);
	tmp |= CCI_AUPLL_SDM_RSTB_MSK;
	sys_write32(tmp, pll + 0x00);
	k_busy_wait(2);
	tmp &= ~CCI_AUPLL_SDM_RSTB_MSK;
	sys_write32(tmp, pll + 0x00);
	k_busy_wait(2);
	tmp |= CCI_AUPLL_SDM_RSTB_MSK;
	sys_write32(tmp, pll + 0x00);

	/* Toggle FBDV reset: 1→0→1 */
	tmp = sys_read32(pll + 0x00);
	tmp |= CCI_AUPLL_FBDV_RSTB_MSK;
	sys_write32(tmp, pll + 0x00);
	k_busy_wait(2);
	tmp &= ~CCI_AUPLL_FBDV_RSTB_MSK;
	sys_write32(tmp, pll + 0x00);
	k_busy_wait(2);
	tmp |= CCI_AUPLL_FBDV_RSTB_MSK;
	sys_write32(tmp, pll + 0x00);

	/* Wait for PLL to stabilize */
	k_busy_wait(45);
}

/*
 * Power up LDO12UHS — provides 1.2V to UHS PSRAM PHY.
 * From SDK bl808_psram_uhs.c: power_up_ldo12uhs()
 */
static void power_up_ldo12uhs(void)
{
	uint32_t tmp;

	tmp = sys_read32(GLB_BASE + GLB_LDO12UHS_OFF);
	tmp |= GLB_PU_LDO12UHS_BIT;
	sys_write32(tmp, GLB_BASE + GLB_LDO12UHS_OFF);
	k_busy_wait(300);

	tmp = sys_read32(GLB_BASE + GLB_LDO12UHS_OFF);
	tmp &= ~GLB_LDO12UHS_VOUT_MSK;
	tmp |= (6U << GLB_LDO12UHS_VOUT_POS);
	sys_write32(tmp, GLB_BASE + GLB_LDO12UHS_OFF);
	k_busy_wait(1);
}

/*
 * PHY analog init: set CEN/CK/CKN pins.
 * From SDK: set_cen_ck_ckn()
 */
static void phy_set_cen_ck_ckn(void)
{
	uint32_t tmp;

	tmp = uhs_read(PHY_CFG_50_OFF);
	tmp &= ~PHY_DQ_OE_MID_N_MSK;
	tmp &= ~PHY_DQ_OE_MID_P_MSK;
	uhs_write(PHY_CFG_50_OFF, tmp);
	k_busy_wait(1);

	tmp = uhs_read(PHY_CFG_40_OFF);
	tmp &= 0xFFFCFFFFU;
	uhs_write(PHY_CFG_40_OFF, tmp);
	uhs_set_bits(PHY_CFG_40_OFF, PHY_UHS_DMY0_MSK, 1, PHY_UHS_DMY0_POS);
	k_busy_wait(1);
}

/*
 * Overwrite default PHY register content.
 * From SDK: set_or_uhs()
 */
static void phy_set_or_uhs(void)
{
	static const uint32_t phy_cfg_data[12] = {
		0x802b0200, 0x60206020, 0x70027002, 0x70027002, 0x70027002, 0x70027002,
		0x70027002, 0x70027002, 0x70027002, 0x70027002, 0x26000000, 0x26000006,
	};
	uint32_t tmp;

	for (int i = 0; i < ARRAY_SIZE(phy_cfg_data); i++) {
		sys_write32(phy_cfg_data[i], PSRAM_UHS_BASE + PHY_CFG_00_OFF + i * 4);
	}

	tmp = uhs_read(PHY_CFG_30_OFF);
	tmp &= 0x08FFFFFFU;
	tmp |= 0x07000000U;
	uhs_write(PHY_CFG_30_OFF, tmp);

	tmp = uhs_read(PHY_CFG_48_OFF);
	tmp &= 0xFFFFFCFFU;
	tmp |= 0x00000200U;
	uhs_write(PHY_CFG_48_OFF, tmp);

	tmp = uhs_read(PHY_CFG_4C_OFF);
	tmp &= 0xFFE0FFFFU;
	uhs_write(PHY_CFG_4C_OFF, tmp);

	tmp = uhs_read(PHY_CFG_50_OFF);
	tmp &= 0xFF88FF88U;
	tmp |= 0x00330033U;
	uhs_write(PHY_CFG_50_OFF, tmp);
	k_busy_wait(1);
}

/*
 * Switch to LDO12UHS power rail.
 * From SDK: switch_to_ldo12uhs()
 */
static void phy_switch_to_ldo12uhs(void)
{
	uint32_t tmp;

	tmp = uhs_read(PHY_CFG_40_OFF);
	tmp &= 0xFFCFFFFFU;
	uhs_write(PHY_CFG_40_OFF, tmp);
	k_busy_wait(1);
}

/*
 * Release CEN/CK/CKN pins.
 * From SDK: release_cen_ck_ckn()
 */
static void phy_release_cen_ck_ckn(void)
{
	uint32_t tmp;

	tmp = uhs_read(PHY_CFG_40_OFF);
	tmp &= 0xFFFCFEFFU;
	tmp |= 0x00030000U;
	uhs_write(PHY_CFG_40_OFF, tmp);
	k_busy_wait(1);

	tmp = uhs_read(PHY_CFG_50_OFF);
	tmp &= ~PHY_DQ_OE_MID_N_MSK;
	tmp |= (3U << PHY_DQ_OE_MID_N_POS);
	tmp &= ~PHY_DQ_OE_MID_P_MSK;
	tmp |= (3U << PHY_DQ_OE_MID_P_POS);
	uhs_write(PHY_CFG_50_OFF, tmp);
	k_busy_wait(1);
}

/*
 * Configure PHY parameters for 2000MHz datarate.
 * From SDK: config_uhs_phy(2000) — only the >1866 branch.
 */
static void phy_config_2000(void)
{
	uint32_t tmp;

	uhs_write(PHY_CFG_30_OFF, 0x0F0A1323);
	uhs_write(PHY_CFG_34_OFF, 0x0B030404);
	uhs_write(PHY_CFG_38_OFF, 0x050E0419);
	uhs_write(PHY_CFG_3C_OFF, 0x0A6A1C1C);
	uhs_write(PHY_CFG_44_OFF, 0x0711070E);

	tmp = uhs_read(PHY_CFG_50_OFF);
	tmp &= ~PHY_WL_CEN_ANA_MSK;
	tmp |= (1U << PHY_WL_CEN_ANA_POS);
	uhs_write(PHY_CFG_50_OFF, tmp);
}

/*
 * Full PHY analog initialization.
 * From SDK: Psram_analog_init()
 */
static void psram_analog_init(void)
{
	power_up_ldo12uhs();
	phy_set_cen_ck_ckn();
	phy_set_or_uhs();
	phy_switch_to_ldo12uhs();
	phy_release_cen_ck_ckn();
	phy_config_2000();
}

/*
 * Initialize UHS PSRAM controller.
 * From SDK: Psram_UHS_Init() with 2000MHz/8MB/2KB-page/normal-temp.
 */
static void psram_uhs_controller_init(uint8_t mem_size_code)
{
	uint32_t tmp;

	/* Timing control for >1600MHz */
	uhs_write(UHS_TIMING_CTRL_OFF, 0x1A03000F);

	/* PHY analog init */
	psram_analog_init();

	/* Wait 150us for PHY settle */
	k_busy_wait(150);

	/* Auto-refresh clock divider: PCK_T_DIV=4 for >=1800MHz (~50MHz refresh) */
	tmp = uhs_read(UHS_MANUAL_OFF);
	tmp &= ~UHS_PCK_T_DIV_MSK;
	tmp |= (4U << UHS_PCK_T_DIV_POS);
	uhs_write(UHS_MANUAL_OFF, tmp);

	/* Refresh window: 32ms for normal temp */
	uhs_write(UHS_AUTO_FRESH_1_OFF, 0x16E360);

	/* Refresh interval: 370 cycles for normal temp */
	tmp = uhs_read(UHS_AUTO_FRESH_2_OFF);
	tmp &= ~UHS_REFI_CYCLE_MSK;
	tmp |= (370U << UHS_REFI_CYCLE_POS);
	uhs_write(UHS_AUTO_FRESH_2_OFF, tmp);

	/* Single refresh burst cycle = 5 */
	tmp = uhs_read(UHS_AUTO_FRESH_4_OFF);
	tmp &= ~UHS_BUST_CYCLE_MSK;
	tmp |= (5U << UHS_BUST_CYCLE_POS);
	uhs_write(UHS_AUTO_FRESH_4_OFF, tmp);

	/* Enable auto-refresh, set memory size and page size */
	tmp = uhs_read(UHS_BASIC_OFF);
	tmp |= UHS_AF_EN;
	tmp &= ~UHS_ADDRMB_MSK;
	tmp |= ((uint32_t)mem_size_code << UHS_ADDRMB_POS);
	tmp &= ~UHS_LINEAR_MSK;
	tmp |= ((uint32_t)PSRAM_PAGE_2KB << UHS_LINEAR_POS);
	uhs_write(UHS_BASIC_OFF, tmp);

	/* Enable initialization */
	tmp = uhs_read(UHS_BASIC_OFF);
	tmp |= UHS_INIT_EN;
	uhs_write(UHS_BASIC_OFF, tmp);
}

/*
 * Apply hardcoded PHY calibration values for 2000Mbps.
 * From SDK board.c: known-good calibration values.
 *
 * rl=39, rdqs=3, rdq=0, wl=13, wdqs=4, wdq=5, ck=9
 */
static void psram_apply_phy_cal(void)
{
	uint32_t tmp;

	/* set_uhs_latency_r(39): RL_ANA = 39%4 = 3, RL_DIG = 39/4 = 9 */
	tmp = uhs_read(PHY_CFG_30_OFF);
	tmp &= ~PHY_RL_ANA_MSK;
	tmp |= (3U << PHY_RL_ANA_POS);
	tmp &= ~PHY_RL_DIG_MSK;
	tmp |= (9U << PHY_RL_DIG_POS);
	uhs_write(PHY_CFG_30_OFF, tmp);
	k_busy_wait(50);

	/* set_uhs_latency_w(13): WL_ANA=13%4=1, WL_DIG=13/4=3,
	 *                        WL_DQ_ANA=14%4=2, WL_DQ_DIG=14/4=3
	 */
	tmp = uhs_read(PHY_CFG_30_OFF);
	tmp &= ~PHY_WL_ANA_MSK;
	tmp |= (1U << PHY_WL_ANA_POS);
	tmp &= ~PHY_WL_DIG_MSK;
	tmp |= (3U << PHY_WL_DIG_POS);
	tmp &= ~PHY_WL_DQ_ANA_MSK;
	tmp |= (2U << PHY_WL_DQ_ANA_POS);
	tmp &= ~PHY_WL_DQ_DIG_MSK;
	tmp |= (3U << PHY_WL_DQ_DIG_POS);
	uhs_write(PHY_CFG_30_OFF, tmp);
	k_busy_wait(50);

	/* cfg_dqs_rx(3): DQS0_DIFF_DLY_RX=3, DQS1_DIFF_DLY_RX=3 */
	uhs_set_bits(PHY_CFG_28_OFF, PHY_DQS0_DIFF_DLY_RX_MSK, 3, PHY_DQS0_DIFF_DLY_RX_POS);
	uhs_set_bits(PHY_CFG_2C_OFF, PHY_DQS1_DIFF_DLY_RX_MSK, 3, PHY_DQS1_DIFF_DLY_RX_POS);
	k_busy_wait(10);

	/* cfg_dq_rx(0): all DQ_DLY_RX = 0
	 * Since set_or_uhs() already wrote these registers with 0x70027002 pattern
	 * for PHY_CFG_08-24, and DLY_RX fields are in the lower nibbles,
	 * we clear them by ANDing out the RX bits.
	 * Each register: bits [7:4]=DQlo_DLY_RX, bits [23:20]=DQhi_DLY_RX
	 */
	for (int i = 0; i < 8; i++) {
		uint32_t off = PHY_CFG_08_OFF + i * 4;

		tmp = uhs_read(off);
		tmp &= ~(0xFU << 4);  /* DQlo_DLY_RX */
		tmp &= ~(0xFU << 20); /* DQhi_DLY_RX */
		uhs_write(off, tmp);
	}
	/* PHY_CFG_24 also has DQ14/DQ15 */

	/* cfg_dq_drv(5): all DQ_DLY_DRV = 5, DM0_DLY_DRV = 5, DM1_DLY_DRV = 5
	 * PHY_CFG_04: DM1_DLY_DRV[15:12]=5, DM0_DLY_DRV[31:28]=5
	 */
	uhs_set_bits(PHY_CFG_04_OFF, PHY_DM0_DLY_DRV_MSK, 5, PHY_DM0_DLY_DRV_POS);
	uhs_set_bits(PHY_CFG_04_OFF, PHY_DM1_DLY_DRV_MSK, 5, PHY_DM1_DLY_DRV_POS);

	/* PHY_CFG_08 - PHY_CFG_24: DQ0-15_DLY_DRV = 5
	 * Each register: bits [15:12]=DQlo_DLY_DRV, bits [31:28]=DQhi_DLY_DRV
	 */
	for (int i = 0; i < 8; i++) {
		uint32_t off = PHY_CFG_08_OFF + i * 4;

		tmp = uhs_read(off);
		tmp &= ~(0xFU << 12); /* DQlo_DLY_DRV */
		tmp |= (5U << 12);
		tmp &= ~(0xFU << 28); /* DQhi_DLY_DRV */
		tmp |= (5U << 28);
		uhs_write(off, tmp);
	}

	/* cfg_ck_cen_drv(wdq+4=9, wdq+1=6): CK_DLY_DRV=9, CEN_DLY_DRV=6 */
	uhs_set_bits(PHY_CFG_00_OFF, PHY_CK_DLY_DRV_MSK, 9, PHY_CK_DLY_DRV_POS);
	uhs_set_bits(PHY_CFG_00_OFF, PHY_CEN_DLY_DRV_MSK, 6, PHY_CEN_DLY_DRV_POS);
	k_busy_wait(50);

	/* cfg_dqs_drv(4): DQS0_DLY_DRV=4, DQS1_DLY_DRV=4 */
	uhs_set_bits(PHY_CFG_28_OFF, PHY_DQS0_DLY_DRV_MSK, 4, PHY_DQS0_DLY_DRV_POS);
	uhs_set_bits(PHY_CFG_2C_OFF, PHY_DQS1_DLY_DRV_MSK, 4, PHY_DQS1_DLY_DRV_POS);
	k_busy_wait(10);
}

/*
 * Release TrustZone Controller PSRAM-A access restrictions.
 * From SDK: Tzc_Sec_PSRAMA_Access_Release() — clears region 0 enable.
 */
static void tzc_psrama_release(void)
{
	uint32_t tmp;

	tmp = sys_read32(TZC_SEC_BASE + TZC_SEC_TZC_PSRAMA_TZSRG_CTRL_OFFSET);
	tmp &= ~BIT(16); /* Disable region 0 → unrestricted access */
	sys_write32(tmp, TZC_SEC_BASE + TZC_SEC_TZC_PSRAMA_TZSRG_CTRL_OFFSET);
}

/*
 * Read a PSRAM mode register via the UHS controller.
 * From SDK: PSram_UHS_Read_Reg()
 */
static int psram_uhs_read_reg(uint32_t reg_addr, uint8_t *val)
{
	uint32_t tmp;
	int cnt;

	/* Request config access */
	tmp = uhs_read(UHS_BASIC_OFF);
	tmp |= UHS_CONFIG_REQ;
	uhs_write(UHS_BASIC_OFF, tmp);

	for (cnt = 0; cnt < UHS_RW_TIMEOUT; cnt++) {
		if (uhs_read(UHS_BASIC_OFF) & UHS_CONFIG_GNT) {
			break;
		}
	}
	if (cnt >= UHS_RW_TIMEOUT) {
		return -ETIMEDOUT;
	}

	/* Set mode register address */
	tmp = uhs_read(UHS_BASIC_OFF);
	tmp &= ~UHS_MODE_REG_MSK;
	tmp |= (reg_addr << UHS_MODE_REG_POS);
	uhs_write(UHS_BASIC_OFF, tmp);

	/* Trigger register read */
	tmp = uhs_read(UHS_CMD_OFF);
	tmp |= UHS_REGR_PULSE;
	uhs_write(UHS_CMD_OFF, tmp);

	for (cnt = 0; cnt < UHS_RW_TIMEOUT; cnt++) {
		if (uhs_read(UHS_CMD_OFF) & UHS_REGR_DONE) {
			break;
		}
	}
	if (cnt >= UHS_RW_TIMEOUT) {
		tmp = uhs_read(UHS_BASIC_OFF);
		tmp &= ~UHS_CONFIG_REQ;
		uhs_write(UHS_BASIC_OFF, tmp);
		return -ETIMEDOUT;
	}

	/* Read result */
	*val = (uhs_read(UHS_CMD_OFF) >> 24) & 0xFFU;

	/* Release config access */
	tmp = uhs_read(UHS_BASIC_OFF);
	tmp &= ~UHS_CONFIG_REQ;
	uhs_write(UHS_BASIC_OFF, tmp);

	return 0;
}

/*
 * mr_read_back: read mode registers 0-4 (skip 3) to verify PSRAM responds.
 * From SDK: mr_read_back()
 */
static int psram_mr_read_back(void)
{
	uint8_t val;
	int err;

	for (int i = 0; i <= 4; i++) {
		if (i == 3) {
			continue;
		}
		err = psram_uhs_read_reg(i, &val);
		if (err) {
			LOG_ERR("MR%d read failed", i);
			return err;
		}
		LOG_DBG("MR%d = 0x%02x", i, val);
	}
	return 0;
}

static int memc_bflb_bl808x_init(const struct device *dev)
{
	struct memc_bflb_bl808x_data *data = dev->data;
	const struct device *efuse = DEVICE_DT_GET_ONE(bflb_efuse);
	uint32_t dev_infos, psram_info;
	uint8_t mem_size_code;
	int err;

	err = syscon_read_reg(efuse, EFUSE_DEV_INFOS_OFFSET, &dev_infos);
	if (err < 0) {
		LOG_ERR("Efuse read failed: %d", err);
		return err;
	}

	psram_info = (dev_infos >> EFUSE_PSRAM_SIZE_POS) & EFUSE_PSRAM_SIZE_MSK;
	if (psram_info == UHS_32MB_PSRAM) {
		data->psram_size = MB(32);
		mem_size_code = PSRAM_SIZE_32MB;
	} else if (psram_info == UHS_64MB_PSRAM) {
		data->psram_size = MB(64);
		mem_size_code = PSRAM_SIZE_64MB;
	} else {
		LOG_WRN("No UHS PSRAM (info=%u)", psram_info);
		return 0;
	}

	/* Init UHS PLL — required before PSRAM_UHS register access */
	uhs_pll_init_2000m();

	/* Check if bootloader already initialized the controller */
	uint32_t uhs_basic = uhs_read(UHS_BASIC_OFF);

	if (!(uhs_basic & UHS_INIT_EN)) {
		psram_uhs_controller_init(mem_size_code);
	}

	/* Apply known-good PHY calibration for 2000Mbps */
	psram_apply_phy_cal();

	/* Release TZC security on PSRAM-A */
	tzc_psrama_release();

	/* Read back mode registers to verify PSRAM responds */
	err = psram_mr_read_back();
	if (err) {
		LOG_ERR("PSRAM mode register readback failed");
		return err;
	}

	/* Verify memory-mapped access */
	volatile uint32_t *p = (volatile uint32_t *)PSRAM_MEM_BASE;

	p[0] = 0xDEADBEEF;
	if (p[0] != 0xDEADBEEF) {
		LOG_ERR("PSRAM write/readback failed");
		return -EIO;
	}
	p[0] = 0;

	LOG_DBG("UHS PSRAM: %u MB at 0x%08x", data->psram_size / MB(1), PSRAM_MEM_BASE);

	return 0;
}

static struct memc_bflb_bl808x_data data;

static const struct memc_bflb_bl808x_config config;

DEVICE_DT_INST_DEFINE(0, memc_bflb_bl808x_init, NULL, &data, &config, POST_KERNEL,
		      CONFIG_MEMC_INIT_PRIORITY, NULL);
