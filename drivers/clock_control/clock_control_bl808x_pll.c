/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * PLL configuration data for BL808X clock driver.
 *
 * This file MUST be a separate compilation unit from clock_control_bl808x.c
 * for two reasons:
 *
 * 1. ITCM rodata limit: clock_control_bl808x.c is relocated to ITCM.
 *    Adding >~200B of rodata to the ITCM relocation section causes boot
 *    failure on the E907 (root cause unclear, proven empirically).
 *
 * 2. Flash timing constraint: the clock driver's update_clocks() must
 *    read this data BEFORE switching root clock to RC32M. After the
 *    switch, the flash SPI clock (BCLK-derived) changes but SF_CTRL
 *    timing parameters stay calibrated for the bootloader's clock —
 *    XIP data reads hang or return garbage. The data is copied to the
 *    stack while flash timing is still valid (see update_clocks comment).
 */

#include "clock_control_bl808x_pll.h"

/* Base PLL configs (VcoSpeed=5, sdmin normalized to 320MHz) */

/* XCLK is 32M */
static const bl808x_pll_config wifipll_32M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1E00000,
	.aupllPostDiv = 0,
};

/* XCLK is 38.4M */
static const bl808x_pll_config wifipll_38P4M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1900000,
	.aupllPostDiv = 0,
};

/* XCLK is 40M */
static const bl808x_pll_config wifipll_40M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1800000,
	.aupllPostDiv = 0,
};

/* XCLK is 24M */
static const bl808x_pll_config wifipll_24M = {
	.pllRefdivRatio = 1,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1400000,
	.aupllPostDiv = 0,
};

/* XCLK is 26M */
static const bl808x_pll_config wifipll_26M = {
	.pllRefdivRatio = 1,
	.pllIntFracSw = 1,
	.pllIcp1u = 1,
	.pllIcp5u = 0,
	.pllRz = 5,
	.pllCz = 2,
	.pllC3 = 2,
	.pllR4Short = 0,
	.pllC4En = 1,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 0,
	.pllSdmBypass = 0,
	.pllSdmin = 0x1276276,
	.aupllPostDiv = 0,
};

/* High-range PLL configs (VcoSpeed=7, sdmin normalized to 320MHz base) */

/* XCLK is 32M, high VCO range */
static const bl808x_pll_config wifipll_32M_500M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 7,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1E00000,
	.aupllPostDiv = 0,
};

/* XCLK is 38.4M, high VCO range */
static const bl808x_pll_config wifipll_38P4M_500M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 7,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1900000,
	.aupllPostDiv = 0,
};

/* XCLK is 40M, high VCO range */
static const bl808x_pll_config wifipll_40M_500M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 7,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1800000,
	.aupllPostDiv = 0,
};

/* XCLK is 24M, high VCO range */
static const bl808x_pll_config wifipll_24M_500M = {
	.pllRefdivRatio = 1,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 7,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1400000,
	.aupllPostDiv = 0,
};

/* XCLK is 26M, high VCO range */
static const bl808x_pll_config wifipll_26M_500M = {
	.pllRefdivRatio = 1,
	.pllIntFracSw = 1,
	.pllIcp1u = 1,
	.pllIcp5u = 0,
	.pllRz = 5,
	.pllCz = 2,
	.pllC3 = 2,
	.pllR4Short = 0,
	.pllC4En = 1,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 7,
	.pllSdmCtrlHw = 0,
	.pllSdmBypass = 0,
	.pllSdmin = 0x1276276,
	.aupllPostDiv = 0,
};

const bl808x_pll_config *const bl808x_pll_configs[CRYSTAL_VALUES_CNT] = {
	&wifipll_32M, &wifipll_24M, &wifipll_38P4M, &wifipll_40M, &wifipll_26M};

const bl808x_pll_config *const bl808x_pll_configs_500M[CRYSTAL_VALUES_CNT] = {
	&wifipll_32M_500M, &wifipll_24M_500M, &wifipll_38P4M_500M, &wifipll_40M_500M,
	&wifipll_26M_500M};
