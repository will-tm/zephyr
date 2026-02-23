/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CLOCK_CONTROL_BL808X_PLL_H_
#define CLOCK_CONTROL_BL808X_PLL_H_

#include <stdint.h>

/* WiFi PLL analog parameter set (same layout as BL808 SDK GLB_WAC_PLL_CFG_BASIC_Type) */
typedef struct {
	uint8_t pllRefdivRatio;
	uint8_t pllIntFracSw;
	uint8_t pllIcp1u;
	uint8_t pllIcp5u;
	uint8_t pllRz;
	uint8_t pllCz;
	uint8_t pllC3;
	uint8_t pllR4Short;
	uint8_t pllC4En;
	uint8_t pllSelSampleClk;
	uint8_t pllVcoSpeed;
	uint8_t pllSdmCtrlHw;
	uint8_t pllSdmBypass;
	uint32_t pllSdmin;
	uint8_t aupllPostDiv;
} bl808x_pll_config;

#define CRYSTAL_VALUES_CNT 5

extern const bl808x_pll_config *const bl808x_pll_configs[CRYSTAL_VALUES_CNT];
extern const bl808x_pll_config *const bl808x_pll_configs_500M[CRYSTAL_VALUES_CNT];

#endif /* CLOCK_CONTROL_BL808X_PLL_H_ */
