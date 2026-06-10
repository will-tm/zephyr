/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Prototypes for BL702L ROM and ROM-extension functions linked via
 * romdriver.ld / rom_map.ld. Replaces scattered extern declarations
 * across the SoC source files.
 */

#ifndef SOC_BFLB_BL70XL_ROM_API_H_
#define SOC_BFLB_BL70XL_ROM_API_H_

#include <stdint.h>

/* GLB */
void GLB_Set_System_CLK(uint8_t xtal_type, uint8_t clk_sel);
void GLB_Set_System_CLK_Div(uint8_t hclk_div, uint8_t bclk_div);
int GLB_Set_MTimer_CLK(uint8_t enable, uint8_t clk_sel, uint8_t div);
void GLB_Power_Off_DLL(void);
void GLB_AHB_MCU_Software_Reset(uint8_t slave);
uint32_t GLB_Set_BLE_CLK(uint8_t enable);
void GLB_Set_SF_CLK(uint8_t enable, uint8_t clk_sel, uint8_t div);

/* HBN */
void HBN_Set_ROOT_CLK_Sel(uint8_t clk_sel);
void HBN_Set_Wakeup_Addr(uint32_t addr);
void HBN_Set_Status_Flag(uint32_t flag);
void HBN_Enable_RTC_Counter(void);
int HBN_Trim_RC32K(void);
void HBN_Get_RTC_Timer_Val(uint32_t *val_low, uint32_t *val_high);
void HBN_GPIO_Wakeup_Set(uint16_t gpio_wakeup_src, uint8_t gpio_trig_type);

/* AON */
void AON_Power_On_XTAL(void);
void AON_Power_Off_XTAL(void);
void AON_Set_LDO11_SOC_Sstart_Delay(uint8_t delay);

/* PDS */
void PDS_Default_Level_Config(const uint32_t *cfg, uint32_t sleep_cnt);
void PDS_IntClear(void);

/* SF_CTRL / SFlash / L1C */
void SF_Cfg_Init_Flash_Gpio(uint8_t flash_pin_cfg, uint8_t restore_default);
void SF_Ctrl_Set_Flash_Image_Offset(uint32_t offset);
uint32_t SF_Ctrl_Get_Flash_Image_Offset(void);
void SF_Ctrl_Set_Owner(uint8_t owner);
void SFlash_Init(const void *cfg);
void SFlash_Reset_Continue_Read(void *flash_cfg);
void SFlash_Powerdown(void);
int SFlash_Releae_Powerdown(void *flash_cfg);
int SFlash_Restore_From_Powerdown(void *flash_cfg, uint8_t cont_read);
int SFlash_Cache_Read_Enable(void *flash_cfg, uint8_t io_mode, uint8_t cont_read,
			     uint8_t way_disable);
int SFlash_SetSPIMode(uint8_t mode);
int SFlash_Write_Enable(void *flash_cfg);
int SFlash_Qspi_Enable(void *flash_cfg);
int SFlash_DisableBurstWrap(void *flash_cfg);
int SFlash_SetBurstWrap(void *flash_cfg);
int SFlash_Software_Reset(void *flash_cfg);
int SFlash_Read(void *flash_cfg, uint8_t io_mode, uint8_t cont_read, uint32_t addr, uint8_t *data,
		uint32_t len);
void L1C_Set_Wrap(uint8_t enable);

/* ROM HAL extension (rom_map.ld) */
extern uint16_t bl_rtc_frequency;
void bl_pds_init(void);
uint16_t rom_bl_rtc_get_xtal_cnt_32k_counter(void);

#endif /* SOC_BFLB_BL70XL_ROM_API_H_ */
