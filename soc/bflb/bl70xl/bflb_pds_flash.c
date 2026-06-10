/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <bflb_soc.h>
#include <l1c_reg.h>

#include "bflb_pds.h"
#include "bl70xl_rom_api.h"

#define PDS_FLASH_CFG_SIZE            84U
#define PDS_FLASH_CFG_PD_DELAY_OFFSET 82U

#define PDS_FLASH_XIP_BASE          0x23000000U
#define PDS_FLASH_BOOTHEADER_OFFSET 0x0CU

#define PDS_FLASH_EFUSE_DEV_INFO_ADDR  (EF_DATA_BASE + 0x74U)
#define PDS_FLASH_EFUSE_SF_SWAP_POS    22U
#define PDS_FLASH_EFUSE_SF_SWAP_MASK   GENMASK(23, 22)
#define PDS_FLASH_EFUSE_FLASH_CFG_POS  26U
#define PDS_FLASH_EFUSE_FLASH_CFG_MASK GENMASK(28, 26)
#define PDS_FLASH_EFUSE_SF_REVERSE     BIT(29)
#define PDS_FLASH_CFG_EXTERNAL         0U
#define PDS_FLASH_PIN_CFG_NORMAL_BASE  1U
#define PDS_FLASH_PIN_CFG_REVERSE_BASE 5U

#define PDS_FLASH_IO_MODE_MASK     GENMASK(3, 0)
#define PDS_FLASH_IO_UNWRAP        BIT(4)
#define PDS_FLASH_IO_QO_MODE       2U
#define PDS_FLASH_IO_QIO_MODE      4U
#define PDS_FLASH_SF_OWNER_SAHB    1U
#define PDS_FLASH_SF_CTRL_SPI_MODE 0U

#define PDS_FLASH_CFG_B2_CLK_DELAY_MASK GENMASK(3, 0)
#define PDS_FLASH_CFG_B2_DO_IDX_POS     4U
#define PDS_FLASH_CFG_B2_DO_IDX_MASK    GENMASK(2, 0)
#define PDS_FLASH_CFG_B3_CLK_INVERT     BIT(0)
#define PDS_FLASH_CFG_B3_RX_CLK_INVERT  BIT(1)
#define PDS_FLASH_CFG_B3_DI_IDX_POS     2U
#define PDS_FLASH_CFG_B3_DI_IDX_MASK    GENMASK(2, 0)
#define PDS_FLASH_CFG_B3_OE_IDX_POS     5U
#define PDS_FLASH_CFG_B3_OE_IDX_MASK    GENMASK(2, 0)

#define PDS_FLASH_L1C_WAY_POS  L1C_WAY_DIS_POS
#define PDS_FLASH_L1C_WAY_MASK GENMASK(3, 0)

/* SF_Ctrl_Cfg_Type — ABI shape consumed by the ROM's SFlash_Init(). */
struct pds_flash_sf_ctrl_cfg {
	uint32_t owner;
	uint32_t sahb_clock;
	uint32_t ahb2sif_mode;
	uint8_t clk_delay;
	uint8_t clk_invert;
	uint8_t rx_clk_invert;
	uint8_t do_delay;
	uint8_t di_delay;
	uint8_t oe_delay;
};

static struct {
	uint8_t flash_cfg[PDS_FLASH_CFG_SIZE];
	uint8_t io_mode;
	uint8_t cont_read;
	uint8_t cache_way_disable;
	uint8_t flash_pin_cfg;
	uint32_t image_offset;
	struct pds_flash_sf_ctrl_cfg sf_ctrl_cfg;
} pds_flash;

static void pds_flash_build_sf_ctrl_cfg(const uint8_t *flash_cfg, struct pds_flash_sf_ctrl_cfg *out)
{
	static const uint8_t delay_lut[8] = {
		0x00U, 0x80U, 0xC0U, 0xE0U, 0xF0U, 0xF8U, 0xFCU, 0xFEU,
	};
	uint8_t idx;

	out->owner = 0U;
	out->sahb_clock = 0U;
	out->ahb2sif_mode = 0U;

	out->clk_delay = flash_cfg[2] & PDS_FLASH_CFG_B2_CLK_DELAY_MASK;
	idx = (flash_cfg[2] >> PDS_FLASH_CFG_B2_DO_IDX_POS) & PDS_FLASH_CFG_B2_DO_IDX_MASK;
	out->do_delay = delay_lut[idx];

	out->clk_invert = flash_cfg[3] & PDS_FLASH_CFG_B3_CLK_INVERT;
	out->rx_clk_invert = (flash_cfg[3] & PDS_FLASH_CFG_B3_RX_CLK_INVERT) >> 1U;

	idx = (flash_cfg[3] >> PDS_FLASH_CFG_B3_DI_IDX_POS) & PDS_FLASH_CFG_B3_DI_IDX_MASK;
	out->di_delay = delay_lut[idx];

	idx = (flash_cfg[3] >> PDS_FLASH_CFG_B3_OE_IDX_POS) & PDS_FLASH_CFG_B3_OE_IDX_MASK;
	out->oe_delay = delay_lut[idx];
}

static void pds_flash_l1c_invalidate_wait(uint32_t saved_cfg)
{
	sys_write32(saved_cfg | L1C_INVALID_EN_MSK, L1C_BASE + L1C_CONFIG_OFFSET);
	while ((sys_read32(L1C_BASE + L1C_CONFIG_OFFSET) & L1C_INVALID_DONE_MSK) == 0U) {
	}
	sys_write32(saved_cfg, L1C_BASE + L1C_CONFIG_OFFSET);
}

static void pds_flash_read_bootheader(uint8_t *dst)
{
	const volatile uint8_t *src;
	unsigned int key;
	uint32_t l1c_cfg;

	key = irq_lock();

	SF_Ctrl_Set_Flash_Image_Offset(0U);
	__asm__ volatile("fence iorw, iorw" ::: "memory");

	l1c_cfg = sys_read32(L1C_BASE + L1C_CONFIG_OFFSET);
	pds_flash_l1c_invalidate_wait(l1c_cfg);

	src = (const volatile uint8_t *)(PDS_FLASH_XIP_BASE + PDS_FLASH_BOOTHEADER_OFFSET);
	for (uint32_t i = 0U; i < PDS_FLASH_CFG_SIZE; i++) {
		dst[i] = src[i];
	}

	SF_Ctrl_Set_Flash_Image_Offset(pds_flash.image_offset);
	__asm__ volatile("fence iorw, iorw" ::: "memory");
	pds_flash_l1c_invalidate_wait(l1c_cfg);

	irq_unlock(key);
}

static uint8_t pds_flash_resolve_pin_cfg(uint32_t dev_info)
{
	uint32_t flash_cfg =
		(dev_info & PDS_FLASH_EFUSE_FLASH_CFG_MASK) >> PDS_FLASH_EFUSE_FLASH_CFG_POS;
	uint32_t sf_swap = (dev_info & PDS_FLASH_EFUSE_SF_SWAP_MASK) >> PDS_FLASH_EFUSE_SF_SWAP_POS;

	if (flash_cfg == PDS_FLASH_CFG_EXTERNAL) {
		return 0U;
	}
	if ((dev_info & PDS_FLASH_EFUSE_SF_REVERSE) == 0U) {
		return (uint8_t)(sf_swap + PDS_FLASH_PIN_CFG_NORMAL_BASE);
	}
	return (uint8_t)(sf_swap + PDS_FLASH_PIN_CFG_REVERSE_BASE);
}

void pds_flash_init(void)
{
	uint32_t l1c_cfg;
	uint32_t dev_info;

	pds_flash.image_offset = SF_Ctrl_Get_Flash_Image_Offset();
	pds_flash_read_bootheader(pds_flash.flash_cfg);

	pds_flash.io_mode = pds_flash.flash_cfg[0];
	pds_flash.cont_read = pds_flash.flash_cfg[1] & 0x1U;
	pds_flash_build_sf_ctrl_cfg(pds_flash.flash_cfg, &pds_flash.sf_ctrl_cfg);

	l1c_cfg = sys_read32(L1C_BASE + L1C_CONFIG_OFFSET);
	pds_flash.cache_way_disable =
		(uint8_t)((l1c_cfg >> PDS_FLASH_L1C_WAY_POS) & PDS_FLASH_L1C_WAY_MASK);

	dev_info = sys_read32(PDS_FLASH_EFUSE_DEV_INFO_ADDR);
	pds_flash.flash_pin_cfg = pds_flash_resolve_pin_cfg(dev_info);
}

void flash_bflb_pm_suspend(void)
{
	SF_Ctrl_Set_Owner(PDS_FLASH_SF_OWNER_SAHB);
	SFlash_Reset_Continue_Read((void *)pds_flash.flash_cfg);
	SFlash_Powerdown();
	GLB_Set_SF_CLK(0U, 0U, 0U);
}

static void pds_flash_resume_abort(void *cfg, uint8_t io_mode)
{
	SF_Ctrl_Set_Owner(PDS_FLASH_SF_OWNER_SAHB);
	SFlash_Restore_From_Powerdown(cfg, pds_flash.cont_read);
	SFlash_Cache_Read_Enable(cfg, io_mode & (uint8_t)PDS_FLASH_IO_MODE_MASK,
				 pds_flash.cont_read, pds_flash.cache_way_disable);
}

static void pds_flash_resume_wakeup(void *cfg, uint8_t io_mode)
{
	uint8_t mode_low = io_mode & (uint8_t)PDS_FLASH_IO_MODE_MASK;
	bool quad_mode = (mode_low == PDS_FLASH_IO_QO_MODE) || (mode_low == PDS_FLASH_IO_QIO_MODE);
	volatile uint32_t pd_delay;

	SF_Cfg_Init_Flash_Gpio(pds_flash.flash_pin_cfg, 0U);
	SFlash_Init(&pds_flash.sf_ctrl_cfg);
	SFlash_Releae_Powerdown(cfg);

	pd_delay = (uint32_t)pds_flash.flash_cfg[PDS_FLASH_CFG_PD_DELAY_OFFSET] * 8U;
	while (pd_delay-- != 0U) {
	}

	SFlash_Reset_Continue_Read(cfg);
	SFlash_Software_Reset(cfg);

	SFlash_Write_Enable(cfg);
	SFlash_DisableBurstWrap(cfg);
	SFlash_SetSPIMode(PDS_FLASH_SF_CTRL_SPI_MODE);

	if (quad_mode) {
		SFlash_Qspi_Enable(cfg);
	}

	if ((io_mode & PDS_FLASH_IO_UNWRAP) != 0U) {
		L1C_Set_Wrap(0U);
	} else {
		L1C_Set_Wrap(1U);
		SFlash_Write_Enable(cfg);
		if (quad_mode) {
			SFlash_SetBurstWrap(cfg);
		}
	}

	if (pds_flash.cont_read != 0U) {
		uint32_t dummy[1];

		SF_Ctrl_Set_Owner(PDS_FLASH_SF_OWNER_SAHB);
		SFlash_Read(cfg, mode_low, 1U, 0U, (uint8_t *)dummy, sizeof(dummy));
	}

	SF_Ctrl_Set_Flash_Image_Offset(pds_flash.image_offset);
	SFlash_Cache_Read_Enable(cfg, mode_low, pds_flash.cont_read, pds_flash.cache_way_disable);
}

void flash_bflb_pm_resume(void)
{
	void *cfg = (void *)pds_flash.flash_cfg;
	uint8_t io_mode = pds_flash.io_mode;

	GLB_Set_SF_CLK(1U, 0U, 0U);

	if (SF_Ctrl_Get_Flash_Image_Offset() != 0U) {
		pds_flash_resume_abort(cfg, io_mode);
	} else {
		pds_flash_resume_wakeup(cfg, io_mode);
	}
}
