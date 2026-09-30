/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * BL616/BL618 RF control block (MIX_BASE) registers used to configure the
 * WLAN receiver and to dump raw RX samples into WRAM.
 *
 * Not documented by Bouffalo Lab. Offsets and bit meanings were recovered
 * from the manufacturing commands (cmd_rf_rx(), cmd_sram_rx(),
 * cmd_dump_rf_register()) of libbl616_phyrf_mfg.a in the Bouffalo SDK, and
 * characterized on a BL618: at 80 MS/s the samples are 11-bit signed Q
 * (bits 15:0) and I (bits 31:16), sign-extended. Fields whose function is
 * not known are only described by the values the vendor code writes.
 */

#ifndef BL61X_SDR_RF_DUMP_REG_H_
#define BL61X_SDR_RF_DUMP_REG_H_

/* Register offsets from MIX_BASE */

#define RF_PU_CTRL_OFFSET   (0x02C) /* Power-up and RX gain source select */
#define RF_RX_CFG_OFFSET    (0x048) /* RX gain (fixed by rf_rx) */
#define RF_ADC_CFG_OFFSET   (0x088) /* RX ADC configuration */
#define RF_DUMP_CFG_OFFSET  (0x220) /* DFE forces and RX test (dump) tap select */
#define RF_DUMP_SRAM_OFFSET (0x23C) /* Dump SRAM bank select */
#define RF_DUMP_CTRL_OFFSET (0x240) /* Dump start/done */
#define RF_DUMP_ADDR_OFFSET (0x244) /* Dump start/end word addresses */
#define RF_DUMP_STAT_OFFSET (0x248) /* Read-only dump status */

/* Register bit fields */

/* 0x02C : rf_pu_ctrl */
#define RF_PU_CTRL_RXGAIN_HW (1U << 1U) /* RX gain from the PHY instead of 0x048 */

/*
 * 0x048 : rf_rx_cfg, RX gain fields (vendor "rxgain" command: gc_lna,
 * gc_rmxgm, gc_rbb1, gc_rbb2), and the mask and value written by rf_rx
 */
#define RF_RX_GC_LNA_POS     (0U)
#define RF_RX_GC_LNA_MAX     (7U)
#define RF_RX_GC_RMXGM_POS   (3U)
#define RF_RX_GC_RMXGM_MAX   (3U)
#define RF_RX_GC_RBB1_POS    (8U)
#define RF_RX_GC_RBB1_MAX    (3U)
#define RF_RX_GC_RBB2_POS    (12U)
#define RF_RX_GC_RBB2_MAX    (7U)
#define RF_RX_CFG_RX_MSK     (0x0000731FU)
#define RF_RX_CFG_RX_VAL     (0x0000311FU)

/* 0x088 : rf_adc_cfg, field mask and value written by rf_rx */
#define RF_ADC_CFG_RX_MSK    (0x03000000U)
#define RF_ADC_CFG_RX_VAL    (0x03000000U)

/* 0x220 : rf_dump_cfg */
#define RF_DUMP_CFG_CLK_EN   (0x000001E3U)
#define RF_DUMP_SEL_POS      (9U)
#define RF_DUMP_SEL_LEN      (2U)
#define RF_DUMP_SEL_MSK      (((1U << RF_DUMP_SEL_LEN) - 1) << RF_DUMP_SEL_POS)
#define RF_DUMP_SEL_UMSK     (~(((1U << RF_DUMP_SEL_LEN) - 1) << RF_DUMP_SEL_POS))

/*
 * 0x23C : rf_dump_sram
 * Mode 2 and 3 wait for an external trigger. Bank 1 is WRAM, one sample per
 * word (CPU view 0x23010000); bank 0 is OCRAM + 128 KiB with one sample every
 * 8 bytes (CPU view 0x22FE0000).
 */
#define RF_DUMP_SRAM_MODE_MSK (0x00030000U)
#define RF_DUMP_SRAM_WRAM     (1U << 18U)

/* 0x240 : rf_dump_ctrl */
#define RF_DUMP_DONE          (1U << 0U) /* Finite dump completed */
#define RF_DUMP_START         (1U << 1U)
#define RF_DUMP_MODE          (1U << 2U) /* Continuous: wrap until START clears */
#define RF_DUMP_CLR           (1U << 3U) /* Pulsed after completion */
#define RF_DUMP_LAPS_POS      (16U)      /* Completed passes over the window */
#define RF_DUMP_LAPS_LEN      (8U)
#define RF_DUMP_LAPS_MSK      (((1U << RF_DUMP_LAPS_LEN) - 1) << RF_DUMP_LAPS_POS)

/* 0x244 : rf_dump_addr, 32-bit word offsets from the selected bank */
#define RF_DUMP_END_POS       (0U)
#define RF_DUMP_END_LEN       (16U)
#define RF_DUMP_END_MSK       (((1U << RF_DUMP_END_LEN) - 1) << RF_DUMP_END_POS)
#define RF_DUMP_START_POS     (16U)
#define RF_DUMP_START_LEN     (16U)
#define RF_DUMP_START_MSK     (((1U << RF_DUMP_START_LEN) - 1) << RF_DUMP_START_POS)

#endif /* BL61X_SDR_RF_DUMP_REG_H_ */
