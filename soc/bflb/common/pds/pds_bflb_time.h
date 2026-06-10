/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_BFLB_COMMON_PDS_PDS_BFLB_TIME_H_
#define SOC_BFLB_COMMON_PDS_PDS_BFLB_TIME_H_

#include <stdint.h>
#include <zephyr/sys/util.h>

/* HBN RTC and its read protocol are identical across the BFLB family. */
#define HBN_RTC_FREQ        32768U
#define HBN_RTC_TIME_H_BITS 8U
#define HBN_RTC_TIME_H_MASK GENMASK(HBN_RTC_TIME_H_BITS - 1U, 0)

uint64_t bflb_pds_mtime_read(void);
void bflb_pds_mtime_write(uint64_t val);

/* Glitch-free read of the 40-bit HBN RTC counter. */
uint64_t bflb_pds_hbn_rtc_read(void);

/*
 * RC32K-calibrated sleep policy. Tracks the live RC32K frequency by comparing
 * mtime (XTAL-derived) against the HBN RTC, then converts the pending mtimer
 * deadline into 32K sleep ticks. Returns 0 if the deadline is too soon for PDS.
 */
uint32_t bflb_pds_calc_sleep_ticks(void);

/*
 * Post-wake: advance mtime by the elapsed HBN-RTC delta (scaled by the tracked
 * RC32K frequency) and re-anchor the calibrator. rtc_before/mtime_before are
 * the counter values captured immediately before sleep.
 */
void bflb_pds_time_compensate(uint64_t rtc_before, uint64_t mtime_before);

#endif /* SOC_BFLB_COMMON_PDS_PDS_BFLB_TIME_H_ */
