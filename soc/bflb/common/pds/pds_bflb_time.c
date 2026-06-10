/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>

#include <bflb_soc.h>
#include <hbn_reg.h>

#include "pds_bflb_time.h"

#define MTIME_BASE DT_REG_ADDR_BY_NAME(DT_NODELABEL(mtimer), mtime)
#define MTIME_LO   (MTIME_BASE)
#define MTIME_HI   (MTIME_BASE + 4U)

#define MTIMECMP_BASE DT_REG_ADDR_BY_NAME(DT_NODELABEL(mtimer), mtimecmp)
#define MTIMECMP_LO   (MTIMECMP_BASE)
#define MTIMECMP_HI   (MTIMECMP_BASE + 4U)

#define PDS_WARMUP_MIN_TICKS_32K 66U
#define PDS_RC32K_CAL_WINDOW_DIV 25U /* ~40 ms per window at mtimer_freq */
#define PDS_RC32K_FREQ_MIN       30000U
#define PDS_RC32K_FREQ_MAX       35000U
#define PDS_RC32K_EMA_WEIGHT     7U /* EMA = (old*7 + new) / 8 */
#define PDS_RC32K_EMA_DIVISOR    8U
#define PDS_RC32K_MIN_ACTIVE_DIV 33U /* ~30 ms minimum active interval */
#define PDS_RC32K_MAX_ACTIVE_S   5U  /* reject stale anchor after 5 s */

/* RTC_TIME_L/H at 0x0C/0x10 are latch regs, not the compare regs at 0x04/0x08. */
static uint64_t hbn_rtc_latch_and_read(void)
{
	uint32_t tmp;
	uint32_t lo;
	uint32_t hi;

	tmp = sys_read32(HBN_BASE + HBN_RTC_TIME_H_OFFSET);
	sys_write32(tmp | HBN_RTC_TIME_LATCH_MSK, HBN_BASE + HBN_RTC_TIME_H_OFFSET);
	sys_write32(tmp & HBN_RTC_TIME_LATCH_UMSK, HBN_BASE + HBN_RTC_TIME_H_OFFSET);
	lo = sys_read32(HBN_BASE + HBN_RTC_TIME_L_OFFSET);
	hi = sys_read32(HBN_BASE + HBN_RTC_TIME_H_OFFSET) & HBN_RTC_TIME_H_MASK;
	return ((uint64_t)hi << 32) | lo;
}

uint64_t bflb_pds_hbn_rtc_read(void)
{
	uint64_t val;
	uint64_t val2;

	do {
		val = hbn_rtc_latch_and_read();
		val2 = hbn_rtc_latch_and_read();
	} while ((val2 < val) || ((val2 - val) > 1U));

	return val2;
}

uint64_t bflb_pds_mtime_read(void)
{
	uint32_t hi;
	uint32_t lo;
	uint32_t hi2;

	do {
		hi = sys_read32(MTIME_HI);
		lo = sys_read32(MTIME_LO);
		hi2 = sys_read32(MTIME_HI);
	} while (hi != hi2);

	return ((uint64_t)hi << 32) | lo;
}

void bflb_pds_mtime_write(uint64_t val)
{
	/* hi=0 first to avoid spurious overflow. */
	sys_write32(0U, MTIME_HI);
	sys_write32((uint32_t)val, MTIME_LO);
	sys_write32((uint32_t)(val >> 32U), MTIME_HI);
}

/*
 * RC32K frequency calibration state. The internal RC32K drifts ~3-5% with
 * temperature; we track the live frequency by comparing mtime (XTAL-derived,
 * accurate) with the HBN RTC.
 */
static uint32_t pds_rc32k_freq = HBN_RTC_FREQ;
static uint64_t pds_cal_mtime;
static uint64_t pds_cal_rtc;
static bool pds_cal_anchored;
static bool pds_cal_valid;

static uint32_t pds_rc32k_measure_once(uint32_t mtimer_freq, uint32_t window)
{
	uint64_t m0, m1, r0, r1;

	r0 = bflb_pds_hbn_rtc_read();
	m0 = bflb_pds_mtime_read();
	do {
		m1 = bflb_pds_mtime_read();
	} while ((m1 - m0) < window);
	r1 = bflb_pds_hbn_rtc_read();

	if (r1 > r0 && m1 > m0) {
		return (uint32_t)((r1 - r0) * mtimer_freq / (m1 - m0));
	}
	return 0U;
}

static void pds_rc32k_calibrate_window(uint32_t mtimer_freq)
{
	uint32_t window = mtimer_freq / PDS_RC32K_CAL_WINDOW_DIV;
	uint32_t a = pds_rc32k_measure_once(mtimer_freq, window);
	uint32_t b = pds_rc32k_measure_once(mtimer_freq, window);
	uint32_t c = pds_rc32k_measure_once(mtimer_freq, window);
	/* Median of three rejects a single noisy window; a bad initial F_rc would
	 * mis-scale DEEPSLSTAT and drift the BLE anchor out within a few cycles.
	 */
	uint32_t med =
		(a > b) ? ((b > c) ? b : ((a > c) ? c : a)) : ((a > c) ? a : ((b > c) ? c : b));

	if (med > PDS_RC32K_FREQ_MIN && med < PDS_RC32K_FREQ_MAX) {
		pds_rc32k_freq = med;
	}
}

uint32_t bflb_pds_calc_sleep_ticks(void)
{
	uint64_t mtime_before = bflb_pds_mtime_read();
	uint64_t rtc_before = bflb_pds_hbn_rtc_read();
	uint64_t mtimecmp_val =
		(uint64_t)sys_read32(MTIMECMP_LO) | ((uint64_t)sys_read32(MTIMECMP_HI) << 32);
	uint32_t mtimer_freq = sys_clock_hw_cycles_per_sec();
	int64_t remaining_cycles;
	uint32_t sleep_32k;

	if (!pds_cal_valid) {
		pds_rc32k_calibrate_window(mtimer_freq);
		pds_cal_valid = true;
	} else if (pds_cal_anchored) {
		uint64_t mt_active = mtime_before - pds_cal_mtime;
		uint64_t rtc_active = rtc_before - pds_cal_rtc;

		if (mt_active >= mtimer_freq / PDS_RC32K_MIN_ACTIVE_DIV &&
		    mt_active < (uint64_t)mtimer_freq * PDS_RC32K_MAX_ACTIVE_S && rtc_active > 0U) {
			uint32_t f = (uint32_t)(rtc_active * mtimer_freq / mt_active);

			pds_rc32k_freq = (uint32_t)(((uint64_t)pds_rc32k_freq * PDS_RC32K_EMA_WEIGHT +
						     f) /
						    PDS_RC32K_EMA_DIVISOR);
		}
	}
	pds_cal_anchored = false;

	remaining_cycles = (int64_t)(mtimecmp_val - mtime_before);
	if (remaining_cycles <= 0) {
		return 0U;
	}

	sleep_32k = (uint32_t)((uint64_t)remaining_cycles * pds_rc32k_freq / mtimer_freq);
	if (sleep_32k <= PDS_WARMUP_MIN_TICKS_32K) {
		return 0U;
	}
	return sleep_32k;
}

void bflb_pds_time_compensate(uint64_t rtc_before, uint64_t mtime_before)
{
	uint64_t rtc_after = bflb_pds_hbn_rtc_read();
	uint64_t mtimer_delta =
		(rtc_after - rtc_before) * sys_clock_hw_cycles_per_sec() / pds_rc32k_freq;
	uint64_t new_mtime = mtime_before + mtimer_delta;

	bflb_pds_mtime_write(new_mtime);

	pds_cal_mtime = new_mtime;
	pds_cal_rtc = rtc_after;
	pds_cal_anchored = true;
}
