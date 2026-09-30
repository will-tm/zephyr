/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BL61X_SDR_RF_H_
#define BL61X_SDR_RF_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Continuous ring: 128 KiB at the start of the WRAM bank */
#define RF_RING_WORDS        32768U
/* Finite dumps: the whole 160 KiB WRAM bank */
#define RF_WINDOW_WORDS      40960U

/* ESP-SDR rate indices: 80, 40, 20, 10, 8, 4, 16 MS/s. Measured 80.4 MS/s. */
#define RF_RATE_INDEX        0U
#define RF_SAMPLE_RATE_HZ    80000000U

/* Tuning goes through the WLAN PHY; PLL lock range unknown */
#define RF_FREQ_MIN_MHZ      100U
#define RF_FREQ_MAX_MHZ      4095U
#define RF_FREQ_DEFAULT_MHZ  2412U

#define RF_DUMP_SEL_MAX      3U

/*
 * Dump taps, measured: 0 = 80 MS/s, 11 bits (ADC); 1 = 40 MS/s, 12 bits
 * (decimated); 2 = 16 MS/s, idle in WLAN receive; 3 = 80 MS/s, 12 bits
 * (filtered). All hold sign-extended I in bits 31:16 and Q in bits 15:0.
 */
#define RF_DUMP_SEL_80M      0U
#define RF_DUMP_SEL_40M      1U
#define RF_DUMP_WIDTH_80M    11U
#define RF_DUMP_WIDTH_40M    12U

/* RX gain ladder: uncalibrated indices, monotonic in gain */
#define RF_GAIN_STEPS        21U
#define RF_GAIN_DEFAULT      10U

int rf_init(void);
void rf_tune(uint32_t freq_mhz);
uint32_t rf_freq_mhz(void);
void rf_dump_sel_set(uint32_t sel);
uint32_t rf_dump_sel_get(void);
uint32_t rf_dump_cfg_read(void);
uint32_t rf_dump_ctrl_read(void);
void rf_gain_set(uint32_t index);
uint32_t rf_gain_get(void);
/*
 * One software AGC step from a short finite dump: lowers the gain when the
 * peak clips, raises it when the peak is small. Returns true when settled.
 */
bool rf_agc_step(void);
/* Peak |I|,|Q| of the words and AGC gain update; returns true when settled */
bool rf_agc_update(const volatile uint32_t *words, uint32_t n);

/* Finite capture of n words at the start of the dump window */
volatile uint32_t *rf_snapshot_buf(void);
bool rf_snapshot(uint32_t n);
/* Same from dump tap sel */
bool rf_snapshot_sel(uint32_t n, uint32_t sel);
/* Cached view of the first n words of a completed dump */
const uint32_t *rf_snapshot_cached(uint32_t n);

/* Continuous capture over the whole dump window */
volatile uint32_t *rf_ring_base(void);
void rf_ring_start(void);
void rf_ring_stop(void);
bool rf_ring_running(void);
uint32_t rf_ring_store_addr_raw(void);
void rf_ring_mode_set(uint32_t ctrl_bits, uint32_t sram_bits);
void rf_ring_mode_get(uint32_t *ctrl_bits, uint32_t *sram_bits);
void rf_ring_ptr_set(uint32_t offset, uint32_t lsb, uint32_t width);
void rf_ring_ptr_get(uint32_t *offset, uint32_t *lsb, uint32_t *width);
int rf_ring_probe(char *out, size_t size);
/* Starts the ring and samples its status every interval_us */
int rf_ring_trace(char *out, size_t size, uint32_t samples, uint32_t interval_us);

/* Vendor CW transmit test (2402..2484 MHz) for bring-up */
void rf_cw_start(uint32_t freq_mhz, int32_t pwr_dbm);
void rf_cw_stop(void);

/* Raw RF control block access for bring-up, offsets from MIX_BASE */
bool rf_reg_read(uint32_t offset, uint32_t *value);
bool rf_reg_write(uint32_t offset, uint32_t value);

#endif /* BL61X_SDR_RF_H_ */
