/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * BL61x RF control block access: receiver force, finite dump and continuous
 * ring dump. The finite sequences replicate the vendor "rf_rx" and "sram_rx"
 * manufacturing commands. The vendor code does not use the continuous mode
 * (RF_DUMP_MODE), which was characterized on hardware: it wraps over the
 * window and counts laps, but exposes no fine write pointer. The mode bits
 * and an optional pointer register remain runtime settings (RINGMODE,
 * RINGPTR).
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <bflb_soc.h>
#include <wifi/phy.h>

#include "rf.h"
#include "rf_dump_reg.h"

#define RF_REG(off)          (MIX_BASE + (off))

#define RING_NODE            DT_NODELABEL(iq_ring)
#define RING_CPU_BASE        DT_REG_ADDR(RING_NODE)
#define RING_BYTES           DT_REG_SIZE(RING_NODE)
/* Cached bus remap view of the same memory */
#define RING_CACHED_BASE     (RING_CPU_BASE + 0x40000000U)
#define BYTES_PER_WORD       4U

#define DUMP_TIMEOUT_US      20000U
#define DUMP_POLL_US         10U
/* rfc_rf_fsm_force() states */
#define RF_FSM_RX            3U
#define RF_FSM_RELEASE       15U

#define US_PER_S             1000000ULL
/*
 * The time-derived write position trails by one chunk: the engine starts
 * writing some time after START, and the estimate must never lead the writer.
 */
#define RING_TIME_LAG_WORDS  1024ULL

#define PTR_WIDTH_MAX        16U
#define WORD_BITS            32U
#define RF_REG_SPAN          0x1000U

#define PROBE_POISON         0xa5a0055aU

/*
 * Software AGC on 11-bit samples. Besides the ADC full scale, an analog
 * stage limits at a gain dependent level (seen at 200..300 codes), so
 * clipping is detected as a pile-up of samples at the peak magnitude.
 */
#define AGC_PROBE_WORDS      4096U
#define AGC_ADC_FULL_SCALE   1000
#define AGC_PILEUP_MARGIN    2
#define AGC_PILEUP_MIN_LEVEL 128
#define AGC_PILEUP_DIVISOR   512U
#define AGC_HIGH_LEVEL       600
#define AGC_LOW_LEVEL        128
#define AGC_BIG_STEP         3U
#define AGC_UP_PROBES        4U
#define HALF_WORD_BITS       16U
#define PROBE_FIRST_OFFSET   0x000U
#define PROBE_LAST_OFFSET    0x3fcU
#define PROBE_REGS           (((PROBE_LAST_OFFSET - PROBE_FIRST_OFFSET) / BYTES_PER_WORD) + 1U)
#define PROBE_SAMPLES        16U
#define PROBE_SAMPLE_US      7U
#define PROBE_LAP_WAIT_US    2000U
#define PROBE_REWRAP_WORDS   1024U
#define PROBE_TRACE_WORD     100U
#define PROBE_STATUS_OFFSET  0x248U

BUILD_ASSERT(RING_BYTES == (RF_WINDOW_WORDS * BYTES_PER_WORD), "iq_ring must cover the WRAM bank");
BUILD_ASSERT(RF_RING_WORDS <= RF_WINDOW_WORDS, "the ring must fit in the dump window");

extern int bflb_rf_init(void);
extern void rf_pri_cw_start(int16_t pwr_dbm, uint16_t freq_mhz);
extern void rf_pri_cw_stop(void);
extern void rfc_rf_fsm_force(uint32_t state);

struct gain_step {
	uint8_t lna;
	uint8_t rmxgm;
	uint8_t rbb1;
	uint8_t rbb2;
};

/* LNA first, so strong signals are attenuated before the mixer */
static const struct gain_step gain_table[RF_GAIN_STEPS] = {
	{0U, 0U, 0U, 0U}, {1U, 0U, 0U, 0U}, {2U, 0U, 0U, 0U}, {3U, 0U, 0U, 0U},
	{4U, 0U, 0U, 0U}, {5U, 0U, 0U, 0U}, {6U, 0U, 0U, 0U}, {7U, 0U, 0U, 0U},
	{7U, 1U, 0U, 0U}, {7U, 2U, 0U, 0U}, {7U, 3U, 0U, 0U}, {7U, 3U, 1U, 0U},
	{7U, 3U, 2U, 0U}, {7U, 3U, 3U, 0U}, {7U, 3U, 3U, 1U}, {7U, 3U, 3U, 2U},
	{7U, 3U, 3U, 3U}, {7U, 3U, 3U, 4U}, {7U, 3U, 3U, 5U}, {7U, 3U, 3U, 6U},
	{7U, 3U, 3U, 7U},
};

static uint32_t freq_mhz = RF_FREQ_DEFAULT_MHZ;
static uint32_t gain_index = RF_GAIN_DEFAULT;
static uint32_t agc_low_probes;
static uint32_t dump_sel;
static uint32_t ring_ctrl_bits = RF_DUMP_MODE;
static uint32_t ring_sram_bits;
static uint32_t ptr_offset;
static uint32_t ptr_lsb;
static uint32_t ptr_width = PTR_WIDTH_MAX;
static uint64_t ring_t0_us;

static uint32_t probe_first[PROBE_REGS];
static uint32_t probe_last[PROBE_REGS];
static uint8_t probe_changes[PROBE_REGS];

static void rf_rx_start(void);
static void gain_apply(void);
static int32_t sample_mag(uint32_t half);
static void dump_cfg_apply(uint32_t sel);
static void dump_addr_set(uint32_t start, uint32_t end);
static void dump_release(void);
static uint64_t now_us(void);
static uint32_t count_word(uint32_t first, uint32_t count, uint32_t value);

/*
 * WLAN receive on freq_mhz: the PHY programs the channel as for a WLAN
 * link, then the RF state machine is held in RX so that the front end
 * (T/R switch, LNA, mixer) stays connected without the modem.
 */
static void rf_rx_start(void)
{
	static struct phy_mac_chan_op chan;
	uint32_t val;

	chan.band = E_PHY_BAND_2G4;
	chan.type = E_PHY_CHNL_BW_20;
	chan.prim20_freq = (uint16_t)freq_mhz;
	chan.center1_freq = (uint16_t)freq_mhz;
	phy_set_channel(&chan, 0U);
	rfc_rf_fsm_force(RF_FSM_RX);

	/* The PHY hands RX gain to its AGC, which does not run: take it back */
	val = sys_read32(RF_REG(RF_PU_CTRL_OFFSET));
	sys_write32(val & ~RF_PU_CTRL_RXGAIN_HW, RF_REG(RF_PU_CTRL_OFFSET));
	gain_apply();

	val = sys_read32(RF_REG(RF_ADC_CFG_OFFSET));
	val = (val & ~RF_ADC_CFG_RX_MSK) | RF_ADC_CFG_RX_VAL;
	sys_write32(val, RF_REG(RF_ADC_CFG_OFFSET));

	val = sys_read32(RF_REG(RF_DUMP_CFG_OFFSET));
	sys_write32(val | RF_DUMP_CFG_CLK_EN, RF_REG(RF_DUMP_CFG_OFFSET));
}

/* The vendor rf_rx writes fixed gain fields; the ladder replaces them */
static void gain_apply(void)
{
	const struct gain_step *g = &gain_table[gain_index];
	uint32_t val = sys_read32(RF_REG(RF_RX_CFG_OFFSET)) & ~RF_RX_CFG_RX_MSK;

	val |= ((uint32_t)g->lna << RF_RX_GC_LNA_POS) | ((uint32_t)g->rmxgm << RF_RX_GC_RMXGM_POS) |
	       ((uint32_t)g->rbb1 << RF_RX_GC_RBB1_POS) | ((uint32_t)g->rbb2 << RF_RX_GC_RBB2_POS);
	sys_write32(val, RF_REG(RF_RX_CFG_OFFSET));
}

static void dump_cfg_apply(uint32_t sel)
{
	uint32_t val = sys_read32(RF_REG(RF_DUMP_CFG_OFFSET));

	val = (val & RF_DUMP_SEL_UMSK) | ((sel << RF_DUMP_SEL_POS) & RF_DUMP_SEL_MSK);
	sys_write32(val, RF_REG(RF_DUMP_CFG_OFFSET));
}

static void dump_addr_set(uint32_t start, uint32_t end)
{
	uint32_t val = ((start << RF_DUMP_START_POS) & RF_DUMP_START_MSK) |
		       ((end << RF_DUMP_END_POS) & RF_DUMP_END_MSK);

	sys_write32(val, RF_REG(RF_DUMP_ADDR_OFFSET));
}

static void dump_release(void)
{
	uint32_t val = sys_read32(RF_REG(RF_DUMP_CTRL_OFFSET)) & ~(RF_DUMP_START | ring_ctrl_bits);

	sys_write32(val | RF_DUMP_CLR, RF_REG(RF_DUMP_CTRL_OFFSET));
	sys_write32(val & ~RF_DUMP_CLR, RF_REG(RF_DUMP_CTRL_OFFSET));
}

static uint64_t now_us(void)
{
	return k_cyc_to_us_floor64(k_cycle_get_64());
}

static uint32_t count_word(uint32_t first, uint32_t count, uint32_t value)
{
	volatile uint32_t *ring = rf_ring_base();
	uint32_t found = 0U;

	for (uint32_t i = first; i < (first + count); i++) {
		if (ring[i] == value) {
			found++;
		}
	}

	return found;
}

int rf_init(void)
{
	int ret = bflb_rf_init();

	if (ret == 0) {
		rf_tune(RF_FREQ_DEFAULT_MHZ);
	}

	return ret;
}

void rf_tune(uint32_t mhz)
{
	freq_mhz = mhz;
	rf_rx_start();
}

uint32_t rf_freq_mhz(void)
{
	return freq_mhz;
}

void rf_dump_sel_set(uint32_t sel)
{
	dump_sel = MIN(sel, RF_DUMP_SEL_MAX);
}

uint32_t rf_dump_sel_get(void)
{
	return dump_sel;
}

uint32_t rf_dump_cfg_read(void)
{
	return sys_read32(RF_REG(RF_DUMP_CFG_OFFSET));
}

uint32_t rf_dump_ctrl_read(void)
{
	return sys_read32(RF_REG(RF_DUMP_CTRL_OFFSET));
}

void rf_gain_set(uint32_t index)
{
	gain_index = MIN(index, RF_GAIN_STEPS - 1U);
	gain_apply();
}

uint32_t rf_gain_get(void)
{
	return gain_index;
}

static int32_t sample_mag(uint32_t half)
{
	/* Sign-extended 16-bit half word */
	int32_t v = (int32_t)(int16_t)(uint16_t)(half & UINT16_MAX);

	return (v < 0) ? -v : v;
}

bool rf_agc_update(const volatile uint32_t *words, uint32_t n)
{
	int32_t peak = 0;
	uint32_t pileup = 0U;
	bool clipped;

	for (uint32_t k = 0U; k < n; k++) {
		uint32_t w = words[k];

		peak = MAX(peak, MAX(sample_mag(w), sample_mag(w >> HALF_WORD_BITS)));
	}
	for (uint32_t k = 0U; k < n; k++) {
		uint32_t w = words[k];

		if (sample_mag(w) >= (peak - AGC_PILEUP_MARGIN)) {
			pileup++;
		}
		if (sample_mag(w >> HALF_WORD_BITS) >= (peak - AGC_PILEUP_MARGIN)) {
			pileup++;
		}
	}

	clipped = (peak >= AGC_ADC_FULL_SCALE) ||
		  ((peak >= AGC_PILEUP_MIN_LEVEL) && (pileup > (n / AGC_PILEUP_DIVISOR)));
	if (clipped || (peak >= AGC_HIGH_LEVEL)) {
		agc_low_probes = 0U;
		if (gain_index == 0U) {
			return true;
		}
		uint32_t step = clipped ? MIN(gain_index, AGC_BIG_STEP) : 1U;

		rf_gain_set(gain_index - step);
		return false;
	}
	if ((peak < AGC_LOW_LEVEL) && (gain_index < (RF_GAIN_STEPS - 1U))) {
		agc_low_probes++;
		if (agc_low_probes >= AGC_UP_PROBES) {
			agc_low_probes = 0U;
			rf_gain_set(gain_index + 1U);
			return false;
		}
		return true;
	}
	agc_low_probes = 0U;

	return true;
}

bool rf_agc_step(void)
{
	if (!rf_snapshot(AGC_PROBE_WORDS)) {
		return true;
	}

	return rf_agc_update(rf_snapshot_buf(), AGC_PROBE_WORDS);
}

volatile uint32_t *rf_snapshot_buf(void)
{
	return rf_ring_base();
}

const uint32_t *rf_snapshot_cached(uint32_t n)
{
	/* The engine wrote behind the cache; nothing is written through this view */
	(void)sys_cache_data_invd_range((void *)RING_CACHED_BASE, n * BYTES_PER_WORD);

	return (const uint32_t *)RING_CACHED_BASE;
}

bool rf_snapshot(uint32_t n)
{
	return rf_snapshot_sel(n, dump_sel);
}

/* Dump sequence of the vendor "sram_rx" manufacturing command */
bool rf_snapshot_sel(uint32_t n, uint32_t sel)
{
	uint32_t val;
	bool done;

	dump_cfg_apply(sel);

	val = sys_read32(RF_REG(RF_DUMP_SRAM_OFFSET));
	val = (val & ~RF_DUMP_SRAM_MODE_MSK) | RF_DUMP_SRAM_WRAM;
	sys_write32(val, RF_REG(RF_DUMP_SRAM_OFFSET));

	dump_addr_set(0U, n);

	val = sys_read32(RF_REG(RF_DUMP_CTRL_OFFSET)) & ~RF_DUMP_MODE;
	sys_write32(val, RF_REG(RF_DUMP_CTRL_OFFSET));
	sys_write32(val | RF_DUMP_START, RF_REG(RF_DUMP_CTRL_OFFSET));

	done = WAIT_FOR((sys_read32(RF_REG(RF_DUMP_CTRL_OFFSET)) & RF_DUMP_DONE) != 0U,
			DUMP_TIMEOUT_US, k_busy_wait(DUMP_POLL_US));

	dump_release();

	return done;
}

volatile uint32_t *rf_ring_base(void)
{
	return (volatile uint32_t *)RING_CPU_BASE;
}

void rf_ring_start(void)
{
	uint32_t val;

	dump_cfg_apply(dump_sel);

	val = sys_read32(RF_REG(RF_DUMP_SRAM_OFFSET)) & ~RF_DUMP_SRAM_MODE_MSK;
	val |= RF_DUMP_SRAM_WRAM | (ring_sram_bits & RF_DUMP_SRAM_MODE_MSK);
	sys_write32(val, RF_REG(RF_DUMP_SRAM_OFFSET));

	dump_addr_set(0U, RF_RING_WORDS);

	val = sys_read32(RF_REG(RF_DUMP_CTRL_OFFSET)) & ~(RF_DUMP_START | RF_DUMP_MODE);
	sys_write32(val | ring_ctrl_bits, RF_REG(RF_DUMP_CTRL_OFFSET));
	ring_t0_us = now_us();
	sys_write32(val | ring_ctrl_bits | RF_DUMP_START, RF_REG(RF_DUMP_CTRL_OFFSET));
}

void rf_ring_stop(void)
{
	dump_release();
}

bool rf_ring_running(void)
{
	return (sys_read32(RF_REG(RF_DUMP_CTRL_OFFSET)) & RF_DUMP_START) != 0U;
}

/*
 * Word index of the writer in the ring. Without a known pointer register the
 * position is derived from time: the ADC and the machine timer both run from
 * the crystal, so only the start latency is uncertain.
 */
uint32_t rf_ring_store_addr_raw(void)
{
	uint64_t elapsed;

	if (ptr_offset != 0U) {
		uint32_t reg = sys_read32(RF_REG(ptr_offset));

		return (reg >> ptr_lsb) & (uint32_t)(BIT64(ptr_width) - 1U);
	}

	elapsed = (now_us() - ring_t0_us) * RF_SAMPLE_RATE_HZ / US_PER_S;
	elapsed = (elapsed > RING_TIME_LAG_WORDS) ? (elapsed - RING_TIME_LAG_WORDS) : 0U;

	return (uint32_t)(elapsed % RF_RING_WORDS);
}

void rf_ring_mode_set(uint32_t ctrl_bits, uint32_t sram_bits)
{
	ring_ctrl_bits = ctrl_bits & ~(RF_DUMP_START | RF_DUMP_DONE | RF_DUMP_CLR);
	ring_sram_bits = sram_bits & RF_DUMP_SRAM_MODE_MSK;
}

void rf_ring_mode_get(uint32_t *ctrl_bits, uint32_t *sram_bits)
{
	*ctrl_bits = ring_ctrl_bits;
	*sram_bits = ring_sram_bits;
}

void rf_ring_ptr_set(uint32_t offset, uint32_t lsb, uint32_t width)
{
	if ((offset < RF_REG_SPAN) && ((offset % BYTES_PER_WORD) == 0U) && (width > 0U) &&
	    (width <= PTR_WIDTH_MAX) && ((lsb + width) <= WORD_BITS)) {
		ptr_offset = offset;
		ptr_lsb = lsb;
		ptr_width = width;
	}
}

void rf_ring_ptr_get(uint32_t *offset, uint32_t *lsb, uint32_t *width)
{
	*offset = ptr_offset;
	*lsb = ptr_lsb;
	*width = ptr_width;
}

/*
 * Runs the ring for a few laps and reports whether the engine wraps and
 * which RF registers change meanwhile (write pointer candidates).
 */
int rf_ring_probe(char *out, size_t size)
{
	volatile uint32_t *ring = rf_ring_base();
	uint32_t lap_unwritten;
	uint32_t rewrap_unwritten;
	int len;

	for (uint32_t i = 0U; i < RF_RING_WORDS; i++) {
		ring[i] = PROBE_POISON;
	}
	for (uint32_t r = 0U; r < PROBE_REGS; r++) {
		probe_changes[r] = 0U;
	}

	rf_ring_start();
	for (uint32_t s = 0U; s < PROBE_SAMPLES; s++) {
		for (uint32_t r = 0U; r < PROBE_REGS; r++) {
			uint32_t off = PROBE_FIRST_OFFSET + (r * BYTES_PER_WORD);
			uint32_t val = sys_read32(RF_REG(off));

			if (s == 0U) {
				probe_first[r] = val;
			} else if (val != probe_last[r]) {
				probe_changes[r]++;
			} else {
				/* unchanged */
			}
			probe_last[r] = val;
		}
		k_busy_wait(PROBE_SAMPLE_US);
	}

	k_busy_wait(PROBE_LAP_WAIT_US);
	lap_unwritten = count_word(0U, RF_RING_WORDS, PROBE_POISON);
	for (uint32_t i = 0U; i < PROBE_REWRAP_WORDS; i++) {
		ring[i] = PROBE_POISON;
	}
	k_busy_wait(PROBE_LAP_WAIT_US);
	rewrap_unwritten = count_word(0U, PROBE_REWRAP_WORDS, PROBE_POISON);
	rf_ring_stop();

	len = snprintf(out, size, "RINGPROBE %u %u", lap_unwritten, rewrap_unwritten);
	for (uint32_t r = 0U; (r < PROBE_REGS) && (len > 0) && ((size_t)len < size); r++) {
		if (probe_changes[r] != 0U) {
			len += snprintf(&out[len], size - (size_t)len, " %03x:%08x:%08x:%u",
					PROBE_FIRST_OFFSET + (r * BYTES_PER_WORD), probe_first[r],
					probe_last[r], probe_changes[r]);
		}
	}
	if ((len > 0) && ((size_t)len < (size - 1U))) {
		out[len] = '\n';
		len++;
		out[len] = '\0';
	}

	return len;
}

bool rf_reg_read(uint32_t offset, uint32_t *value)
{
	if ((offset >= RF_REG_SPAN) || ((offset % BYTES_PER_WORD) != 0U)) {
		return false;
	}
	*value = sys_read32(RF_REG(offset));

	return true;
}

bool rf_reg_write(uint32_t offset, uint32_t value)
{
	if ((offset >= RF_REG_SPAN) || ((offset % BYTES_PER_WORD) != 0U)) {
		return false;
	}
	sys_write32(value, RF_REG(offset));

	return true;
}

int rf_ring_trace(char *out, size_t size, uint32_t samples, uint32_t interval_us)
{
	volatile uint32_t *ring = rf_ring_base();
	uint64_t t0;
	int len = 0;

	rf_ring_start();
	t0 = now_us();
	for (uint32_t s = 0U; (s < samples) && (len >= 0) && ((size_t)len < size); s++) {
		bool rewritten = ring[PROBE_TRACE_WORD] != PROBE_POISON;

		ring[PROBE_TRACE_WORD] = PROBE_POISON;
		len += snprintf(&out[len], size - (size_t)len, "%u %08x %08x %u\n",
				(uint32_t)(now_us() - t0), sys_read32(RF_REG(RF_DUMP_CTRL_OFFSET)),
				sys_read32(RF_REG(PROBE_STATUS_OFFSET)), rewritten ? 1U : 0U);
		k_busy_wait(interval_us);
	}
	rf_ring_stop();

	return len;
}

void rf_cw_start(uint32_t mhz, int32_t pwr_dbm)
{
	rfc_rf_fsm_force(RF_FSM_RELEASE);
	rf_pri_cw_start((int16_t)pwr_dbm, (uint16_t)mhz);
}

void rf_cw_stop(void)
{
	rf_pri_cw_stop();
	rf_tune(freq_mhz);
}
