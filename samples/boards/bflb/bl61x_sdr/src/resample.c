/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

#include "resample.h"

#define Q15_SHIFT      15
#define Q15_HALF       (1 << (Q15_SHIFT - 1))
#define HALF_WORD_BITS 16U
#define HALF_WORD_MASK 0xffffU

/* Kaiser windowed sinc, 40 MS/s: half-band, pass 8 MHz, stop 12 MHz */
static const int16_t taps_40m_20m[] = {
	8, 0, -28, 0, 66, 0, -134, 0, 243, 0, -414, 0,
	673, 0, -1079, 0, 1772, 0, -3280, 0, 10363, 16383, 10363, 0,
	-3280, 0, 1772, 0, -1079, 0, 673, 0, -414, 0, 243, 0,
	-134, 0, 66, 0, -28, 0, 8,
};

/* Kaiser windowed sinc, 80 MS/s (40 MS/s interpolated by 2): pass 6 MHz, stop 10 MHz */
static const int16_t taps_40m_16m[] = {
	3, 6, 7, 4, -5, -18, -29, -29, -14, 17, 55, 83,
	80, 36, -43, -132, -190, -178, -78, 90, 270, 380, 349, 151,
	-171, -506, -705, -643, -277, 312, 920, 1284, 1176, 511, -584, -1761,
	-2534, -2421, -1115, 1385, 4725, 8259, 11210, 12887, 12887, 11210, 8259, 4725,
	1385, -1115, -2421, -2534, -1761, -584, 511, 1176, 1284, 920, 312, -277,
	-643, -705, -506, -171, 151, 349, 380, 270, 90, -78, -178, -190,
	-132, -43, 36, 80, 83, 55, 17, -14, -29, -29, -18, -5,
	4, 7, 6, 3,
};

const struct resample resample_40m_20m = {
	.taps = taps_40m_20m,
	.len = ARRAY_SIZE(taps_40m_20m),
	.up = 1U,
	.down = 2U,
};

const struct resample resample_40m_16m = {
	.taps = taps_40m_16m,
	.len = ARRAY_SIZE(taps_40m_16m),
	.up = 2U,
	.down = 5U,
};

static int32_t q15_round(int32_t acc);
static uint32_t saturate(int32_t val, int32_t lim);

/* Round half away from zero without shifting negative values */
static int32_t q15_round(int32_t acc)
{
	return (acc >= 0) ? ((acc + Q15_HALF) >> Q15_SHIFT) : -((Q15_HALF - acc) >> Q15_SHIFT);
}

static uint32_t saturate(int32_t val, int32_t lim)
{
	int32_t v = CLAMP(val, -lim, lim - 1);

	return (uint32_t)v & HALF_WORD_MASK;
}

uint32_t resample_input_words(const struct resample *r, uint32_t n)
{
	/* Output span plus the filter support, at the input rate */
	return DIV_ROUND_UP(((n - 1U) * r->down) + r->len, r->up);
}

void resample_run(const struct resample *r, const uint32_t *in, uint32_t in_words, uint32_t *out,
		  uint32_t n, uint32_t width)
{
	const int32_t lim = (int32_t)BIT(width - 1U);
	const int32_t up = (int32_t)r->up;
	/* Virtual (interpolated) positions of the outputs, centered on the input */
	const int32_t span = (int32_t)((n - 1U) * r->down);
	const int32_t first = ((((int32_t)in_words * up) - 1) - span) / 2;
	const int32_t delay = ((int32_t)r->len - 1) / 2;

	for (uint32_t k = 0U; k < n; k++) {
		/* Newest virtual input under the filter, and its tap phase */
		int32_t newest = first + ((int32_t)k * (int32_t)r->down) + delay;
		int32_t phase = newest % up;
		int32_t idx = (newest - phase) / up;
		int32_t count = ((int32_t)r->len - phase + up - 1) / up;
		bool inside = ((idx - (count - 1)) >= 0) && (idx < (int32_t)in_words);
		int32_t acc_i = 0;
		int32_t acc_q = 0;

		for (int32_t t = 0; t < count; t++) {
			int32_t j = phase + (t * up);
			int32_t i = idx - t;

			if (inside || ((i >= 0) && (i < (int32_t)in_words))) {
				uint32_t w = in[i];
				int32_t h = r->taps[j];

				acc_i += h * (int16_t)(w & HALF_WORD_MASK);
				acc_q += h * (int16_t)(w >> HALF_WORD_BITS);
			}
		}

		out[k] = saturate(q15_round(acc_i), lim) |
			 (saturate(q15_round(acc_q), lim) << HALF_WORD_BITS);
	}
}
