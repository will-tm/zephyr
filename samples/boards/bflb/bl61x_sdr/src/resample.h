/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Rational resampling of dump words by a polyphase FIR */

#ifndef BL61X_SDR_RESAMPLE_H_
#define BL61X_SDR_RESAMPLE_H_

#include <stdint.h>

struct resample {
	/* Lowpass taps (Q15) at the input rate times up, DC gain up */
	const int16_t *taps;
	uint32_t len;
	uint32_t up;
	uint32_t down;
};

/* 40 MS/s to 20 MS/s: passband 8 MHz, 70 dB stopband from 12 MHz */
extern const struct resample resample_40m_20m;
/* 40 MS/s to 16 MS/s: passband 6 MHz, 69 dB stopband from 10 MHz */
extern const struct resample resample_40m_16m;

/* Input words that cover n output samples */
uint32_t resample_input_words(const struct resample *r, uint32_t n);

/*
 * Filters in_words dump words (signed I in bits 15:0, Q in bits 31:16) into
 * n words of the same layout, saturated to width bits. The outputs are
 * centered on the input; taps reaching outside it see zeros.
 */
void resample_run(const struct resample *r, const uint32_t *in, uint32_t in_words, uint32_t *out,
		  uint32_t n, uint32_t width);

#endif /* BL61X_SDR_RESAMPLE_H_ */
