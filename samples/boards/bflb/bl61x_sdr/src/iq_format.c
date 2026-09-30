/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>

#include "iq_format.h"

/* Default guess for the dump word layout */
/* ESP-SDR orientation, measured with a CW tone: I in bits 31:16, Q in bits 15:0 */
#define I_LSB_DEFAULT  16U
#define Q_LSB_DEFAULT  0U
/* Measured: 11-bit signed samples, sign-extended to 16 bits */
#define WIDTH_DEFAULT  IQ_FORMAT_WIDTH_DEFAULT
#define IQ10_BITS      10U
#define IQ8_SHIFT      2U
#define BYTE_BITS      8U
#define BYTE_MASK      0xffU
#define WORD_BITS      32U

static uint32_t i_lsb = I_LSB_DEFAULT;
static uint32_t q_lsb = Q_LSB_DEFAULT;
static uint32_t width = WIDTH_DEFAULT;

static int32_t field10(uint32_t word, uint32_t lsb);

static int32_t field10(uint32_t word, uint32_t lsb)
{
	uint32_t raw = (word >> lsb) & (BIT(width) - 1U);
	int32_t val = (int32_t)raw;

	if ((raw & BIT(width - 1U)) != 0U) {
		val -= (int32_t)BIT(width);
	}
	if (width > IQ10_BITS) {
		uint32_t shift = width - IQ10_BITS;

		/* Floor, as an arithmetic shift, using shifts of non-negative values */
		uint32_t round = (uint32_t)BIT(shift) - 1U;

		val = (val >= 0) ? (int32_t)((uint32_t)val >> shift)
				 : -(int32_t)(((uint32_t)(-val) + round) >> shift);
	} else {
		val *= (int32_t)BIT(IQ10_BITS - width);
	}

	return val;
}

bool iq_format_is_default(void)
{
	return (i_lsb == I_LSB_DEFAULT) && (q_lsb == Q_LSB_DEFAULT) && (width == WIDTH_DEFAULT);
}

bool iq_format_set(uint32_t new_i_lsb, uint32_t new_q_lsb, uint32_t new_width)
{
	if ((new_width == 0U) || (new_width > IQ_FORMAT_WIDTH_MAX) ||
	    ((new_i_lsb + new_width) > WORD_BITS) || ((new_q_lsb + new_width) > WORD_BITS)) {
		return false;
	}

	i_lsb = new_i_lsb;
	q_lsb = new_q_lsb;
	width = new_width;

	return true;
}

void iq_format_get(uint32_t *out_i_lsb, uint32_t *out_q_lsb, uint32_t *out_width)
{
	*out_i_lsb = i_lsb;
	*out_q_lsb = q_lsb;
	*out_width = width;
}

int32_t iq_format_i10(uint32_t word)
{
	return field10(word, i_lsb);
}

int32_t iq_format_q10(uint32_t word)
{
	return field10(word, q_lsb);
}

/* Top 8 of the 10 bits, as the ESP-SDR packer (floor, not truncation) */
uint16_t iq_format_iq8(uint32_t word)
{
	uint32_t i8 = ((uint32_t)iq_format_i10(word) >> IQ8_SHIFT) & BYTE_MASK;
	uint32_t q8 = ((uint32_t)iq_format_q10(word) >> IQ8_SHIFT) & BYTE_MASK;

	return (uint16_t)(i8 | (q8 << BYTE_BITS));
}
