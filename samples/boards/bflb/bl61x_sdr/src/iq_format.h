/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BL61X_SDR_IQ_FORMAT_H_
#define BL61X_SDR_IQ_FORMAT_H_

#include <stdbool.h>
#include <stdint.h>

#define IQ_FORMAT_WIDTH_MAX     16U
#define IQ_FORMAT_WIDTH_DEFAULT 11U

/* Bit layout of I and Q in a dump word, adjustable at runtime */
bool iq_format_set(uint32_t i_lsb, uint32_t q_lsb, uint32_t width);
void iq_format_get(uint32_t *i_lsb, uint32_t *q_lsb, uint32_t *width);

/* Signed components scaled to 10 bits */
int32_t iq_format_i10(uint32_t word);
int32_t iq_format_q10(uint32_t word);

/* Interleaved int8 pair: I in the low byte, Q in the high byte */
uint16_t iq_format_iq8(uint32_t word);

/* True for the measured layout: 11-bit I in bits 31:16, Q in bits 15:0 */
bool iq_format_is_default(void);

/*
 * Words with width-bit sign-extended I in bits 31:16 and Q in bits 15:0
 * (width 10..16): interleaved int8 pair, the top 8 bits of each component;
 * two's complement makes the shift a floor.
 */
static inline uint16_t iq_format_iq8_width(uint32_t word, uint32_t width)
{
	return (uint16_t)(((word >> (width + 8U)) & 0xffU) | ((word << (16U - width)) & 0xff00U));
}

/* Same layout: 10-bit I in bits 9:0 and Q in bits 19:10 */
static inline uint32_t iq_format_iq20_width(uint32_t word, uint32_t width)
{
	return ((word >> (width + 6U)) & 0x3ffU) | ((word << (20U - width)) & 0xffc00U);
}

/* Default layout fast path */
static inline uint16_t iq_format_iq8_default(uint32_t word)
{
	return iq_format_iq8_width(word, IQ_FORMAT_WIDTH_DEFAULT);
}

#endif /* BL61X_SDR_IQ_FORMAT_H_ */
