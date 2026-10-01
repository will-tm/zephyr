/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_mjpeg

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/video/video.h>

#include <mjpeg_reg.h>

#include "video_common.h"

LOG_MODULE_REGISTER(video_bflb_mjpeg, CONFIG_VIDEO_LOG_LEVEL);

/* Quantization tables, in natural order from the hardware side */
#define MJPEG_Q_PARAM_YY_OFFSET 0x400
#define MJPEG_Q_PARAM_UV_OFFSET 0x480

/* Header memory the encoder emits ahead of the scan data */
#define MJPEG_HEADER_OFFSET 0x800
#define MJPEG_HEADER_MAX    1024

/* YUV_MODE value for interleaved 4:2:2 input */
#define MJPEG_YUV_MODE_YUV422 3U

/* Burst length of the encoder AXI accesses: INCR16 */
#define MJPEG_XLEN_INCR16 3U

/* Granularity of the size of the output buffer the encoder is given */
#define MJPEG_STORE_UNIT 128U

/* Clear bit of the frame interrupt in the frame FIFO pop register */
#define MJPEG_INT_FRAME_CLR BIT(8)

#define MJPEG_DEFAULT_QUALITY 85

/* Standard tables from ITU-T T.81 annex K, in natural order */
static const uint8_t mjpeg_q_luma[64] = {
	16, 11, 10, 16, 24,  40,  51,  61,  12, 12, 14, 19, 26,  58,  60,  55,
	14, 13, 16, 24, 40,  57,  69,  56,  14, 17, 22, 29, 51,  87,  80,  62,
	18, 22, 37, 56, 68,  109, 103, 77,  24, 35, 55, 64, 81,  104, 113, 92,
	49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99,
};

static const uint8_t mjpeg_q_chroma[64] = {
	17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99,
	24, 26, 56, 99, 99, 99, 99, 99, 47, 66, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
};

/* Position in natural order of each coefficient in zig-zag order */
static const uint8_t mjpeg_zigzag[64] = {
	0,  1,  8,  16, 9,  2,  3,  10, 17, 24, 32, 25, 18, 11, 4,  5,
	12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6,  7,  14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

static const uint8_t mjpeg_dc_luma_bits[16] = {0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t mjpeg_dc_chroma_bits[16] = {0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0};
static const uint8_t mjpeg_dc_vals[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

static const uint8_t mjpeg_ac_luma_bits[16] = {0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d};
static const uint8_t mjpeg_ac_luma_vals[162] = {
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51,
	0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1,
	0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18,
	0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57,
	0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75,
	0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92,
	0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
	0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8,
	0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2,
	0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
};

static const uint8_t mjpeg_ac_chroma_bits[16] = {0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77};
static const uint8_t mjpeg_ac_chroma_vals[162] = {
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07,
	0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09,
	0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25,
	0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
	0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56,
	0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74,
	0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
	0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba,
	0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6,
	0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2,
	0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
};

/* The encoder works on 16x8 pixel blocks of interleaved 4:2:2 input */
#define MJPEG_FORMAT_CAP(fourcc)                                                                   \
	{                                                                                          \
		.pixelformat = (fourcc), .width_min = 16, .width_max = 4080, .width_step = 16,     \
		.height_min = 8, .height_max = 4088, .height_step = 8,                             \
	}

static const struct video_format_cap mjpeg_in_fmts[] = {
	MJPEG_FORMAT_CAP(VIDEO_PIX_FMT_YUYV),
	{0},
};

static const struct video_format_cap mjpeg_out_fmts[] = {
	MJPEG_FORMAT_CAP(VIDEO_PIX_FMT_JPEG),
	{0},
};

struct video_bflb_mjpeg_side {
	struct video_format fmt;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	bool streaming;
};

struct video_bflb_mjpeg_config {
	uint32_t base;
	void (*irq_config)(void);
};

struct video_bflb_mjpeg_data {
	struct video_bflb_mjpeg_side in;
	struct video_bflb_mjpeg_side out;
	struct k_spinlock lock;
	struct video_buffer *cur_in;
	struct video_buffer *cur_out;
	struct video_ctrl quality;
#ifdef CONFIG_POLL
	struct k_poll_signal *signal;
#endif
};

static void video_bflb_mjpeg_raise_signal(struct video_bflb_mjpeg_data *data, int result)
{
#ifdef CONFIG_POLL
	if (data->signal != NULL) {
		k_poll_signal_raise(data->signal, result);
	}
#else
	ARG_UNUSED(data);
	ARG_UNUSED(result);
#endif
}

/* Scale a base table to a quality, as the JPEG reference implementation does */
static void video_bflb_mjpeg_scale_table(const uint8_t *base, uint8_t quality, uint8_t *table)
{
	uint32_t scale = (quality < 50) ? 5000000U / quality : 200000U - quality * 2000U;

	for (int i = 0; i < 64; i++) {
		table[i] = CLAMP((base[i] * scale + 50000U) / 100000U, 1U, 255U);
	}
}

/* The hardware takes the reciprocal of each step, two coefficients per word */
static void video_bflb_mjpeg_load_table(const struct device *dev, uint32_t offset,
					const uint8_t *table)
{
	const struct video_bflb_mjpeg_config *cfg = dev->config;

	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 4; j++) {
			uint8_t a = table[16 * j + i];
			uint8_t b = table[16 * j + i + 8];
			uint16_t ra = DIV_ROUND_CLOSEST(2048U, a);
			uint16_t rb = DIV_ROUND_CLOSEST(2048U, b);

			sys_write32(ra | (uint32_t)rb << 16, cfg->base + offset + (i * 4 + j) * 4);
		}
	}
}

static size_t video_bflb_mjpeg_put_dht(uint8_t *p, uint8_t class_id, const uint8_t *bits,
				       const uint8_t *vals, size_t nvals)
{
	size_t len = 2 + 1 + 16 + nvals;

	p[0] = 0xff;
	p[1] = 0xc4;
	sys_put_be16(len, &p[2]);
	p[4] = class_id;
	memcpy(&p[5], bits, 16);
	memcpy(&p[21], vals, nvals);

	return 2 + len;
}

/* Build the JPEG header the encoder emits before the scan data of each frame */
static size_t video_bflb_mjpeg_build_header(uint8_t *p, uint16_t width, uint16_t height,
					    const uint8_t *q_luma, const uint8_t *q_chroma)
{
	static const uint8_t sof_components[] = {
		/* Y sampled 2x1, Cb and Cr 1x1, each with its quantization table */
		0x01, 0x21, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01,
	};
	static const uint8_t sos[] = {
		0xff, 0xda, 0x00, 0x0c, 0x03, 0x01, 0x00, 0x02, 0x11, 0x03, 0x11, 0x00, 0x3f, 0x00,
	};
	size_t n = 0;

	p[n++] = 0xff;
	p[n++] = 0xd8;

	for (int t = 0; t < 2; t++) {
		const uint8_t *q = (t == 0) ? q_luma : q_chroma;

		p[n++] = 0xff;
		p[n++] = 0xdb;
		p[n++] = 0x00;
		p[n++] = 0x43;
		p[n++] = t;
		for (int i = 0; i < 64; i++) {
			p[n++] = q[mjpeg_zigzag[i]];
		}
	}

	p[n++] = 0xff;
	p[n++] = 0xc0;
	p[n++] = 0x00;
	p[n++] = 0x11;
	p[n++] = 0x08;
	sys_put_be16(height, &p[n]);
	n += 2;
	sys_put_be16(width, &p[n]);
	n += 2;
	p[n++] = 0x03;
	memcpy(&p[n], sof_components, sizeof(sof_components));
	n += sizeof(sof_components);

	n += video_bflb_mjpeg_put_dht(&p[n], 0x00, mjpeg_dc_luma_bits, mjpeg_dc_vals,
				      sizeof(mjpeg_dc_vals));
	n += video_bflb_mjpeg_put_dht(&p[n], 0x10, mjpeg_ac_luma_bits, mjpeg_ac_luma_vals,
				      sizeof(mjpeg_ac_luma_vals));
	n += video_bflb_mjpeg_put_dht(&p[n], 0x01, mjpeg_dc_chroma_bits, mjpeg_dc_vals,
				      sizeof(mjpeg_dc_vals));
	n += video_bflb_mjpeg_put_dht(&p[n], 0x11, mjpeg_ac_chroma_bits, mjpeg_ac_chroma_vals,
				      sizeof(mjpeg_ac_chroma_vals));

	memcpy(&p[n], sos, sizeof(sos));
	n += sizeof(sos);

	return n;
}

/* Program the geometry, the quantization and the header of the stream to encode */
static void video_bflb_mjpeg_configure(const struct device *dev)
{
	const struct video_bflb_mjpeg_config *cfg = dev->config;
	struct video_bflb_mjpeg_data *data = dev->data;
	uint16_t width = data->in.fmt.width;
	uint16_t height = data->in.fmt.height;
	uint8_t quality = CLAMP(data->quality.val, 1, 100);
	uint8_t q_luma[64];
	uint8_t q_chroma[64];
	uint32_t header[MJPEG_HEADER_MAX / 4];
	size_t header_len;
	uint32_t regval;

	sys_write32(0, cfg->base + MJPEG_CONTROL_1_OFFSET);

	regval = MJPEG_XLEN_INCR16 << MJPEG_REG_W_XLEN_SHIFT;
	regval |= MJPEG_REG_READ_FWRAP | MJPEG_REG_MJPEG_BIT_ORDER;
	regval |= MJPEG_YUV_MODE_YUV422 << MJPEG_REG_YUV_MODE_SHIFT;
	sys_write32(regval, cfg->base + MJPEG_CONTROL_1_OFFSET);

	sys_write32((width / 16) << MJPEG_REG_FRAME_WBLK_SHIFT |
			    (height / 8) << MJPEG_REG_FRAME_HBLK_SHIFT,
		    cfg->base + MJPEG_FRAME_SIZE_OFFSET);

	regval = sys_read32(cfg->base + MJPEG_CONTROL_2_OFFSET);
	regval &= ~(MJPEG_REG_MJPEG_WAIT_CYCLE_MASK | MJPEG_REG_MJPEG_SW_MODE);
	regval |= 0x100 << MJPEG_REG_MJPEG_WAIT_CYCLE_SHIFT;
	sys_write32(regval, cfg->base + MJPEG_CONTROL_2_OFFSET);

	regval = sys_read32(cfg->base + MJPEG_SWAP_MODE_OFFSET);
	sys_write32(regval & ~MJPEG_REG_W_SWAP_MODE, cfg->base + MJPEG_SWAP_MODE_OFFSET);

	/* The whole frame is in memory, in blocks of eight lines */
	sys_write32(0, cfg->base + MJPEG_UV_FRAME_ADDR_OFFSET);
	sys_write32(height / 8, cfg->base + MJPEG_YUV_MEM_OFFSET);

	/* One frame per interrupt */
	regval = sys_read32(cfg->base + MJPEG_CONTROL_3_OFFSET);
	regval &= ~MJPEG_REG_FRAME_CNT_TRGR_INT_MASK;
	regval |= (1U << MJPEG_REG_FRAME_CNT_TRGR_INT_SHIFT) | MJPEG_REG_INT_NORMAL_EN;
	sys_write32(regval, cfg->base + MJPEG_CONTROL_3_OFFSET);

	video_bflb_mjpeg_scale_table(mjpeg_q_luma, quality, q_luma);
	video_bflb_mjpeg_scale_table(mjpeg_q_chroma, quality, q_chroma);
	video_bflb_mjpeg_load_table(dev, MJPEG_Q_PARAM_YY_OFFSET, q_luma);
	video_bflb_mjpeg_load_table(dev, MJPEG_Q_PARAM_UV_OFFSET, q_chroma);
	regval = sys_read32(cfg->base + MJPEG_Q_ENC_OFFSET);
	sys_write32(regval | MJPEG_REG_Q_SRAM_SW, cfg->base + MJPEG_Q_ENC_OFFSET);

	memset(header, 0, sizeof(header));
	header_len = video_bflb_mjpeg_build_header((uint8_t *)header, width, height, q_luma,
						   q_chroma);
	for (size_t i = 0; i < DIV_ROUND_UP(header_len, 4); i++) {
		sys_write32(header[i], cfg->base + MJPEG_HEADER_OFFSET + i * 4);
	}

	/* Byte order of interleaved YUYV, and the header and end of image to emit */
	regval = sys_read32(cfg->base + MJPEG_HEADER_BYTE_OFFSET);
	regval &= ~(MJPEG_REG_HEAD_BYTE_MASK | MJPEG_REG_Y0_ORDER_MASK | MJPEG_REG_U0_ORDER_MASK |
		    MJPEG_REG_Y1_ORDER_MASK | MJPEG_REG_V0_ORDER_MASK);
	regval |= 0U << MJPEG_REG_Y0_ORDER_SHIFT | 1U << MJPEG_REG_U0_ORDER_SHIFT |
		  2U << MJPEG_REG_Y1_ORDER_SHIFT | 3U << MJPEG_REG_V0_ORDER_SHIFT;
	regval |= header_len << MJPEG_REG_HEAD_BYTE_SHIFT | MJPEG_REG_TAIL_EXP;
	sys_write32(regval, cfg->base + MJPEG_HEADER_BYTE_OFFSET);

	/* Drop any stale frame and interrupt */
	sys_write32(0x3f00, cfg->base + MJPEG_FRAME_FIFO_POP_OFFSET);
}

/* Encode the next frame if both an input and an output buffer are waiting */
static void video_bflb_mjpeg_kick(const struct device *dev)
{
	const struct video_bflb_mjpeg_config *cfg = dev->config;
	struct video_bflb_mjpeg_data *data = dev->data;
	uint32_t regval;

	if (data->cur_in != NULL || !data->in.streaming || !data->out.streaming ||
	    k_fifo_is_empty(&data->in.fifo_in) || k_fifo_is_empty(&data->out.fifo_in)) {
		return;
	}

	data->cur_in = k_fifo_get(&data->in.fifo_in, K_NO_WAIT);
	data->cur_out = k_fifo_get(&data->out.fifo_in, K_NO_WAIT);

	sys_write32((uint32_t)data->cur_in->buffer, cfg->base + MJPEG_YY_FRAME_ADDR_OFFSET);
	sys_write32((uint32_t)data->cur_out->buffer, cfg->base + MJPEG_JPEG_FRAME_ADDR_OFFSET);
	sys_write32(data->cur_out->size / MJPEG_STORE_UNIT,
		    cfg->base + MJPEG_JPEG_STORE_MEMORY_OFFSET);

	/* Software mode run of a single frame */
	regval = sys_read32(cfg->base + MJPEG_CONTROL_2_OFFSET);
	regval &= ~(MJPEG_REG_SW_KICK_MODE | MJPEG_REG_SW_FRAME_MASK);
	regval |= 1U << MJPEG_REG_SW_FRAME_SHIFT;
	sys_write32(regval | MJPEG_REG_MJPEG_SW_MODE, cfg->base + MJPEG_CONTROL_2_OFFSET);
	sys_write32(regval, cfg->base + MJPEG_CONTROL_2_OFFSET);
	sys_write32(regval | MJPEG_REG_MJPEG_SW_RUN, cfg->base + MJPEG_CONTROL_2_OFFSET);
	sys_write32(regval, cfg->base + MJPEG_CONTROL_2_OFFSET);
}

static void video_bflb_mjpeg_isr(const struct device *dev)
{
	const struct video_bflb_mjpeg_config *cfg = dev->config;
	struct video_bflb_mjpeg_data *data = dev->data;
	k_spinlock_key_t key;

	if ((sys_read32(cfg->base + MJPEG_CONTROL_3_OFFSET) & MJPEG_STS_NORMAL_INT) == 0) {
		return;
	}

	sys_write32(MJPEG_INT_FRAME_CLR, cfg->base + MJPEG_FRAME_FIFO_POP_OFFSET);

	key = k_spin_lock(&data->lock);

	while ((sys_read32(cfg->base + MJPEG_CONTROL_3_OFFSET) & MJPEG_FRAME_VALID_CNT_MASK) != 0) {
		uint32_t bits = sys_read32(cfg->base + MJPEG_BIT_CNT0_OFFSET);

		sys_write32(1, cfg->base + MJPEG_FRAME_FIFO_POP_OFFSET);

		if (data->cur_in == NULL) {
			continue;
		}

		data->cur_out->bytesused = DIV_ROUND_UP(bits, 8);
		data->cur_out->timestamp = k_uptime_get_32();
		data->cur_in->bytesused = 0;
		k_fifo_put(&data->in.fifo_out, data->cur_in);
		k_fifo_put(&data->out.fifo_out, data->cur_out);
		data->cur_in = NULL;
		data->cur_out = NULL;
		video_bflb_mjpeg_raise_signal(data, VIDEO_BUF_DONE);
	}

	video_bflb_mjpeg_kick(dev);

	k_spin_unlock(&data->lock, key);
}

static struct video_bflb_mjpeg_side *video_bflb_mjpeg_side(struct video_bflb_mjpeg_data *data,
							   enum video_buf_type type)
{
	return (type == VIDEO_BUF_TYPE_INPUT) ? &data->in : &data->out;
}

static int video_bflb_mjpeg_set_fmt(const struct device *dev, struct video_format *fmt)
{
	struct video_bflb_mjpeg_data *data = dev->data;
	struct video_bflb_mjpeg_side *side = video_bflb_mjpeg_side(data, fmt->type);
	const struct video_format_cap *caps =
		(fmt->type == VIDEO_BUF_TYPE_INPUT) ? mjpeg_in_fmts : mjpeg_out_fmts;
	size_t idx;
	int ret;

	ret = video_format_caps_index(caps, fmt, &idx);
	if (ret < 0) {
		LOG_ERR("Unsupported format %s %ux%u", VIDEO_FOURCC_TO_STR(fmt->pixelformat),
			fmt->width, fmt->height);
		return ret;
	}

	if (side->streaming) {
		return -EBUSY;
	}

	if (fmt->type == VIDEO_BUF_TYPE_INPUT) {
		fmt->pitch = fmt->width * 2;
		fmt->size = fmt->pitch * fmt->height;
	} else {
		/* Room for a frame of a byte per pixel, far above what the encoder emits */
		fmt->pitch = 0;
		fmt->size = ROUND_UP(fmt->width * fmt->height, MJPEG_STORE_UNIT);
	}

	side->fmt = *fmt;

	return 0;
}

static int video_bflb_mjpeg_get_fmt(const struct device *dev, struct video_format *fmt)
{
	struct video_bflb_mjpeg_data *data = dev->data;

	*fmt = video_bflb_mjpeg_side(data, fmt->type)->fmt;

	return 0;
}

static int video_bflb_mjpeg_set_stream(const struct device *dev, bool enable,
				       enum video_buf_type type)
{
	struct video_bflb_mjpeg_data *data = dev->data;
	struct video_bflb_mjpeg_side *side = video_bflb_mjpeg_side(data, type);
	struct video_bflb_mjpeg_side *other =
		video_bflb_mjpeg_side(data, (type == VIDEO_BUF_TYPE_INPUT) ? VIDEO_BUF_TYPE_OUTPUT
									  : VIDEO_BUF_TYPE_INPUT);
	k_spinlock_key_t key;

	if (enable && (data->in.fmt.width != data->out.fmt.width ||
		       data->in.fmt.height != data->out.fmt.height)) {
		LOG_ERR("Input and output resolutions differ");
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	/* The stream is set up once both of its ends are started */
	if (enable && !side->streaming && other->streaming && data->cur_in == NULL) {
		video_bflb_mjpeg_configure(dev);
	}

	side->streaming = enable;
	video_bflb_mjpeg_kick(dev);

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int video_bflb_mjpeg_enqueue(const struct device *dev, struct video_buffer *vbuf)
{
	struct video_bflb_mjpeg_data *data = dev->data;
	struct video_bflb_mjpeg_side *side = video_bflb_mjpeg_side(data, vbuf->type);
	k_spinlock_key_t key;

	if (vbuf->type == VIDEO_BUF_TYPE_OUTPUT) {
		if (vbuf->size < MJPEG_STORE_UNIT) {
			return -EINVAL;
		}
		/* The encoder writes through AXI, so no dirty line may be left behind */
		sys_cache_data_flush_and_invd_range(vbuf->buffer, vbuf->size);
	} else {
		if (vbuf->bytesused < side->fmt.size) {
			LOG_ERR("Input buffer holds %u bytes of a %u byte frame", vbuf->bytesused,
				side->fmt.size);
			return -EINVAL;
		}
		/* The encoder reads through AXI, so all written lines must be in memory */
		sys_cache_data_flush_range(vbuf->buffer, vbuf->bytesused);
	}

	key = k_spin_lock(&data->lock);
	k_fifo_put(&side->fifo_in, vbuf);
	video_bflb_mjpeg_kick(dev);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int video_bflb_mjpeg_flush(const struct device *dev, bool cancel)
{
	struct video_bflb_mjpeg_data *data = dev->data;
	struct video_buffer *vbuf;
	k_spinlock_key_t key;

	/* The frame being encoded completes either way */
	while (data->cur_in != NULL ||
	       (!cancel && (!k_fifo_is_empty(&data->in.fifo_in) ||
			    !k_fifo_is_empty(&data->out.fifo_in)))) {
		k_sleep(K_MSEC(1));
	}

	if (!cancel) {
		return 0;
	}

	key = k_spin_lock(&data->lock);
	while ((vbuf = k_fifo_get(&data->in.fifo_in, K_NO_WAIT)) != NULL) {
		k_fifo_put(&data->in.fifo_out, vbuf);
	}
	while ((vbuf = k_fifo_get(&data->out.fifo_in, K_NO_WAIT)) != NULL) {
		k_fifo_put(&data->out.fifo_out, vbuf);
	}
	k_spin_unlock(&data->lock, key);

	video_bflb_mjpeg_raise_signal(data, VIDEO_BUF_ABORTED);

	return 0;
}

static int video_bflb_mjpeg_dequeue(const struct device *dev, struct video_buffer **vbuf,
				    k_timeout_t timeout)
{
	struct video_bflb_mjpeg_data *data = dev->data;
	struct video_bflb_mjpeg_side *side = video_bflb_mjpeg_side(data, (*vbuf)->type);

	*vbuf = k_fifo_get(&side->fifo_out, timeout);
	if (*vbuf == NULL) {
		return -EAGAIN;
	}

	return 0;
}

static int video_bflb_mjpeg_get_caps(const struct device *dev, struct video_caps *caps)
{
	caps->format_caps = (caps->type == VIDEO_BUF_TYPE_INPUT) ? mjpeg_in_fmts : mjpeg_out_fmts;
	caps->min_vbuf_count = 1;

	return 0;
}

#ifdef CONFIG_POLL
static int video_bflb_mjpeg_set_signal(const struct device *dev, struct k_poll_signal *sig)
{
	struct video_bflb_mjpeg_data *data = dev->data;

	data->signal = sig;

	return 0;
}
#endif

static DEVICE_API(video, video_bflb_mjpeg_driver_api) = {
	.set_format = video_bflb_mjpeg_set_fmt,
	.get_format = video_bflb_mjpeg_get_fmt,
	.set_stream = video_bflb_mjpeg_set_stream,
	.enqueue = video_bflb_mjpeg_enqueue,
	.dequeue = video_bflb_mjpeg_dequeue,
	.flush = video_bflb_mjpeg_flush,
	.get_caps = video_bflb_mjpeg_get_caps,
#ifdef CONFIG_POLL
	.set_signal = video_bflb_mjpeg_set_signal,
#endif
};

static int video_bflb_mjpeg_init(const struct device *dev)
{
	const struct video_bflb_mjpeg_config *cfg = dev->config;
	struct video_bflb_mjpeg_data *data = dev->data;
	int ret;

	k_fifo_init(&data->in.fifo_in);
	k_fifo_init(&data->in.fifo_out);
	k_fifo_init(&data->out.fifo_in);
	k_fifo_init(&data->out.fifo_out);
	data->in.fmt.type = VIDEO_BUF_TYPE_INPUT;
	data->out.fmt.type = VIDEO_BUF_TYPE_OUTPUT;

	ret = video_init_ctrl(&data->quality, dev, VIDEO_CID_JPEG_COMPRESSION_QUALITY,
			      (struct video_ctrl_range){.min = 1,
							.max = 100,
							.step = 1,
							.def = MJPEG_DEFAULT_QUALITY});
	if (ret < 0) {
		return ret;
	}

	cfg->irq_config();

	return 0;
}

#define VIDEO_BFLB_MJPEG_INIT(n)                                                                   \
	static void video_bflb_mjpeg_irq_config_##n(void)                                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), video_bflb_mjpeg_isr,       \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct video_bflb_mjpeg_config video_bflb_mjpeg_config_##n = {                \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.irq_config = video_bflb_mjpeg_irq_config_##n,                                     \
	};                                                                                         \
                                                                                                   \
	static struct video_bflb_mjpeg_data video_bflb_mjpeg_data_##n;                             \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, video_bflb_mjpeg_init, NULL, &video_bflb_mjpeg_data_##n,          \
			      &video_bflb_mjpeg_config_##n, POST_KERNEL,                           \
			      CONFIG_VIDEO_INIT_PRIORITY, &video_bflb_mjpeg_driver_api);           \
                                                                                                   \
	VIDEO_DEVICE_DEFINE(mjpeg_##n, DEVICE_DT_INST_GET(n), NULL);

DT_INST_FOREACH_STATUS_OKAY(VIDEO_BFLB_MJPEG_INIT)
