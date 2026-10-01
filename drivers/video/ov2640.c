/*
 * Copyright (c) 2021 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ovti_ov2640

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/video/video.h>

#include "video_common.h"

LOG_MODULE_REGISTER(video_ov2640, CONFIG_VIDEO_LOG_LEVEL);

/* DSP register bank FF=0x00*/
#define QS     0x44
#define HSIZE  0x51
#define VSIZE  0x52
#define XOFFL  0x53
#define YOFFL  0x54
#define VHYX   0x55
#define TEST   0x57
#define ZMOW   0x5A
#define ZMOH   0x5B
#define ZMHH   0x5C
#define BPADDR 0x7C
#define BPDATA 0x7D
#define SIZEL  0x8C
#define HSIZE8 0xC0
#define VSIZE8 0xC1
#define CTRL1  0xC3

#define CTRLI       0x50
#define CTRLI_LP_DP 0x80

#define CTRL0        0xC2
#define CTRL0_YUV422 0x08
#define CTRL0_YUV_EN 0x04
#define CTRL0_RGB_EN 0x02

#define CTRL2           0x86
#define CTRL2_DCW_EN    0x20
#define CTRL2_SDE_EN    0x10
#define CTRL2_UV_ADJ_EN 0x08
#define CTRL2_UV_AVG_EN 0x04
#define CTRL2_CMX_EN    0x01

#define CTRL3              0x87
#define CTRL3_BPC_EN       0x80
#define CTRL3_WPC_EN       0x40
#define R_DVP_SP           0xD3
#define R_DVP_SP_AUTO_MODE 0x80

#define R_BYPASS           0x05
#define R_BYPASS_DSP_EN    0x00
#define R_BYPASS_DSP_BYPAS 0x01

#define IMAGE_MODE         0xDA
#define IMAGE_MODE_JPEG_EN 0x10
#define IMAGE_MODE_RGB565  0x08

#define RESET      0xE0
#define RESET_JPEG 0x10
#define RESET_DVP  0x04

#define MC_BIST              0xF9
#define MC_BIST_RESET        0x80
#define MC_BIST_BOOT_ROM_SEL 0x40

#define BANK_SEL        0xFF
#define BANK_SEL_DSP    0x00
#define BANK_SEL_SENSOR 0x01

/* Sensor register bank FF=0x01*/
#define COM1        0x03
#define REG_PID     0x0A
#define REG_PID_VAL 0x26
#define REG_VER     0x0B
#define REG_VER_VAL 0x42
#define AEC         0x10
#define CLKRC       0x11
#define COM10       0x15
#define HSTART      0x17
#define HSTOP       0x18
#define VSTART      0x19
#define VSTOP       0x1A
#define AEW         0x24
#define AEB         0x25
#define ARCOM2      0x34
#define FLL         0x46
#define FLH         0x47
#define COM19       0x48
#define ZOOMS       0x49
#define BD50        0x4F
#define BD60        0x50
#define REG5D       0x5D
#define REG5E       0x5E
#define REG5F       0x5F
#define REG60       0x60
#define HISTO_LOW   0x61
#define HISTO_HIGH  0x62

#define REG04           0x04
#define REG04_DEFAULT   0x28
#define REG04_HFLIP_IMG 0x80
#define REG04_VFLIP_IMG 0x40
#define REG04_VREF_EN   0x10
#define REG04_HREF_EN   0x08
#define REG04_SET(x)    (REG04_DEFAULT | x)

#define COM2              0x09
#define COM2_OUT_DRIVE_3x 0x02

#define COM3             0x0C
#define COM3_DEFAULT     0x38
#define COM3_BAND_AUTO   0x02
#define COM3_BAND_SET(x) (COM3_DEFAULT | x)

#define COM7           0x12
#define COM7_SRST      0x80
#define COM7_RES_UXGA  0x00 /* UXGA */
#define COM7_ZOOM_EN   0x04 /* Enable Zoom */
#define COM7_COLOR_BAR 0x02 /* Enable Color Bar Test */

#define COM8         0x13
#define COM8_DEFAULT 0xC0
#define COM8_BNDF_EN 0x20 /* Enable Banding filter */
#define COM8_AGC_EN  0x04 /* AGC Auto/Manual control selection */
#define COM8_AEC_EN  0x01 /* Auto/Manual Exposure control */
#define COM8_SET(x)  (COM8_DEFAULT | x)

#define COM9             0x14 /* AGC gain ceiling */
#define COM9_DEFAULT     0x08
#define COM9_AGC_GAIN_8x 0x02 /* AGC:    8x */
#define COM9_AGC_SET(x)  (COM9_DEFAULT | (x << 5))

#define COM10 0x15

#define CTRL1_AWB 0x08 /* Enable AWB */

#define VV                  0x26
#define VV_AGC_TH_SET(h, l) ((h << 4) | (l & 0x0F))

#define REG32      0x32
#define REG32_UXGA 0x36

#define CIF_WIDTH      352
#define CIF_HEIGHT     288
#define HD_720_WIDTH   1280
#define HD_720_HEIGHT  720
#define HD_1080_WIDTH  1920
#define HD_1080_HEIGHT 1080
#define QCIF_WIDTH     176
#define QCIF_HEIGHT    144
#define QQCIF_WIDTH    88
#define QQCIF_HEIGHT   72
#define QQVGA_WIDTH    160
#define QQVGA_HEIGHT   120
#define QVGA_WIDTH     320
#define QVGA_HEIGHT    240
#define SVGA_WIDTH     800
#define SVGA_HEIGHT    600
#define SXGA_WIDTH     1280
#define SXGA_HEIGHT    1024
#define VGA_WIDTH      640
#define VGA_HEIGHT     480
#define UXGA_WIDTH     1600
#define UXGA_HEIGHT    1200
#define XGA_WIDTH      1024
#define XGA_HEIGHT     768

struct ov2640_reg {
	uint8_t addr;
	uint8_t value;
};

/*
 * Register settings of the esp32-camera OV2640 driver, which runs the sensor
 * on DVP hosts that sample a frame between the VSYNC edges and expect HREF to
 * frame each line: a 24MHz CIF base set, a sensor mode per output size range,
 * and the DSP output format applied once the window is set.
 */
static const struct ov2640_reg ov2640_init_regs[] = {
	{0xff, 0x00}, {0x2c, 0xff}, {0x2e, 0xdf}, {0xff, 0x01},
	{0x3c, 0x32}, {0x11, 0x01}, {0x09, 0x02}, {0x04, 0x28},
	{0x13, 0xe5}, {0x14, 0x48}, {0x2c, 0x0c}, {0x33, 0x78},
	{0x3a, 0x33}, {0x3b, 0xfb}, {0x3e, 0x00}, {0x43, 0x11},
	{0x16, 0x10}, {0x39, 0x92}, {0x35, 0xda}, {0x22, 0x1a},
	{0x37, 0xc3}, {0x23, 0x00}, {0x34, 0xc0}, {0x06, 0x88},
	{0x07, 0xc0}, {0x0d, 0x87}, {0x0e, 0x41}, {0x4c, 0x00},
	{0x4a, 0x81}, {0x21, 0x99}, {0x24, 0x40}, {0x25, 0x38},
	{0x26, 0x82}, {0x5c, 0x00}, {0x63, 0x00}, {0x61, 0x70},
	{0x62, 0x80}, {0x7c, 0x05}, {0x20, 0x80}, {0x28, 0x30},
	{0x6c, 0x00}, {0x6d, 0x80}, {0x6e, 0x00}, {0x70, 0x02},
	{0x71, 0x94}, {0x73, 0xc1}, {0x3d, 0x34}, {0x5a, 0x57},
	{0x4f, 0xbb}, {0x50, 0x9c}, {0x12, 0x20}, {0x17, 0x11},
	{0x18, 0x43}, {0x19, 0x00}, {0x1a, 0x25}, {0x32, 0x89},
	{0x37, 0xc0}, {0x4f, 0xca}, {0x50, 0xa8}, {0x6d, 0x00},
	{0x3d, 0x38}, {0xff, 0x00}, {0xe5, 0x7f}, {0xf9, 0xc0},
	{0x41, 0x24}, {0xe0, 0x14}, {0x76, 0xff}, {0x33, 0xa0},
	{0x42, 0x20}, {0x43, 0x18}, {0x4c, 0x00}, {0x87, 0x50},
	{0x88, 0x3f}, {0xd7, 0x03}, {0xd9, 0x10}, {0xd3, 0x82},
	{0xc8, 0x08}, {0xc9, 0x80}, {0x7c, 0x00}, {0x7d, 0x00},
	{0x7c, 0x03}, {0x7d, 0x48}, {0x7d, 0x48}, {0x7c, 0x08},
	{0x7d, 0x20}, {0x7d, 0x10}, {0x7d, 0x0e}, {0x90, 0x00},
	{0x91, 0x0e}, {0x91, 0x1a}, {0x91, 0x31}, {0x91, 0x5a},
	{0x91, 0x69}, {0x91, 0x75}, {0x91, 0x7e}, {0x91, 0x88},
	{0x91, 0x8f}, {0x91, 0x96}, {0x91, 0xa3}, {0x91, 0xaf},
	{0x91, 0xc4}, {0x91, 0xd7}, {0x91, 0xe8}, {0x91, 0x20},
	{0x92, 0x00}, {0x93, 0x06}, {0x93, 0xe3}, {0x93, 0x05},
	{0x93, 0x05}, {0x93, 0x00}, {0x93, 0x04}, {0x93, 0x00},
	{0x93, 0x00}, {0x93, 0x00}, {0x93, 0x00}, {0x93, 0x00},
	{0x93, 0x00}, {0x93, 0x00}, {0x96, 0x00}, {0x97, 0x08},
	{0x97, 0x19}, {0x97, 0x02}, {0x97, 0x0c}, {0x97, 0x24},
	{0x97, 0x30}, {0x97, 0x28}, {0x97, 0x26}, {0x97, 0x02},
	{0x97, 0x98}, {0x97, 0x80}, {0x97, 0x00}, {0x97, 0x00},
	{0xa4, 0x00}, {0xa8, 0x00}, {0xc5, 0x11}, {0xc6, 0x51},
	{0xbf, 0x80}, {0xc7, 0x10}, {0xb6, 0x66}, {0xb8, 0xa5},
	{0xb7, 0x64}, {0xb9, 0x7c}, {0xb3, 0xaf}, {0xb4, 0x97},
	{0xb5, 0xff}, {0xb0, 0xc5}, {0xb1, 0x94}, {0xb2, 0x0f},
	{0xc4, 0x5c}, {0xc3, 0xfd}, {0x7f, 0x00}, {0xe5, 0x1f},
	{0xe1, 0x67}, {0xdd, 0x7f}, {0xda, 0x00}, {0xe0, 0x00},
	{0x05, 0x00},
};

static const struct ov2640_reg ov2640_cif_mode_regs[] = {
	{0xff, 0x01}, {0x12, 0x20}, {0x03, 0x0a}, {0x32, 0x89},
	{0x17, 0x11}, {0x18, 0x43}, {0x19, 0x00}, {0x1a, 0x25},
	{0x4f, 0xca}, {0x50, 0xa8}, {0x5a, 0x23}, {0x6d, 0x00},
	{0x3d, 0x38}, {0x39, 0x92}, {0x35, 0xda}, {0x22, 0x1a},
	{0x37, 0xc3}, {0x23, 0x00}, {0x34, 0xc0}, {0x06, 0x88},
	{0x07, 0xc0}, {0x0d, 0x87}, {0x0e, 0x41}, {0x4c, 0x00},
	{0xff, 0x00}, {0xe0, 0x04}, {0xc0, 0x32}, {0xc1, 0x25},
	{0x8c, 0x00}, {0x51, 0x64}, {0x52, 0x4a}, {0x53, 0x00},
	{0x54, 0x00}, {0x55, 0x00}, {0x57, 0x00}, {0x86, 0x3d},
	{0x50, 0x80},
};

static const struct ov2640_reg ov2640_svga_mode_regs[] = {
	{0xff, 0x01}, {0x12, 0x40}, {0x03, 0x0a}, {0x32, 0x09},
	{0x17, 0x11}, {0x18, 0x43}, {0x19, 0x00}, {0x1a, 0x4b},
	{0x37, 0xc0}, {0x4f, 0xca}, {0x50, 0xa8}, {0x5a, 0x23},
	{0x6d, 0x00}, {0x3d, 0x38}, {0x39, 0x92}, {0x35, 0xda},
	{0x22, 0x1a}, {0x37, 0xc3}, {0x23, 0x00}, {0x34, 0xc0},
	{0x06, 0x88}, {0x07, 0xc0}, {0x0d, 0x87}, {0x0e, 0x41},
	{0x42, 0x03}, {0x4c, 0x00}, {0xff, 0x00}, {0xe0, 0x04},
	{0xc0, 0x64}, {0xc1, 0x4b}, {0x8c, 0x00}, {0x51, 0xc8},
	{0x52, 0x96}, {0x53, 0x00}, {0x54, 0x00}, {0x55, 0x00},
	{0x57, 0x00}, {0x86, 0x3d}, {0x50, 0x80},
};

static const struct ov2640_reg ov2640_uxga_mode_regs[] = {
	{0xff, 0x01}, {0x12, 0x00}, {0x03, 0x0f}, {0x32, 0x36},
	{0x17, 0x11}, {0x18, 0x75}, {0x19, 0x01}, {0x1a, 0x97},
	{0x3d, 0x34}, {0x4f, 0xbb}, {0x50, 0x9c}, {0x5a, 0x57},
	{0x6d, 0x80}, {0x39, 0x82}, {0x23, 0x00}, {0x07, 0xc0},
	{0x4c, 0x00}, {0x35, 0x88}, {0x22, 0x0a}, {0x37, 0x40},
	{0x34, 0xa0}, {0x06, 0x02}, {0x0d, 0xb7}, {0x0e, 0x01},
	{0x42, 0x83}, {0xff, 0x00}, {0xe0, 0x04}, {0xc0, 0xc8},
	{0xc1, 0x96}, {0x8c, 0x00}, {0x51, 0x90}, {0x52, 0x2c},
	{0x53, 0x00}, {0x54, 0x00}, {0x55, 0x88}, {0x57, 0x00},
	{0x86, 0x3d}, {0x50, 0x00},
};

/* JPEG keeps HREF high for the whole frame, so the host sees one stream per frame */
static const struct ov2640_reg ov2640_jpeg_regs[] = {
	{0xff, 0x00}, {0xe0, 0x14}, {0xda, 0x12}, {0xd7, 0x03},
	{0xe1, 0x77}, {0xe5, 0x1f}, {0xd9, 0x10}, {0xdf, 0x80},
	{0x33, 0x80}, {0x3c, 0x10}, {0xeb, 0x30}, {0xdd, 0x7f},
	{0xe0, 0x00},
};

static const struct ov2640_reg ov2640_yuyv_regs[] = {
	{0xff, 0x00}, {0xe0, 0x04}, {0xda, 0x00}, {0xd7, 0x01},
	{0xe1, 0x67}, {0xe0, 0x00},
};

static const struct ov2640_reg ov2640_rgb565_regs[] = {
	{0xff, 0x00}, {0xe0, 0x04}, {0xda, 0x08}, {0xd7, 0x03},
	{0xe1, 0x77}, {0xe0, 0x00},
};

#define NUM_BRIGHTNESS_LEVELS (5)
static const uint8_t brightness_regs[NUM_BRIGHTNESS_LEVELS + 1][5] = {
	{BPADDR, BPDATA, BPADDR, BPDATA, BPDATA}, /* reg addr */
	{0x00, 0x04, 0x09, 0x00, 0x00},           /* -2 */
	{0x00, 0x04, 0x09, 0x10, 0x00},           /* -1 */
	{0x00, 0x04, 0x09, 0x20, 0x00},           /*  0 */
	{0x00, 0x04, 0x09, 0x30, 0x00},           /* +1 */
	{0x00, 0x04, 0x09, 0x40, 0x00},           /* +2 */
};

#define NUM_CONTRAST_LEVELS (5)
static const uint8_t contrast_regs[NUM_CONTRAST_LEVELS + 1][7] = {
	{BPADDR, BPDATA, BPADDR, BPDATA, BPDATA, BPDATA, BPDATA}, /* reg addr */
	{0x00, 0x04, 0x07, 0x20, 0x18, 0x34, 0x06},               /* -2 */
	{0x00, 0x04, 0x07, 0x20, 0x1c, 0x2a, 0x06},               /* -1 */
	{0x00, 0x04, 0x07, 0x20, 0x20, 0x20, 0x06},               /*  0 */
	{0x00, 0x04, 0x07, 0x20, 0x24, 0x16, 0x06},               /* +1 */
	{0x00, 0x04, 0x07, 0x20, 0x28, 0x0c, 0x06},               /* +2 */
};

#define NUM_SATURATION_LEVELS (5)
static const uint8_t saturation_regs[NUM_SATURATION_LEVELS + 1][5] = {
	{BPADDR, BPDATA, BPADDR, BPDATA, BPDATA}, /* reg addr */
	{0x00, 0x02, 0x03, 0x28, 0x28},           /* -2 */
	{0x00, 0x02, 0x03, 0x38, 0x38},           /* -1 */
	{0x00, 0x02, 0x03, 0x48, 0x48},           /*  0 */
	{0x00, 0x02, 0x03, 0x58, 0x58},           /* +1 */
	{0x00, 0x02, 0x03, 0x58, 0x58},           /* +2 */
};

struct ov2640_config {
	struct i2c_dt_spec i2c;
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	struct gpio_dt_spec reset_gpio;
#endif
#if DT_INST_NODE_HAS_PROP(0, pwdn_gpios)
	struct gpio_dt_spec pwdn_gpio;
#endif
	uint8_t clock_rate_control;
};

struct ov2640_ctrls {
	struct video_ctrl hflip;
	struct video_ctrl vflip;
	struct video_ctrl ae;
	struct video_ctrl awb;
	struct video_ctrl gain;
	struct video_ctrl contrast;
	struct video_ctrl brightness;
	struct video_ctrl saturation;
	struct video_ctrl jpeg;
	struct video_ctrl test_pattern;
};

struct ov2640_data {
	struct ov2640_ctrls ctrls;
	struct video_format fmt;
};

#define OV2640_VIDEO_FORMAT_CAP(width, height, format)                                             \
	{                                                                                          \
		.pixelformat = (format),                                                           \
		.width_min = (width),                                                              \
		.width_max = (width),                                                              \
		.height_min = (height),                                                            \
		.height_max = (height),                                                            \
		.width_step = 0,                                                                   \
		.height_step = 0,                                                                  \
	}

static const struct video_format_cap fmts[] = {
	OV2640_VIDEO_FORMAT_CAP(QQVGA_WIDTH, QQVGA_HEIGHT,
				VIDEO_PIX_FMT_RGB565), /* 160 x 120 QQVGA */
	OV2640_VIDEO_FORMAT_CAP(QCIF_WIDTH, QCIF_HEIGHT, VIDEO_PIX_FMT_RGB565), /* 176 x 144 QCIF */
	OV2640_VIDEO_FORMAT_CAP(240, 240, VIDEO_PIX_FMT_RGB565),
	OV2640_VIDEO_FORMAT_CAP(QVGA_WIDTH, QVGA_HEIGHT, VIDEO_PIX_FMT_RGB565), /* 320 x 240 QVGA */
	OV2640_VIDEO_FORMAT_CAP(CIF_WIDTH, CIF_HEIGHT, VIDEO_PIX_FMT_RGB565), /* 352 x 288 CIF   */
	OV2640_VIDEO_FORMAT_CAP(VGA_WIDTH, VGA_HEIGHT, VIDEO_PIX_FMT_RGB565), /* 640 x 480 VGA   */
	OV2640_VIDEO_FORMAT_CAP(SVGA_WIDTH, SVGA_HEIGHT, VIDEO_PIX_FMT_RGB565), /* 800 x 600 SVGA */
	OV2640_VIDEO_FORMAT_CAP(XGA_WIDTH, XGA_HEIGHT, VIDEO_PIX_FMT_RGB565), /* 1024 x 768 XVGA  */
	OV2640_VIDEO_FORMAT_CAP(SXGA_WIDTH, SXGA_HEIGHT,
				VIDEO_PIX_FMT_RGB565), /* 1280 x 1024 SXGA  */
	OV2640_VIDEO_FORMAT_CAP(UXGA_WIDTH, UXGA_HEIGHT,
				VIDEO_PIX_FMT_RGB565), /* 1600 x 1200 UXGA  */
	OV2640_VIDEO_FORMAT_CAP(QQVGA_WIDTH, QQVGA_HEIGHT, VIDEO_PIX_FMT_YUYV),
	OV2640_VIDEO_FORMAT_CAP(QCIF_WIDTH, QCIF_HEIGHT, VIDEO_PIX_FMT_YUYV),
	OV2640_VIDEO_FORMAT_CAP(240, 240, VIDEO_PIX_FMT_YUYV),
	OV2640_VIDEO_FORMAT_CAP(QVGA_WIDTH, QVGA_HEIGHT, VIDEO_PIX_FMT_YUYV),
	OV2640_VIDEO_FORMAT_CAP(CIF_WIDTH, CIF_HEIGHT, VIDEO_PIX_FMT_YUYV),
	OV2640_VIDEO_FORMAT_CAP(VGA_WIDTH, VGA_HEIGHT, VIDEO_PIX_FMT_YUYV),
	OV2640_VIDEO_FORMAT_CAP(SVGA_WIDTH, SVGA_HEIGHT, VIDEO_PIX_FMT_YUYV),
	OV2640_VIDEO_FORMAT_CAP(XGA_WIDTH, XGA_HEIGHT, VIDEO_PIX_FMT_YUYV),
	OV2640_VIDEO_FORMAT_CAP(SXGA_WIDTH, SXGA_HEIGHT, VIDEO_PIX_FMT_YUYV),
	OV2640_VIDEO_FORMAT_CAP(UXGA_WIDTH, UXGA_HEIGHT, VIDEO_PIX_FMT_YUYV),
	OV2640_VIDEO_FORMAT_CAP(QQVGA_WIDTH, QQVGA_HEIGHT,
				VIDEO_PIX_FMT_JPEG),                          /* 160 x 120 QQVGA */
	OV2640_VIDEO_FORMAT_CAP(QCIF_WIDTH, QCIF_HEIGHT, VIDEO_PIX_FMT_JPEG), /* 176 x 144 QCIF  */
	OV2640_VIDEO_FORMAT_CAP(CIF_WIDTH, CIF_HEIGHT, VIDEO_PIX_FMT_JPEG),   /* 352 x 288 CIF   */
	OV2640_VIDEO_FORMAT_CAP(240, 240, VIDEO_PIX_FMT_JPEG),
	OV2640_VIDEO_FORMAT_CAP(QVGA_WIDTH, QVGA_HEIGHT, VIDEO_PIX_FMT_JPEG), /* 320 x 240 QVGA */
	OV2640_VIDEO_FORMAT_CAP(VGA_WIDTH, VGA_HEIGHT, VIDEO_PIX_FMT_JPEG),   /* 640 x 480 VGA   */
	OV2640_VIDEO_FORMAT_CAP(SVGA_WIDTH, SVGA_HEIGHT, VIDEO_PIX_FMT_JPEG), /* 800 x 600 SVGA  */
	OV2640_VIDEO_FORMAT_CAP(XGA_WIDTH, XGA_HEIGHT, VIDEO_PIX_FMT_JPEG),   /* 1024 x 768 XVGA  */
	OV2640_VIDEO_FORMAT_CAP(SXGA_WIDTH, SXGA_HEIGHT, VIDEO_PIX_FMT_JPEG), /* 1280 x 1024 SXGA */
	OV2640_VIDEO_FORMAT_CAP(UXGA_WIDTH, UXGA_HEIGHT, VIDEO_PIX_FMT_JPEG), /* 1600 x 1200 UXGA */
	{0}};

static int ov2640_write_reg(const struct i2c_dt_spec *spec, uint8_t reg_addr, uint8_t value)
{
	uint8_t tries = 3;

	/**
	 * It rarely happens that the camera does not respond with ACK signal.
	 * In that case it usually responds on 2nd try but there is a 3rd one
	 * just to be sure that the connection error is not caused by driver
	 * itself.
	 */
	while (tries-- > 0) {
		if (!i2c_reg_write_byte_dt(spec, reg_addr, value)) {
			return 0;
		}
		/* If writing failed wait 5ms before next attempt */
		k_msleep(5);
	}
	LOG_ERR("failed to write 0x%x to 0x%x", value, reg_addr);

	return -1;
}

static int ov2640_read_reg(const struct i2c_dt_spec *spec, uint8_t reg_addr)
{
	uint8_t tries = 3;
	uint8_t value;

	/**
	 * It rarely happens that the camera does not respond with ACK signal.
	 * In that case it usually responds on 2nd try but there is a 3rd one
	 * just to be sure that the connection error is not caused by driver
	 * itself.
	 */
	while (tries-- > 0) {
		if (!i2c_reg_read_byte_dt(spec, reg_addr, &value)) {
			return value;
		}
		/* If reading failed wait 5ms before next attempt */
		k_msleep(5);
	}
	LOG_ERR("failed to read 0x%x register", reg_addr);

	return -1;
}

static int ov2640_write_all(const struct device *dev, const struct ov2640_reg *regs,
			    uint16_t reg_num)
{
	uint16_t i = 0;
	const struct ov2640_config *cfg = dev->config;

	for (i = 0; i < reg_num; i++) {
		int err;

		err = ov2640_write_reg(&cfg->i2c, regs[i].addr, regs[i].value);
		if (err) {
			return err;
		}
	}

	return 0;
}

static int ov2640_soft_reset(const struct device *dev)
{
	int ret = 0;
	const struct ov2640_config *cfg = dev->config;

	/* Switch to DSP register bank */
	ret |= ov2640_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_SENSOR);

	/* Initiate system reset */
	ret |= ov2640_write_reg(&cfg->i2c, COM7, COM7_SRST);

	return ret;
}

static int ov2640_set_level(const struct device *dev, int level, int max_level, int cols,
			    const uint8_t regs[][cols])
{
	int ret = 0;
	const struct ov2640_config *cfg = dev->config;

	level += max_level / 2 + 1;

	/* Switch to DSP register bank */
	ret |= ov2640_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_DSP);

	for (int i = 0; i < (ARRAY_SIZE(regs[0]) / sizeof(regs[0][0])); i++) {
		ret |= ov2640_write_reg(&cfg->i2c, regs[0][i], regs[level][i]);
	}

	return ret;
}

static int ov2640_set_output_format(const struct device *dev, int output_format)
{
	int ret;

	if (output_format == VIDEO_PIX_FMT_JPEG) {
		ret = ov2640_write_all(dev, ov2640_jpeg_regs, ARRAY_SIZE(ov2640_jpeg_regs));
	} else if (output_format == VIDEO_PIX_FMT_RGB565) {
		ret = ov2640_write_all(dev, ov2640_rgb565_regs, ARRAY_SIZE(ov2640_rgb565_regs));
	} else if (output_format == VIDEO_PIX_FMT_YUYV) {
		ret = ov2640_write_all(dev, ov2640_yuyv_regs, ARRAY_SIZE(ov2640_yuyv_regs));
	} else {
		LOG_ERR("Image format not supported");
		return -ENOTSUP;
	}
	k_msleep(10);

	return ret;
}

static int ov2640_set_quality(const struct device *dev, int qs)
{
	int ret = 0;
	const struct ov2640_config *cfg = dev->config;

	/* Switch to DSP register bank */
	ret |= ov2640_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_DSP);

	/* Write QS register */
	ret |= ov2640_write_reg(&cfg->i2c, QS, qs);

	return ret;
}

static int ov2640_set_colorbar(const struct device *dev, uint8_t enable)
{
	int ret = 0;
	const struct ov2640_config *cfg = dev->config;

	uint8_t reg;

	/* Switch to SENSOR register bank */
	ret |= ov2640_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_SENSOR);

	/* Update COM7 to enable/disable color bar test pattern */
	reg = ov2640_read_reg(&cfg->i2c, COM7);

	if (enable) {
		reg |= COM7_COLOR_BAR;
	} else {
		reg &= ~COM7_COLOR_BAR;
	}

	ret |= ov2640_write_reg(&cfg->i2c, COM7, reg);

	return ret;
}

static int ov2640_set_white_bal(const struct device *dev, int enable)
{
	int ret = 0;
	const struct ov2640_config *cfg = dev->config;

	uint8_t reg;

	/* Switch to SENSOR register bank */
	ret |= ov2640_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_SENSOR);

	/* Update CTRL1 to enable/disable automatic white balance*/
	reg = ov2640_read_reg(&cfg->i2c, CTRL1);

	if (enable) {
		reg |= CTRL1_AWB;
	} else {
		reg &= ~CTRL1_AWB;
	}

	ret |= ov2640_write_reg(&cfg->i2c, CTRL1, reg);

	return ret;
}

static int ov2640_set_gain_ctrl(const struct device *dev, int enable)
{
	int ret = 0;
	const struct ov2640_config *cfg = dev->config;

	uint8_t reg;

	/* Switch to SENSOR register bank */
	ret |= ov2640_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_SENSOR);

	/* Update COM8 to enable/disable automatic gain control */
	reg = ov2640_read_reg(&cfg->i2c, COM8);

	if (enable) {
		reg |= COM8_AGC_EN;
	} else {
		reg &= ~COM8_AGC_EN;
	}

	ret |= ov2640_write_reg(&cfg->i2c, COM8, reg);

	return ret;
}

static int ov2640_set_exposure_ctrl(const struct device *dev, int enable)
{
	int ret = 0;
	const struct ov2640_config *cfg = dev->config;

	uint8_t reg;

	/* Switch to SENSOR register bank */
	ret |= ov2640_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_SENSOR);

	/* Update COM8  to enable/disable automatic exposure control */
	reg = ov2640_read_reg(&cfg->i2c, COM8);

	if (enable) {
		reg |= COM8_AEC_EN;
	} else {
		reg &= ~COM8_AEC_EN;
	}

	ret |= ov2640_write_reg(&cfg->i2c, COM8, reg);

	return ret;
}

static int ov2640_set_horizontal_mirror(const struct device *dev, int enable)
{
	int ret = 0;
	const struct ov2640_config *cfg = dev->config;

	uint8_t reg;

	/* Switch to SENSOR register bank */
	ret |= ov2640_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_SENSOR);

	/* Update REG04 to enable/disable horizontal mirror */
	reg = ov2640_read_reg(&cfg->i2c, REG04);

	if (enable) {
		reg |= REG04_HFLIP_IMG;
	} else {
		reg &= ~REG04_HFLIP_IMG;
	}

	ret |= ov2640_write_reg(&cfg->i2c, REG04, reg);

	return ret;
}

static int ov2640_set_vertical_flip(const struct device *dev, int enable)
{
	int ret = 0;
	const struct ov2640_config *cfg = dev->config;

	uint8_t reg;

	/* Switch to SENSOR register bank */
	ret |= ov2640_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_SENSOR);

	/* Update REG04 to enable/disable vertical flip */
	reg = ov2640_read_reg(&cfg->i2c, REG04);

	if (enable) {
		reg |= REG04_VFLIP_IMG | REG04_VREF_EN;
	} else {
		reg &= ~(REG04_VFLIP_IMG | REG04_VREF_EN);
	}

	ret |= ov2640_write_reg(&cfg->i2c, REG04, reg);

	return ret;
}

/*
 * Pick the sensor mode covering the output size, crop the sensor array to the
 * aspect ratio of the output, and let the DSP scale that window down to it.
 */
static int ov2640_set_resolution(const struct device *dev, uint16_t img_width,
				 uint16_t img_height, uint32_t pixelformat)
{
	const struct ov2640_config *cfg = dev->config;
	const struct ov2640_reg *mode_regs;
	size_t mode_size;
	uint16_t off_x = 0;
	uint16_t off_y = 0;
	uint16_t max_x = UXGA_WIDTH;
	uint16_t max_y = UXGA_HEIGHT;
	uint16_t w = img_width / 4;
	uint16_t h = img_height / 4;
	uint8_t pclk_div = 8;
	int ret = 0;

	if (img_width == img_height) {
		off_x = 200;
		max_x = 1200;
	} else if (img_width * 9 == img_height * 11) {
		off_x = 50;
		max_x = 1500;
	}

	if (img_width <= 400 && img_height <= 296) {
		mode_regs = ov2640_cif_mode_regs;
		mode_size = ARRAY_SIZE(ov2640_cif_mode_regs);
		off_x /= 4;
		off_y /= 4;
		max_x /= 4;
		max_y = MIN(max_y / 4, 296);
	} else if (img_width <= SVGA_WIDTH && img_height <= SVGA_HEIGHT) {
		mode_regs = ov2640_svga_mode_regs;
		mode_size = ARRAY_SIZE(ov2640_svga_mode_regs);
		off_x /= 2;
		off_y /= 2;
		max_x /= 2;
		max_y /= 2;
	} else {
		mode_regs = ov2640_uxga_mode_regs;
		mode_size = ARRAY_SIZE(ov2640_uxga_mode_regs);
		pclk_div = 12;
	}

	LOG_DBG("Selected resolution %ux%u", img_width, img_height);

	max_x /= 4;
	max_y /= 4;

	ret |= ov2640_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_DSP);
	ret |= ov2640_write_reg(&cfg->i2c, R_BYPASS, R_BYPASS_DSP_BYPAS);
	ret |= ov2640_write_all(dev, mode_regs, mode_size);

	ret |= ov2640_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_DSP);
	ret |= ov2640_write_reg(&cfg->i2c, HSIZE, max_x & 0xff);
	ret |= ov2640_write_reg(&cfg->i2c, VSIZE, max_y & 0xff);
	ret |= ov2640_write_reg(&cfg->i2c, XOFFL, off_x & 0xff);
	ret |= ov2640_write_reg(&cfg->i2c, YOFFL, off_y & 0xff);
	ret |= ov2640_write_reg(&cfg->i2c, VHYX,
				((max_y >> 1) & 0x80) | ((off_y >> 4) & 0x70) |
				((max_x >> 5) & 0x08) | ((off_x >> 8) & 0x07));
	ret |= ov2640_write_reg(&cfg->i2c, TEST, (max_x >> 2) & 0x80);
	ret |= ov2640_write_reg(&cfg->i2c, ZMOW, w & 0xff);
	ret |= ov2640_write_reg(&cfg->i2c, ZMOH, h & 0xff);
	ret |= ov2640_write_reg(&cfg->i2c, ZMHH, ((h >> 6) & 0x04) | ((w >> 8) & 0x03));

	/* A compressed stream runs the sensor at full speed with a fixed DVP clock divider */
	ret |= ov2640_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_SENSOR);
	ret |= ov2640_write_reg(&cfg->i2c, CLKRC,
				pixelformat == VIDEO_PIX_FMT_JPEG ? 0x00 : cfg->clock_rate_control);
	ret |= ov2640_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_DSP);
	ret |= ov2640_write_reg(&cfg->i2c, R_DVP_SP,
				pixelformat == VIDEO_PIX_FMT_JPEG ? pclk_div
								 : R_DVP_SP_AUTO_MODE | pclk_div);
	ret |= ov2640_write_reg(&cfg->i2c, R_BYPASS, R_BYPASS_DSP_EN);

	k_msleep(10);

	return ret;
}

uint8_t ov2640_check_connection(const struct device *dev)
{
	int ret = 0;
	const struct ov2640_config *cfg = dev->config;

	uint8_t reg_pid_val, reg_ver_val;

	ret |= ov2640_write_reg(&cfg->i2c, BANK_SEL, BANK_SEL_SENSOR);
	reg_pid_val = ov2640_read_reg(&cfg->i2c, REG_PID);
	reg_ver_val = ov2640_read_reg(&cfg->i2c, REG_VER);

	if (REG_PID_VAL != reg_pid_val || REG_VER_VAL != reg_ver_val) {
		LOG_ERR("OV2640 not detected\n");
		return -ENODEV;
	}

	return ret;
}

/* Smallest buffer a compressed frame is given */
#define OV2640_JPEG_MIN_SIZE 16384U

static int ov2640_set_fmt(const struct device *dev, struct video_format *fmt)
{
	struct ov2640_data *drv_data = dev->data;
	uint16_t width, height;
	int ret = 0;
	int i = 0;

	/* We only support RGB565, YUYV and JPEG pixel formats */
	if (fmt->pixelformat != VIDEO_PIX_FMT_RGB565 && fmt->pixelformat != VIDEO_PIX_FMT_YUYV &&
	    fmt->pixelformat != VIDEO_PIX_FMT_JPEG) {
		LOG_ERR("ov2640 camera supports only RGB565, YUYV and JPG pixelformats!");
		return -ENOTSUP;
	}

	width = fmt->width;
	height = fmt->height;

	/* A compressed frame has no fixed length. The sensor compresses well enough for a
	 * fifth of the pixel count, the budget the esp32-camera driver uses as well, with a
	 * floor for the small sizes where the headers weigh the most
	 */
	if (fmt->pixelformat == VIDEO_PIX_FMT_JPEG) {
		fmt->pitch = 0;
		fmt->size = MAX(fmt->width * fmt->height / 5, OV2640_JPEG_MIN_SIZE);
	}

	if (!memcmp(&drv_data->fmt, fmt, sizeof(drv_data->fmt))) {
		/* nothing to do */
		return 0;
	}

	drv_data->fmt = *fmt;

	/* Check if camera is capable of handling given format */
	while (fmts[i].pixelformat) {
		if (fmts[i].width_min == width && fmts[i].height_min == height &&
		    fmts[i].pixelformat == fmt->pixelformat) {
			/* The output format has to follow the window, which resets the DSP */
			ret |= ov2640_set_resolution(dev, fmt->width, fmt->height,
						     fmt->pixelformat);
			ret |= ov2640_set_output_format(dev, fmt->pixelformat);
			return ret;
		}
		i++;
	}

	/* Camera is not capable of handling given format */
	LOG_ERR("Image format not supported\n");
	return -ENOTSUP;
}

static int ov2640_get_fmt(const struct device *dev, struct video_format *fmt)
{
	struct ov2640_data *drv_data = dev->data;

	*fmt = drv_data->fmt;

	return 0;
}

/*
 * Frame rate of the sensor mode a format is sent in, from a 24 MHz reference clock: the
 * CIF, SVGA and UXGA modes each give half the rate of the previous one. A compressed
 * stream runs the sensor clock undivided, a raw one through the clock rate control.
 */
static void ov2640_format_frmival(const struct device *dev, const struct video_format *fmt,
				  struct video_frmival *frmival)
{
	const struct ov2640_config *cfg = dev->config;
	uint8_t clkrc = fmt->pixelformat == VIDEO_PIX_FMT_JPEG ? 0x00 : cfg->clock_rate_control;
	uint32_t fps;

	if (fmt->width <= 400 && fmt->height <= 296) {
		fps = 60;
	} else if (fmt->width <= SVGA_WIDTH && fmt->height <= SVGA_HEIGHT) {
		fps = 30;
	} else {
		fps = 15;
	}

	/* CLKRC[7] doubles the clock, CLKRC[5:0] divides it by one more than its value */
	frmival->numerator = (clkrc & 0x3f) + 1;
	frmival->denominator = (clkrc & BIT(7)) ? fps * 2 : fps;
}

static int ov2640_get_frmival(const struct device *dev, struct video_frmival *frmival)
{
	struct ov2640_data *drv_data = dev->data;

	ov2640_format_frmival(dev, &drv_data->fmt, frmival);

	return 0;
}

/* The rate follows from the format, so the only interval offered is the one it runs at */
static int ov2640_set_frmival(const struct device *dev, struct video_frmival *frmival)
{
	return ov2640_get_frmival(dev, frmival);
}

static int ov2640_enum_frmival(const struct device *dev, struct video_frmival_enum *fie)
{
	if (fie->index > 0) {
		return -EINVAL;
	}

	fie->type = VIDEO_FRMIVAL_TYPE_DISCRETE;
	ov2640_format_frmival(dev, fie->format, &fie->discrete);

	return 0;
}

static int ov2640_set_stream(const struct device *dev, bool enable, enum video_buf_type type)
{
	return 0;
}

static int ov2640_get_caps(const struct device *dev, struct video_caps *caps)
{
	caps->format_caps = fmts;
	return 0;
}

static int ov2640_set_ctrl(const struct device *dev, uint32_t id)
{
	struct ov2640_data *drv_data = dev->data;
	struct ov2640_ctrls *ctrls = &drv_data->ctrls;

	switch (id) {
	case VIDEO_CID_HFLIP:
		return ov2640_set_horizontal_mirror(dev, ctrls->hflip.val);
	case VIDEO_CID_VFLIP:
		return ov2640_set_vertical_flip(dev, ctrls->vflip.val);
	case VIDEO_CID_EXPOSURE:
		return ov2640_set_exposure_ctrl(dev, ctrls->ae.val);
	case VIDEO_CID_WHITE_BALANCE_TEMPERATURE:
		return ov2640_set_white_bal(dev, ctrls->awb.val);
	case VIDEO_CID_GAIN:
		return ov2640_set_gain_ctrl(dev, ctrls->gain.val);
	case VIDEO_CID_BRIGHTNESS:
		return ov2640_set_level(dev, ctrls->brightness.val, NUM_BRIGHTNESS_LEVELS,
					ARRAY_SIZE(brightness_regs[0]), brightness_regs);
	case VIDEO_CID_CONTRAST:
		return ov2640_set_level(dev, ctrls->contrast.val, NUM_CONTRAST_LEVELS,
					ARRAY_SIZE(contrast_regs[0]), contrast_regs);
	case VIDEO_CID_SATURATION:
		return ov2640_set_level(dev, ctrls->saturation.val, NUM_SATURATION_LEVELS,
					ARRAY_SIZE(saturation_regs[0]), saturation_regs);
	case VIDEO_CID_JPEG_COMPRESSION_QUALITY:
		return ov2640_set_quality(dev, ctrls->jpeg.val);
	case VIDEO_CID_TEST_PATTERN:
		return ov2640_set_colorbar(dev, ctrls->test_pattern.val);
	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(video, ov2640_driver_api) = {
	.set_format = ov2640_set_fmt,
	.get_format = ov2640_get_fmt,
	.get_caps = ov2640_get_caps,
	.set_stream = ov2640_set_stream,
	.set_ctrl = ov2640_set_ctrl,
	.set_frmival = ov2640_set_frmival,
	.get_frmival = ov2640_get_frmival,
	.enum_frmival = ov2640_enum_frmival,
};

static int ov2640_init_controls(const struct device *dev)
{
	int ret;
	struct ov2640_data *drv_data = dev->data;
	struct ov2640_ctrls *ctrls = &drv_data->ctrls;

	ret = video_init_ctrl(&ctrls->hflip, dev, VIDEO_CID_HFLIP,
			      (struct video_ctrl_range){.min = 0, .max = 1, .step = 1, .def = 0});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->vflip, dev, VIDEO_CID_VFLIP,
			      (struct video_ctrl_range){.min = 0, .max = 1, .step = 1, .def = 0});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->ae, dev, VIDEO_CID_EXPOSURE,
			      (struct video_ctrl_range){.min = 0, .max = 1, .step = 1, .def = 1});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->awb, dev, VIDEO_CID_WHITE_BALANCE_TEMPERATURE,
			      (struct video_ctrl_range){.min = 0, .max = 1, .step = 1, .def = 1});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->gain, dev, VIDEO_CID_GAIN,
			      (struct video_ctrl_range){.min = 0, .max = 1, .step = 1, .def = 1});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->brightness, dev, VIDEO_CID_BRIGHTNESS,
			      (struct video_ctrl_range){.min = -2, .max = 2, .step = 1, .def = 0});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->contrast, dev, VIDEO_CID_CONTRAST,
			      (struct video_ctrl_range){.min = -2, .max = 2, .step = 1, .def = 0});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->saturation, dev, VIDEO_CID_SATURATION,
			      (struct video_ctrl_range){.min = -2, .max = 2, .step = 1, .def = 0});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(
		&ctrls->jpeg, dev, VIDEO_CID_JPEG_COMPRESSION_QUALITY,
		(struct video_ctrl_range){.min = 5, .max = 100, .step = 1, .def = 50});
	if (ret) {
		return ret;
	}

	return video_init_ctrl(&ctrls->test_pattern, dev, VIDEO_CID_TEST_PATTERN,
			       (struct video_ctrl_range){.min = 0, .max = 1, .step = 1, .def = 0});
}

static int ov2640_init(const struct device *dev)
{

#if DT_INST_NODE_HAS_PROP(0, pwdn_gpios) || DT_INST_NODE_HAS_PROP(0, reset_gpios)
	const struct ov2640_config *cfg = dev->config;
#endif

	int ret = 0;
	/* set default/init format SVGA RGB565 */
	struct video_format fmt = {
		.pixelformat = VIDEO_PIX_FMT_RGB565,
		.width = SVGA_WIDTH,
		.height = SVGA_HEIGHT,
	};

#if DT_INST_NODE_HAS_PROP(0, pwdn_gpios)
	ret = gpio_pin_configure_dt(&cfg->pwdn_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return ret;
	}

	k_sleep(K_MSEC(1));
#endif

#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		return ret;
	}

	k_sleep(K_MSEC(1));
	gpio_pin_set_dt(&cfg->reset_gpio, 0);
	k_sleep(K_MSEC(1));
#endif

	ret = ov2640_check_connection(dev);

	if (ret) {
		return ret;
	}

	ov2640_soft_reset(dev);
	k_msleep(300);

	ov2640_write_all(dev, ov2640_init_regs, ARRAY_SIZE(ov2640_init_regs));

	ret = ov2640_set_fmt(dev, &fmt);
	if (ret) {
		LOG_ERR("Unable to configure default format");
		return -EIO;
	}

	ret |= ov2640_set_exposure_ctrl(dev, 1);
	ret |= ov2640_set_white_bal(dev, 1);

	if (ret) {
		return ret;
	}

	/* Initialize controls */
	return ov2640_init_controls(dev);
}

/* Unique Instance */
static const struct ov2640_config ov2640_cfg_0 = {
	.i2c = I2C_DT_SPEC_INST_GET(0),
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	.reset_gpio = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
#endif
#if DT_INST_NODE_HAS_PROP(0, pwdn_gpios)
	.pwdn_gpio = GPIO_DT_SPEC_INST_GET(0, pwdn_gpios),
#endif
	.clock_rate_control = DT_INST_PROP(0, clock_rate_control),
};
static struct ov2640_data ov2640_data_0;

static int ov2640_init_0(const struct device *dev)
{
	const struct ov2640_config *cfg = dev->config;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
		LOG_ERR("%s: device %s is not ready", dev->name, cfg->reset_gpio.port->name);
		return -ENODEV;
	}
#endif

#if DT_INST_NODE_HAS_PROP(0, pwdn_gpios)
	if (!gpio_is_ready_dt(&cfg->pwdn_gpio)) {
		LOG_ERR("%s: device %s is not ready", dev->name, cfg->pwdn_gpio.port->name);
		return -ENODEV;
	}
#endif

	uint32_t i2c_cfg = I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD);

	if (i2c_configure(cfg->i2c.bus, i2c_cfg)) {
		LOG_ERR("Failed to configure ov2640 i2c interface.");
	}

	return ov2640_init(dev);
}

DEVICE_DT_INST_DEFINE(0, &ov2640_init_0, NULL, &ov2640_data_0, &ov2640_cfg_0, POST_KERNEL,
		      CONFIG_VIDEO_INIT_PRIORITY, &ov2640_driver_api);

VIDEO_DEVICE_DEFINE(ov2640, DEVICE_DT_INST_GET(0), NULL);
