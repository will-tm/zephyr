/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB device controller driver for Bouffalo Lab BL808x SoCs.
 *
 * The BL808 has a USB V2 controller (FOTG210-like) with:
 *  - Dedicated CX (Control Exchange) engine for EP0
 *  - 8 IN + 8 OUT data endpoints (separate)
 *  - Shared FIFO pool (F0-F3 in base regs, F4-F7 in ext regs)
 *  - Built-in DMA engine for FIFO data transfer
 *  - PHY controlled via PDS registers
 *  - Grouped interrupt architecture (G0=CX, G1=FIFO, G2=device)
 */

#define DT_DRV_COMPAT bflb_bl808x_udc

#include "udc_common.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/cache.h>
#include <zephyr/sys/clock.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_bflb_bl808x, CONFIG_UDC_DRIVER_LOG_LEVEL);

#include <soc.h>
#include <bflb_soc.h>
#include <glb_reg.h>
#include <pds_reg.h>
#include <bouffalolab/common/usb_v2_reg.h>

/* EP0 + 4 data endpoints, fixed for all BL808x variants */
#define USB_BL808X_NUM_BIDIR_EPS  5
/* Number of data FIFOs (F0-F3) */
#define USB_BL808X_NUM_DATA_FIFOS 4U

/* Hardware speed encoding in OTG_CSR register */
#define USB_BL808X_SPEED_FULL 0U
#define USB_BL808X_SPEED_LOW  1U
#define USB_BL808X_SPEED_HIGH 2U

/* Endpoint direction encoding for EPMAP registers */
#define USB_BL808X_EP_DIR_IN  0U
#define USB_BL808X_EP_DIR_OUT 1U

/* FIFO direction encoding for FMAP register */
#define USB_BL808X_FIFO_DIR_OUT 0U
#define USB_BL808X_FIFO_DIR_IN  1U
#define USB_BL808X_FIFO_DIR_BID 2U
#define USB_BL808X_FIFO_EP_NONE 15U

/* FIFO config register (FCFG) field layout: 6-bit field per FIFO, 8-bit stride
 */
#define USB_BL808X_FCFG_FIELD_MASK   0x3FU
#define USB_BL808X_FCFG_FIELD_STRIDE 8U

/* VDMA parameter register stride (PS1+PS2 = 8 bytes per FIFO) */
#define USB_BL808X_VDMA_FIFO_STRIDE 8U

/* Endpoint MPS register stride (INMPS/OUTMPS registers are 4 bytes apart) */
#define USB_BL808X_MPS_REG_STRIDE 4U

/* Maximum packet size for a single HS FIFO block (512 bytes) */
#define USB_BL808X_HSFIFOCAP 512U

/* SOF timer reload values per speed (SDK defaults) */
#define USB_BL808X_SOF_TIMER_HS 0x44CU
#define USB_BL808X_SOF_TIMER_FS 0x2710U

/* Time to wait after soft reset before reading speed register */
#define USB_BL808X_RESET_SETTLE_TIME K_MSEC(30)

/* CX_COMEND interrupt bit — not defined in the vendor register header */
#define USB_MCX_COMEND_INT (1U << 3)

/* Timeout for EP DMA completion check workaround */
#define UDC_BL808X_EVT_CHECK_EP_TIME(size) K_MSEC((int)((size) * 10U))

/* Number of DMA completion polls after bus reset (workaround) */
#define USB_BL808X_WA_RESET_PACKETS 12U

/* Per-instance device configuration (from devicetree) */
struct udc_bflb_bl808x_config {
	uint32_t base;
	void (*irq_enable_func)(const struct device *const dev);
	void (*irq_disable_func)(const struct device *const dev);
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	int speed_idx;
};

/* Per-instance runtime state */
struct udc_bflb_bl808x_data {
	/* Per-endpoint last-known transfer direction (true = IN) */
	bool ep_is_in[USB_BL808X_NUM_BIDIR_EPS];
	/* Setup packet received, pending processing in work queue */
	bool setup_received;
	/* Timepoint until which speed register reads are deferred */
	k_timepoint_t reset_expiration;
	/* Remaining DMA completion polls after bus reset (workaround) */
	uint32_t wa_reset_packet_count;
};

/* Work queue event types for deferred USB processing */
enum udc_bflb_bl808x_ev_type {
	/* Start the next queued transfer on an endpoint */
	UDC_BL808X_EVT_XFER,
	/* VDMA complete for control (CX) FIFO */
	UDC_BL808X_EVT_CTRL_END,
	/* VDMA complete for a data endpoint FIFO */
	UDC_BL808X_EVT_END,
	/* Poll for missed DMA completion interrupt (workaround) */
	UDC_BL808X_EVT_CHECK_EP,
};

/* Deferred event submitted to the UDC work queue */
struct udc_bflb_bl808x_ev {
	const struct device *dev;
	uint8_t ep_addr;
	struct k_work_delayable work;
	enum udc_bflb_bl808x_ev_type event;
};

K_MEM_SLAB_DEFINE(udc_bflb_bl808x_ev_slab, sizeof(struct udc_bflb_bl808x_ev),
		  CONFIG_UDC_BFLB_BL808X_EVENT_COUNT, sizeof(void *));

/* Forward declarations */
static void udc_bflb_bl808x_ev_submit(const struct device *const dev,
				      const uint8_t ep_addr,
				      const enum udc_bflb_bl808x_ev_type event,
				      k_timeout_t delay);

/*
 * Register helpers — low-level hardware access
 *
 */

static enum udc_bus_speed
udc_bflb_bl808x_device_speed(const struct device *const dev)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	struct udc_bflb_bl808x_data *const priv = udc_get_private(dev);
	uint32_t speed;

	/* Reset or init ongoing, result would be incorrect */
	while (!sys_timepoint_expired(priv->reset_expiration)) {
		k_msleep(1);
	}

	speed = sys_read32(cfg->base + USB_OTG_CSR_OFFSET);
	speed &= USB_SPD_TYP_HOV_POV_MASK;
	speed = speed >> USB_SPD_TYP_HOV_POV_SHIFT;

	if (speed == USB_BL808X_SPEED_FULL) {
		return UDC_BUS_SPEED_FS;
	} else if (speed == USB_BL808X_SPEED_HIGH) {
		return UDC_BUS_SPEED_HS;
	}

	return UDC_BUS_UNKNOWN;
}

/*
 * Map FIFO index to endpoint index.
 * In FS mode: 1:1 mapping (FIFO N → EP N).
 * In HS mode: FIFOs 0-1 → EP1, FIFOs 2-3 → EP2 (paired for >512B MPS).
 */
static uint8_t udc_bflb_bl808x_fifo_to_ep(const struct device *const dev,
					  const uint8_t fifo)
{
	if (udc_bflb_bl808x_device_speed(dev) == UDC_BUS_SPEED_FS) {
		return fifo;
	}

	if (fifo < 2U) {
		return 1U;
	}

	return 2U;
}

static void udc_bflb_bl808x_cx_done(const struct device *const dev)
{
	uint32_t tmp;
	const struct udc_bflb_bl808x_config *const cfg = dev->config;

	tmp = sys_read32(cfg->base + USB_DEV_CXCFE_OFFSET);
	tmp |= USB_CX_DONE;
	sys_write32(tmp, cfg->base + USB_DEV_CXCFE_OFFSET);
}

static void udc_bflb_bl808x_ep_send_zlp(const struct device *const dev,
					const uint8_t ep_idx)
{
	uint32_t tmp;
	const struct udc_bflb_bl808x_config *const cfg = dev->config;

	tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET +
			 (ep_idx - 1) * USB_BL808X_MPS_REG_STRIDE);
	tmp |= USB_TX0BYTE_IEP1;
	sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET +
				 (ep_idx - 1) * USB_BL808X_MPS_REG_STRIDE);
}

static void udc_bflb_bl808x_fifo_configure(const struct device *const dev,
					   const uint8_t fifo_idx,
					   struct udc_ep_config *const config,
					   const uint8_t block_num,
					   const bool enabled)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;
	uint8_t ep_type = config->attributes & USB_EP_TRANSFER_TYPE_MASK;

	__ASSERT_NO_MSG(fifo_idx >= 1U && fifo_idx <= USB_BL808X_NUM_DATA_FIFOS);

	tmp = sys_read32(cfg->base + USB_DEV_FCFG_OFFSET);
	tmp &= ~(USB_BL808X_FCFG_FIELD_MASK
		 << ((fifo_idx - 1U) * USB_BL808X_FCFG_FIELD_STRIDE));
	tmp |= ((uint32_t)ep_type
		<< ((fifo_idx - 1U) * USB_BL808X_FCFG_FIELD_STRIDE +
		    USB_BLK_TYP_F0_SHIFT));
	tmp |= ((uint32_t)(block_num - 1U)
		<< ((fifo_idx - 1U) * USB_BL808X_FCFG_FIELD_STRIDE +
		    USB_BLKNO_F0_SHIFT));
	if (config->mps > USB_BL808X_HSFIFOCAP) {
		tmp |= (1U << ((fifo_idx - 1U) * USB_BL808X_FCFG_FIELD_STRIDE +
			       USB_BLKSZ_F0));
	}
	if (enabled) {
		tmp |= (1U << ((fifo_idx - 1U) * USB_BL808X_FCFG_FIELD_STRIDE +
			       USB_EN_F0));
	} else {
		tmp &= ~(1U << ((fifo_idx - 1U) * USB_BL808X_FCFG_FIELD_STRIDE +
				USB_EN_F0));
	}
	sys_write32(tmp, cfg->base + USB_DEV_FCFG_OFFSET);
}

static void udc_bflb_bl808x_ep_set_out_mps(const struct device *const dev,
					   const uint8_t ep_idx,
					   const uint16_t ep_mps)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET +
			 (ep_idx - 1) * USB_BL808X_MPS_REG_STRIDE);
	tmp |= USB_RSTG_OEP1;
	sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET +
				 (ep_idx - 1) * USB_BL808X_MPS_REG_STRIDE);

	tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET +
			 (ep_idx - 1) * USB_BL808X_MPS_REG_STRIDE);
	tmp &= ~USB_RSTG_OEP1;
	sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET +
				 (ep_idx - 1) * USB_BL808X_MPS_REG_STRIDE);

	tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET +
			 (ep_idx - 1) * USB_BL808X_MPS_REG_STRIDE);
	tmp &= ~USB_MAXPS_OEP1_MASK;
	tmp |= ep_mps;
	sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET +
				 (ep_idx - 1) * USB_BL808X_MPS_REG_STRIDE);
}

static void udc_bflb_bl808x_ep_set_in_mps(const struct device *const dev,
					  const uint8_t ep_idx,
					  const uint16_t ep_mps)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET +
			 (ep_idx - 1) * USB_BL808X_MPS_REG_STRIDE);
	tmp |= USB_RSTG_IEP1;
	sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET +
				 (ep_idx - 1) * USB_BL808X_MPS_REG_STRIDE);

	tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET +
			 (ep_idx - 1) * USB_BL808X_MPS_REG_STRIDE);
	tmp &= ~USB_RSTG_IEP1;
	sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET +
				 (ep_idx - 1) * USB_BL808X_MPS_REG_STRIDE);

	tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET +
			 (ep_idx - 1) * USB_BL808X_MPS_REG_STRIDE);
	tmp &= ~USB_MAXPS_IEP1_MASK;
	tmp |= ep_mps;
	tmp &= ~USB_TX_NUM_HBW_IEP1_MASK;
	sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET +
				 (ep_idx - 1) * USB_BL808X_MPS_REG_STRIDE);
}

static void udc_bflb_bl808x_cx_fifo_reset(const struct device *const dev)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_DEV_CXCFE_OFFSET);
	tmp |= USB_CX_CLR;
	sys_write32(tmp, cfg->base + USB_DEV_CXCFE_OFFSET);
}

static void udc_bflb_bl808x_fifo_reset(const struct device *const dev,
				       const uint8_t fifo_idx)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;

	__ASSERT_NO_MSG(fifo_idx >= 1U && fifo_idx <= USB_BL808X_NUM_DATA_FIFOS);

	tmp = sys_read32(cfg->base + USB_DEV_FIBC0_OFFSET +
			 4U * (fifo_idx - 1U));
	tmp |= USB_FFRST0_HOV;
	sys_write32(tmp,
		    cfg->base + USB_DEV_FIBC0_OFFSET + 4U * (fifo_idx - 1U));
}

/*
 * Map an endpoint to a FIFO in the EPMAP register.
 * ep_idx/fifo_idx are 1-based; ep_dir: 0=IN, 1=OUT.
 * EPMAP0 covers EP1-4, EPMAP1 covers EP5-8.
 */
static void udc_bflb_bl808x_epmap_set(const struct device *const dev,
				      const uint8_t ep_idx,
				      const uint8_t fifo_idx,
				      const uint8_t ep_dir)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;
	const uint8_t ep_dir_bit = ep_dir * 4U;

	if (ep_idx < 5U) {
		tmp = sys_read32(cfg->base + USB_DEV_EPMAP0_OFFSET);
		tmp &= ~(0xFU << ((ep_idx - 1U) * 8U + ep_dir_bit));
		tmp |= ((uint32_t)(fifo_idx - 1U)
			<< ((ep_idx - 1U) * 8U + ep_dir_bit));
		sys_write32(tmp, cfg->base + USB_DEV_EPMAP0_OFFSET);
	} else {
		tmp = sys_read32(cfg->base + USB_DEV_EPMAP1_OFFSET);
		tmp &= ~(0xFU << ((ep_idx - 5U) * 8U + ep_dir_bit));
		tmp |= ((uint32_t)(fifo_idx - 1U)
			<< ((ep_idx - 5U) * 8U + ep_dir_bit));
		sys_write32(tmp, cfg->base + USB_DEV_EPMAP1_OFFSET);
	}
}

/*
 * Map a FIFO to an endpoint in the FMAP register.
 * ep_idx/fifo_idx are 1-based; fifo_dir: 0=OUT, 1=IN, 2=bidirectional.
 * Use USB_BL808X_FIFO_EP_NONE as ep_idx to disconnect a FIFO.
 */
static void udc_bflb_bl808x_fmap_set(const struct device *const dev,
				     const uint8_t ep_idx,
				     const uint8_t fifo_idx,
				     const uint8_t fifo_dir)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;

	__ASSERT_NO_MSG(fifo_idx >= 1U && fifo_idx <= USB_BL808X_NUM_DATA_FIFOS);
	__ASSERT_NO_MSG(fifo_dir <= USB_BL808X_FIFO_DIR_BID);

	tmp = sys_read32(cfg->base + USB_DEV_FMAP_OFFSET);
	tmp &= ~(USB_BL808X_FCFG_FIELD_MASK
		 << ((fifo_idx - 1U) * USB_BL808X_FCFG_FIELD_STRIDE));
	tmp |= ((uint32_t)ep_idx
		<< ((fifo_idx - 1U) * USB_BL808X_FCFG_FIELD_STRIDE));
	tmp |= ((uint32_t)fifo_dir
		<< ((fifo_idx - 1U) * USB_BL808X_FCFG_FIELD_STRIDE +
		    USB_DIR_FIFO0_SHIFT));
	sys_write32(tmp, cfg->base + USB_DEV_FMAP_OFFSET);
}

/*
 * VDMA (Virtual DMA) — all USB data transfers use the built-in DMA engine
 *
 */

static void udc_bflb_bl808x_vdma_startread(const struct device *const dev,
					   const uint8_t fifo_idx,
					   uint8_t *const buf,
					   const uint32_t len)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	const uint32_t fifo_off = (fifo_idx - 1U) * USB_BL808X_VDMA_FIFO_STRIDE;
	uint32_t tmp;

	sys_cache_data_flush_and_invd_range(buf, len);

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);
	tmp &= ~USB_VDMA_LEN_CXF_MASK;
	tmp &= ~USB_VDMA_IO_CXF;
	tmp &= ~USB_VDMA_TYPE_CXF;
	tmp |= (len << USB_VDMA_LEN_CXF_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);

	sys_write32((uint32_t)buf, cfg->base + USB_VDMA_F0PS2_OFFSET + fifo_off);

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);
	tmp |= USB_VDMA_START_CXF;
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);
}

static void udc_bflb_bl808x_vdma_startwrite(const struct device *const dev,
					    const uint8_t fifo_idx,
					    uint8_t *data, const uint32_t len)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	const uint32_t fifo_off = (fifo_idx - 1U) * USB_BL808X_VDMA_FIFO_STRIDE;
	uint32_t tmp;

	sys_cache_data_flush_and_invd_range(data, len);

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);
	tmp &= ~USB_VDMA_LEN_CXF_MASK;
	tmp &= ~USB_VDMA_IO_CXF;
	tmp |= USB_VDMA_TYPE_CXF;
	tmp |= (len << USB_VDMA_LEN_CXF_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);

	sys_write32((uint32_t)data,
		    cfg->base + USB_VDMA_F0PS2_OFFSET + fifo_off);

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);
	tmp |= USB_VDMA_START_CXF;
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);
}

static void udc_bflb_bl808x_vdma_startread_ctrl(const struct device *const dev,
						uint8_t *buf, uint32_t len)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	struct udc_bflb_bl808x_data *const priv = udc_get_private(dev);
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp &= ~USB_VDMA_LEN_CXF_MASK;
	tmp &= ~USB_VDMA_IO_CXF;
	tmp &= ~USB_VDMA_TYPE_CXF;
	tmp |= (len << USB_VDMA_LEN_CXF_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);

	sys_write32((uint32_t)buf, cfg->base + USB_VDMA_CXFPS2_OFFSET);

	priv->ep_is_in[0] = false;

	sys_cache_data_flush_and_invd_range(buf, len);

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp |= USB_VDMA_START_CXF;
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);
}

static void udc_bflb_bl808x_vdma_startwrite_ctrl(const struct device *const dev,
						 uint8_t *data,
						 const uint32_t len)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	struct udc_bflb_bl808x_data *const priv = udc_get_private(dev);
	uint32_t tmp;

	sys_cache_data_flush_and_invd_range(data, len);

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp &= ~USB_VDMA_LEN_CXF_MASK;
	tmp &= ~USB_VDMA_IO_CXF;
	tmp |= USB_VDMA_TYPE_CXF;
	tmp |= (len << USB_VDMA_LEN_CXF_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);

	sys_write32((uint32_t)data, cfg->base + USB_VDMA_CXFPS2_OFFSET);

	priv->ep_is_in[0] = true;

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp |= USB_VDMA_START_CXF;
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);
}

static uint8_t udc_bflb_bl808x_ep_to_fifo(struct udc_ep_config *const ep_cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);

	if (ep_cfg->mps > USB_BL808X_HSFIFOCAP) {
		if (ep_idx == 1) {
			return 1;
		} else {
			return 3;
		}
	}

	return ep_idx;
}

static int udc_bflb_bl808x_set_address(const struct device *const dev,
				       const uint8_t addr)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;

	if ((sys_read32(cfg->base + USB_DEV_ADR_OFFSET) & USB_DEVADR_MASK) !=
	    addr) {
		LOG_DBG("Set new address %u for %p", addr, dev);
		tmp = sys_read32(cfg->base + USB_DEV_ADR_OFFSET);
		tmp &= ~USB_DEVADR_MASK;
		tmp |= addr;
		sys_write32(tmp, cfg->base + USB_DEV_ADR_OFFSET);
	} else {
		LOG_DBG("Address %u already set for %p", addr, dev);
	}

	return 0;
}

/*
 * Control (CX) transfer handling
 *
 */

static uint32_t udc_bflb_bl808x_cx_vdma_remaining(const struct device *const dev)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = (sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET) &
	       USB_VDMA_LEN_CXF_MASK);

	return (tmp >> USB_VDMA_LEN_CXF_SHIFT);
}

static void udc_bflb_bl808x_ctrl_setup_start(const struct device *const dev)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	struct net_buf *buf;
	struct udc_ep_config *const ep_cfg =
		udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	uint32_t tmp;
	uint32_t *setup_data;

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 8U);
	if (buf == NULL) {
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOMEM);
		return;
	}

	udc_ep_buf_set_setup(buf);
	udc_buf_put(ep_cfg, buf);
	net_buf_add(buf, 8);

	/* Read setup packet directly from CX FIFO register port. */
	tmp = sys_read32(cfg->base + USB_DMA_TFN_OFFSET);
	tmp |= USB_ACC_CXF_HOV;
	sys_write32(tmp, cfg->base + USB_DMA_TFN_OFFSET);

	setup_data = (uint32_t *)buf->data;
	setup_data[0] = sys_read32(cfg->base + USB_DMA_CPS3_OFFSET);
	setup_data[1] = sys_read32(cfg->base + USB_DMA_CPS3_OFFSET);

	tmp = sys_read32(cfg->base + USB_DMA_TFN_OFFSET);
	tmp &= ~USB_ACC_CXF_HOV;
	sys_write32(tmp, cfg->base + USB_DMA_TFN_OFFSET);

	udc_bflb_bl808x_ev_submit(dev, USB_CONTROL_EP_OUT,
				  UDC_BL808X_EVT_CTRL_END, K_NO_WAIT);
}

static void udc_bflb_bl808x_ctrl_dout_start(const struct device *const dev,
					    const uint16_t size)
{
	struct net_buf *buf;
	struct udc_ep_config *const ep_cfg =
		udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);

	LOG_DBG("ctrl dout start ep 0x%02x", ep_cfg->addr);

	if (!udc_ctrl_stage_is_data_out(dev) ||
	    udc_bflb_bl808x_cx_vdma_remaining(dev) != 0) {
		LOG_ERR("Unexpected control dout token");
		return;
	}

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, size);
	if (buf == NULL) {
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOMEM);
		return;
	}

	udc_buf_put(ep_cfg, buf);
	net_buf_add(buf, size);

	udc_bflb_bl808x_vdma_startread_ctrl(dev, buf->data, size);
}

static void udc_bflb_bl808x_ctrl_din_start(const struct device *const dev)
{
	struct net_buf *buf;
	struct udc_ep_config *const ep_cfg =
		udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);

	LOG_DBG("ctrl din start ep 0x%02x", ep_cfg->addr);

	if (!udc_ctrl_stage_is_data_in(dev) ||
	    udc_bflb_bl808x_cx_vdma_remaining(dev) != 0) {
		LOG_ERR("Unexpected control din token");
		return;
	}

	buf = udc_buf_peek(udc_get_ep_cfg(dev, USB_CONTROL_EP_IN));
	if (buf == NULL) {
		udc_submit_event(dev, UDC_EVT_ERROR, -ENODATA);
		return;
	}

	LOG_DBG("start DMA for buf %p, data %p, len %i", (void *)buf,
		(void *)buf->data, buf->len);
	udc_bflb_bl808x_vdma_startwrite_ctrl(dev, buf->data, buf->len);
}

static int udc_bflb_bl808x_ctrl_xfer_done(const struct device *const dev)
{
	struct udc_bflb_bl808x_data *const priv = udc_get_private(dev);
	struct net_buf *buf;
	int err;

	if (priv->setup_received) {
		const struct udc_bflb_bl808x_config *const cfg = dev->config;
		uint32_t tmp;
		int ret;

		buf = udc_buf_get(udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT));
		if (buf == NULL) {
			/* Setup buf was drained by a bus reset — stale event */
			LOG_WRN("Setup buf drained by reset");
			priv->setup_received = false;
			tmp = sys_read32(cfg->base + USB_DEV_MISG0_OFFSET);
			tmp &= ~USB_MCX_SETUP_INT;
			sys_write32(tmp, cfg->base + USB_DEV_MISG0_OFFSET);
			return 0;
		}

		udc_ctrl_update_stage(dev, buf);
		priv->setup_received = false;

		if (udc_ctrl_stage_is_data_in(dev)) {
			ret = udc_ctrl_submit_s_in_status(dev);
		} else if (udc_ctrl_stage_is_data_out(dev)) {
			udc_bflb_bl808x_ctrl_dout_start(
				dev, udc_data_stage_length(buf));
			ret = 0;
		} else if (udc_ctrl_stage_is_no_data(dev)) {
			struct usb_setup_packet *spkg =
				(struct usb_setup_packet *)buf->data;

			if (spkg->bRequest == USB_SREQ_SET_ADDRESS) {
				udc_bflb_bl808x_set_address(dev, spkg->wValue);
			}
			ret = udc_ctrl_submit_s_status(dev);
		} else {
			ret = -EINVAL;
		}

		/* Re-enable setup interrupt after submit completes */
		tmp = sys_read32(cfg->base + USB_DEV_MISG0_OFFSET);
		tmp &= ~USB_MCX_SETUP_INT;
		sys_write32(tmp, cfg->base + USB_DEV_MISG0_OFFSET);

		return ret;
	} else if (udc_ctrl_stage_is_data_out(dev)) {
		buf = udc_buf_get(udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT));
		udc_ctrl_update_stage(dev, buf);
		return udc_ctrl_submit_s_out_status(dev, buf);
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		buf = udc_buf_get(udc_get_ep_cfg(dev, USB_CONTROL_EP_IN));
		if (buf == NULL) {
			LOG_ERR("No buf for DATA_IN completion");
			return -ENODATA;
		}
		/* Use data buf (EP_IN) for stage transition, like DWC2 */
		udc_ctrl_update_stage(dev, buf);
		if (udc_ctrl_stage_is_status_out(dev)) {
			net_buf_unref(buf);
			/* FOTG210 handles status OUT automatically after
			 * CX_DONE. Allocate a status buf to notify the
			 * USB stack that the transfer is complete.
			 */
			buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 0U);
			if (buf == NULL) {
				return -ENOMEM;
			}
			err = udc_ctrl_submit_status(dev, buf);
			udc_ctrl_update_stage(dev, buf);
			return err;
		}
		net_buf_unref(buf);
	} else if (udc_ctrl_stage_is_no_data(dev) ||
		   udc_ctrl_stage_is_status_in(dev)) {
		/*
		 * CX_COMEND after status IN ZLP was sent.
		 * Notify USB stack and transition to SETUP.
		 */
		buf = udc_buf_get(udc_get_ep_cfg(dev, USB_CONTROL_EP_IN));
		if (buf != NULL) {
			udc_ctrl_submit_status(dev, buf);
			udc_ctrl_update_stage(dev, buf);
		}
	} else {
		LOG_DBG("Control transfer completed (no pending stage)");
	}

	return 0;
}

/*
 * Data endpoint transfer handling
 *
 */

static uint32_t udc_bflb_bl808x_ep_vdma_remaining(const struct device *const dev,
						  const uint8_t fifo_idx)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = (sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET +
			  (fifo_idx - 1U) * USB_BL808X_VDMA_FIFO_STRIDE) &
	       USB_VDMA_LEN_CXF_MASK);

	return (tmp >> USB_VDMA_LEN_CXF_SHIFT);
}

static void udc_bflb_bl808x_ep_dout_start(const struct device *const dev,
					  struct udc_ep_config *const ep_cfg)
{
	struct udc_bflb_bl808x_data *const priv = udc_get_private(dev);
	struct net_buf *buf;
	uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);

	LOG_DBG("dout start ep 0x%02x", ep_cfg->addr);

	if (priv->ep_is_in[ep_idx]) {
		LOG_ERR("Unexpected ep 0x%02x dout token", ep_cfg->addr);
	}

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		LOG_ERR("No buffer for OUT ep 0x%02x", ep_cfg->addr);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
	} else {
		priv->ep_is_in[ep_idx] = false;
		udc_bflb_bl808x_vdma_startread(
			dev, udc_bflb_bl808x_ep_to_fifo(ep_cfg), buf->data,
			buf->size);
		if (priv->wa_reset_packet_count > 0) {
			udc_bflb_bl808x_ev_submit(
				dev, ep_cfg->addr, UDC_BL808X_EVT_CHECK_EP,
				UDC_BL808X_EVT_CHECK_EP_TIME(buf->size));
			priv->wa_reset_packet_count--;
		}
	}
}

static void udc_bflb_bl808x_ep_din_start(const struct device *const dev,
					 struct udc_ep_config *const ep_cfg)
{
	struct udc_bflb_bl808x_data *const priv = udc_get_private(dev);
	struct net_buf *buf;
	const uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);

	LOG_DBG("din start ep 0x%02x", ep_cfg->addr);

	if (!priv->ep_is_in[ep_idx]) {
		LOG_ERR("Unexpected ep 0x%02x din token", ep_cfg->addr);
	}

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		LOG_ERR("No buffer for IN ep 0x%02x", ep_cfg->addr);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
	} else {
		priv->ep_is_in[ep_idx] = true;
		udc_bflb_bl808x_vdma_startwrite(
			dev, udc_bflb_bl808x_ep_to_fifo(ep_cfg), buf->data,
			buf->len);
		if (priv->wa_reset_packet_count > 0) {
			udc_bflb_bl808x_ev_submit(
				dev, ep_cfg->addr, UDC_BL808X_EVT_CHECK_EP,
				UDC_BL808X_EVT_CHECK_EP_TIME(buf->len));
			priv->wa_reset_packet_count--;
		}
	}
}

static int udc_bflb_bl808x_ep_xfer_done(const struct device *const dev,
					struct udc_ep_config *const ep_cfg)
{
	struct net_buf *buf;
	uint32_t remain = 0;

	buf = udc_buf_get(ep_cfg);
	if (buf == NULL) {
		LOG_ERR("No buf for ep 0x%02x event end", ep_cfg->addr);
		return -ENODATA;
	}
	LOG_DBG("Event end for 0x%02x buf %lx, len %u, size %u", ep_cfg->addr,
		(uintptr_t)buf, buf->len, buf->size);
	if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
		remain = udc_bflb_bl808x_ep_vdma_remaining(
			dev, udc_bflb_bl808x_ep_to_fifo(ep_cfg));
		LOG_DBG("%u bytes transferred out of %u, %u bytes remaining",
			ep_cfg->mps - remain, ep_cfg->mps, remain);
		net_buf_add(buf, ep_cfg->mps - remain);
	} else {
		net_buf_pull(buf, buf->len);
	}

	return udc_submit_ep_event(dev, buf, 0);
}

/*
 * Work queue — deferred event processing outside ISR context
 *
 */

static void udc_bflb_bl808x_work_handler_xfer(const struct device *const dev,
					      struct udc_ep_config *const ep_cfg)
{
	struct udc_bflb_bl808x_data *const priv = udc_get_private(dev);
	const struct net_buf *buf = udc_buf_peek(ep_cfg);
	const uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);

	if (buf == NULL) {
		LOG_ERR("No buffer queued for ep 0x%02x xfer", ep_cfg->addr);
		return;
	}

	if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
		priv->ep_is_in[ep_idx] = false;
		udc_ep_set_busy(ep_cfg, true);
		udc_bflb_bl808x_ep_dout_start(dev, ep_cfg);
	} else if (buf->len == 0) {
		LOG_DBG("IN ep 0x%02x: zero-length packet", ep_cfg->addr);
		udc_bflb_bl808x_ep_send_zlp(dev, ep_idx);
		udc_bflb_bl808x_ep_xfer_done(dev, ep_cfg);
	} else {
		if (udc_get_buf_info(buf)->zlp) {
			LOG_DBG("IN: ZLP");
		}
		priv->ep_is_in[ep_idx] = true;
		udc_ep_set_busy(ep_cfg, true);
		udc_bflb_bl808x_ep_din_start(dev, ep_cfg);
	}
}

static void
udc_bflb_bl808x_work_handler_check(const struct device *const dev,
				   struct udc_ep_config *const ep_cfg)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	int err;
	uint32_t done = sys_read32(cfg->base + USB_DEV_ISG3_OFFSET) &
			(1U << udc_bflb_bl808x_ep_to_fifo(ep_cfg));

	if (udc_ep_is_busy(ep_cfg)) {
		err = udc_bflb_bl808x_ep_xfer_done(dev, ep_cfg);
		udc_ep_set_busy(ep_cfg, false);
		if (unlikely(err)) {
			udc_submit_event(dev, UDC_EVT_ERROR, err);
		}
		sys_write32(done, cfg->base + USB_DEV_ISG3_OFFSET);
	} else {
		/* Not busy, interrupt worked, we have nothing to do */
		return;
	}
}

static void udc_bflb_bl808x_work_handler(struct k_work *item)
{
	struct k_work_delayable *item_delayable =
		k_work_delayable_from_work(item);
	const struct udc_bflb_bl808x_ev *const ev =
		CONTAINER_OF(item_delayable, struct udc_bflb_bl808x_ev, work);
	struct udc_ep_config *const ep_cfg =
		udc_get_ep_cfg(ev->dev, ev->ep_addr);
	int err = 0;

	LOG_DBG("dev %p, ep 0x%02x, event %u", ev->dev, ev->ep_addr, ev->event);

	if (unlikely(ep_cfg == NULL)) {
		err = -ENODATA;
		LOG_ERR("Invalid endpoint config in work queue");
	} else {
		switch (ev->event) {
		case UDC_BL808X_EVT_CTRL_END:
			err = udc_bflb_bl808x_ctrl_xfer_done(ev->dev);
			break;
		case UDC_BL808X_EVT_END:
			if (udc_ep_is_busy(ep_cfg)) {
				err = udc_bflb_bl808x_ep_xfer_done(ev->dev,
								   ep_cfg);
				udc_ep_set_busy(ep_cfg, false);
			}
			break;
		case UDC_BL808X_EVT_XFER:
			udc_bflb_bl808x_work_handler_xfer(ev->dev, ep_cfg);
			break;
		case UDC_BL808X_EVT_CHECK_EP:
			udc_bflb_bl808x_work_handler_check(ev->dev, ep_cfg);
			break;
		default:
			break;
		}
	}

	if (unlikely(err)) {
		udc_submit_event(ev->dev, UDC_EVT_ERROR, err);
	}

	k_mem_slab_free(&udc_bflb_bl808x_ev_slab, (void *)ev);
}

static void udc_bflb_bl808x_ev_submit(const struct device *const dev,
				      const uint8_t ep_addr,
				      const enum udc_bflb_bl808x_ev_type event,
				      k_timeout_t delay)
{
	struct udc_bflb_bl808x_ev *ev;
	int ret;

	LOG_DBG("Submit ep 0x%02x event %u", ep_addr, event);

	ret = k_mem_slab_alloc(&udc_bflb_bl808x_ev_slab, (void **)&ev,
			       K_NO_WAIT);
	if (ret < 0) {
		udc_submit_event(dev, UDC_EVT_ERROR, ret);
		LOG_ERR("Failed to allocate slab");
		return;
	}

	ev->dev = dev;
	ev->ep_addr = ep_addr;
	ev->event = event;
	k_work_init_delayable(&ev->work, udc_bflb_bl808x_work_handler);
	ret = k_work_schedule_for_queue(udc_get_work_q(), &ev->work, delay);
	if (ret < 0) {
		udc_submit_event(dev, UDC_EVT_ERROR, ret);
		LOG_ERR("Failed to submit event");
		return;
	}
}

/*
 * UDC API implementation
 *
 */

static int udc_bflb_bl808x_ep_enqueue(const struct device *const dev,
				      struct udc_ep_config *const config,
				      struct net_buf *buf)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(config->addr);

	LOG_DBG("%p enqueue %p for ep 0x%02x", dev, buf, config->addr);

	if (config->stat.halted) {
		LOG_DBG("ep 0x%02x halted", config->addr);
		return 0;
	}

	if (udc_ep_is_busy(config)) {
		return -EBUSY;
	}

	if (ep_idx == 0) {
		if (USB_EP_DIR_IS_OUT(config->addr)) {
			udc_buf_put(config, buf);
			udc_bflb_bl808x_ctrl_dout_start(
				dev, udc_data_stage_length(buf));
		} else if (buf->len == 0) {
			/* Status IN (ZLP): tell hardware to complete the
			 * control transfer, then notify the USB stack.
			 * By this point the setup buf is already freed
			 * and buf is standalone (not a frag).
			 */
			udc_bflb_bl808x_cx_done(dev);
			udc_ctrl_update_stage(dev, buf);
			udc_ctrl_submit_status(dev, buf);
		} else {
			udc_buf_put(config, buf);
			udc_bflb_bl808x_ctrl_din_start(dev);
		}
	} else {
		if (udc_buf_peek(config) == NULL) {
			udc_buf_put(config, buf);
			udc_bflb_bl808x_work_handler_xfer(dev, config);
		} else {
			udc_buf_put(config, buf);
			udc_bflb_bl808x_ev_submit(dev, config->addr,
						  UDC_BL808X_EVT_XFER,
						  K_NO_WAIT);
		}
	}

	return 0;
}

static int udc_bflb_bl808x_ep_dequeue(const struct device *const dev,
				      struct udc_ep_config *const ep_cfg)
{
	unsigned int lock_key;
	struct net_buf *buf;

	lock_key = irq_lock();

	buf = udc_buf_get_all(ep_cfg);
	if (buf != NULL) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	irq_unlock(lock_key);

	return 0;
}

static int udc_bflb_bl808x_ep_enable(const struct device *const dev,
				     struct udc_ep_config *const config)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;
	const uint8_t ep_idx = USB_EP_GET_IDX(config->addr);

	LOG_DBG("Enable ep 0x%02x", config->addr);

	if (USB_EP_DIR_IS_OUT(config->addr)) {
		udc_bflb_bl808x_ep_set_out_mps(dev, ep_idx, config->mps);
	} else {
		udc_bflb_bl808x_ep_set_in_mps(dev, ep_idx, config->mps);
	}

	if (config->mps > USB_BL808X_HSFIFOCAP) {
		if (ep_idx > 2) {
			LOG_ERR("HS dual-FIFO only supported for EP1-2");
			return -ENOTSUP;
		}
		if (ep_idx == 1) {
			/* EP1 uses FIFO pair 1+2 for >512B MPS */
			udc_bflb_bl808x_epmap_set(dev, ep_idx, 1,
						  USB_BL808X_EP_DIR_IN);
			udc_bflb_bl808x_epmap_set(dev, ep_idx, 1,
						  USB_BL808X_EP_DIR_OUT);
			udc_bflb_bl808x_fmap_set(dev, ep_idx, 1,
						 USB_BL808X_FIFO_DIR_BID);
			udc_bflb_bl808x_fmap_set(dev, ep_idx, 2,
						 USB_BL808X_FIFO_DIR_BID);
			udc_bflb_bl808x_fifo_configure(dev, 1, config, 1, true);
			udc_bflb_bl808x_fifo_configure(dev, 2, config, 1, false);
		} else if (ep_idx == 2) {
			/* EP2 uses FIFO pair 3+4 for >512B MPS */
			udc_bflb_bl808x_epmap_set(dev, ep_idx, 3,
						  USB_BL808X_EP_DIR_IN);
			udc_bflb_bl808x_epmap_set(dev, ep_idx, 3,
						  USB_BL808X_EP_DIR_OUT);
			udc_bflb_bl808x_fmap_set(dev, ep_idx, 3,
						 USB_BL808X_FIFO_DIR_BID);
			udc_bflb_bl808x_fmap_set(dev, ep_idx, 4,
						 USB_BL808X_FIFO_DIR_BID);
			udc_bflb_bl808x_fifo_configure(dev, 3, config, 1, true);
			udc_bflb_bl808x_fifo_configure(dev, 4, config, 1, false);
		}
	} else {
		udc_bflb_bl808x_epmap_set(dev, ep_idx, ep_idx,
					  USB_BL808X_EP_DIR_IN);
		udc_bflb_bl808x_epmap_set(dev, ep_idx, ep_idx,
					  USB_BL808X_EP_DIR_OUT);
		udc_bflb_bl808x_fmap_set(dev, ep_idx, ep_idx,
					 USB_BL808X_FIFO_DIR_BID);
		udc_bflb_bl808x_fifo_configure(dev, ep_idx, config, 1, true);
	}

	tmp = sys_read32(cfg->base + USB_DEV_ADR_OFFSET);
	tmp |= USB_AFT_CONF;
	sys_write32(tmp, cfg->base + USB_DEV_ADR_OFFSET);

	return 0;
}

static int udc_bflb_bl808x_ep_disable(const struct device *const dev,
				      struct udc_ep_config *const config)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(config->addr);

	LOG_DBG("Disable ep 0x%02x", config->addr);

	if (ep_idx == 0) {
		return 0;
	}

	/* Reset the FIFO(s) associated with this endpoint */
	if (config->mps > USB_BL808X_HSFIFOCAP) {
		if (ep_idx == 1) {
			udc_bflb_bl808x_fifo_reset(dev, 1);
			udc_bflb_bl808x_fifo_reset(dev, 2);
		} else if (ep_idx == 2) {
			udc_bflb_bl808x_fifo_reset(dev, 3);
			udc_bflb_bl808x_fifo_reset(dev, 4);
		}
	} else {
		udc_bflb_bl808x_fifo_reset(dev, ep_idx);
	}

	return 0;
}

static int udc_bflb_bl808x_ep_set_halt(const struct device *const dev,
				       struct udc_ep_config *const config)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;
	const uint8_t ep_idx = USB_EP_GET_IDX(config->addr);

	LOG_DBG("Set halt ep 0x%02x", config->addr);

	if (ep_idx == 0) {
		tmp = sys_read32(cfg->base + USB_DEV_CXCFE_OFFSET);
		tmp |= USB_CX_STL;
		sys_write32(tmp, cfg->base + USB_DEV_CXCFE_OFFSET);
	} else {
		if (USB_EP_DIR_IS_OUT(config->addr)) {
			tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET +
					 (ep_idx - 1) *
						 USB_BL808X_MPS_REG_STRIDE);
			tmp |= USB_STL_OEP1;
			sys_write32(tmp,
				    cfg->base + USB_DEV_OUTMPS1_OFFSET +
					    (ep_idx - 1) *
						    USB_BL808X_MPS_REG_STRIDE);
		} else {
			tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET +
					 (ep_idx - 1) *
						 USB_BL808X_MPS_REG_STRIDE);
			tmp |= USB_STL_IEP1;
			sys_write32(tmp,
				    cfg->base + USB_DEV_INMPS1_OFFSET +
					    (ep_idx - 1) *
						    USB_BL808X_MPS_REG_STRIDE);
		}
		config->stat.halted = true;
	}

	return 0;
}

static int udc_bflb_bl808x_ep_clear_halt(const struct device *const dev,
					 struct udc_ep_config *const config)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;
	const uint8_t ep_idx = USB_EP_GET_IDX(config->addr);

	LOG_DBG("Clear halt ep 0x%02x", config->addr);

	if (ep_idx == 0) {
		tmp = sys_read32(cfg->base + USB_DEV_CXCFE_OFFSET);
		tmp &= ~USB_CX_STL;
		sys_write32(tmp, cfg->base + USB_DEV_CXCFE_OFFSET);
	} else {
		if (USB_EP_DIR_IS_OUT(config->addr)) {
			tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET +
					 (ep_idx - 1) *
						 USB_BL808X_MPS_REG_STRIDE);
			tmp &= ~USB_STL_OEP1;
			sys_write32(tmp,
				    cfg->base + USB_DEV_OUTMPS1_OFFSET +
					    (ep_idx - 1) *
						    USB_BL808X_MPS_REG_STRIDE);
		} else {
			tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET +
					 (ep_idx - 1) *
						 USB_BL808X_MPS_REG_STRIDE);
			tmp &= ~USB_STL_IEP1;
			sys_write32(tmp,
				    cfg->base + USB_DEV_INMPS1_OFFSET +
					    (ep_idx - 1) *
						    USB_BL808X_MPS_REG_STRIDE);
		}
		udc_bflb_bl808x_ev_submit(dev, config->addr, UDC_BL808X_EVT_XFER,
					  K_NO_WAIT);
	}

	config->stat.halted = false;

	return 0;
}

static int udc_bflb_bl808x_host_wakeup(const struct device *const dev)
{
	LOG_DBG("Remote wakeup from %p", dev);

	return -ENOTSUP;
}

static int udc_bflb_bl808x_enable(const struct device *const dev)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	struct udc_bflb_bl808x_data *const priv = udc_get_private(dev);
	uint32_t tmp;

	LOG_DBG("Enable device %p", dev);

	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp |= PDS_REG_USB_IDDIG_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	/* Disable global interrupt while configuring */
	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp &= ~USB_GLINT_EN_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	/* Force D+/D- disconnect (unplug signal) during configuration */
	tmp = sys_read32(cfg->base + USB_PHY_TST_OFFSET);
	tmp |= USB_UNPLUG;
	sys_write32(tmp, cfg->base + USB_PHY_TST_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp &= ~USB_CAP_RMWAKUP;
	tmp |= USB_CHIP_EN_HOV;
	if (cfg->speed_idx < UDC_BUS_SPEED_HS) {
		tmp |= USB_FORCE_FS;
	} else {
		tmp &= ~USB_FORCE_FS;
	}
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp |= USB_SFRST_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	/* wait for soft reset */
	while ((sys_read32(cfg->base + USB_DEV_CTL_OFFSET) & USB_SFRST_HOV) !=
	       0) {
	}

	tmp = sys_read32(cfg->base + USB_DEV_ADR_OFFSET);
	tmp &= ~USB_AFT_CONF;
	sys_write32(tmp, cfg->base + USB_DEV_ADR_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_SMT_OFFSET);
	tmp &= ~USB_SOFMT_MASK;
	if (cfg->speed_idx == UDC_BUS_SPEED_HS) {
		tmp |= USB_BL808X_SOF_TIMER_HS;
	} else {
		tmp |= USB_BL808X_SOF_TIMER_FS;
	}
	sys_write32(tmp, cfg->base + USB_DEV_SMT_OFFSET);

	/*
	 * Clear all pending interrupts in each group.
	 * ISGx registers are write-1-to-clear, MISGx registers are masks.
	 */
	sys_write32(0xFFFFFFFFU,
		    cfg->base + USB_DEV_ISG0_OFFSET); /* G0: control */
	sys_write32(0xFFFFFFFFU, cfg->base + USB_DEV_ISG1_OFFSET); /* G1: FIFO */
	sys_write32(0x3FFU, cfg->base + USB_DEV_ISG2_OFFSET); /* G2: device */
	sys_write32(0xFFFFFFFFU, cfg->base + USB_DEV_ISG3_OFFSET); /* G3: DMA */

	/* G0: unmask CX_SETUP only (control transfer setup token) */
	tmp = sys_read32(cfg->base + USB_DEV_MISG0_OFFSET);
	tmp &= ~USB_MCX_SETUP_INT;
	tmp |= USB_MCX_COMFAIL_INT | USB_MCX_COMABORT_INT | USB_MCX_COMEND_INT |
	       USB_MCX_IN_INT | USB_MCX_OUT_INT;
	sys_write32(tmp, cfg->base + USB_DEV_MISG0_OFFSET);

	/* G1: mask all FIFO interrupts (DMA completion used instead) */
	sys_write32(0xFFFFFFFFU, cfg->base + USB_DEV_MISG1_OFFSET);

	/* G2: unmask bits [4:0] = USBRST, SUSP, RESM, ISO_ERR, ISO_ABORT */
	sys_write32(0xFFFFFFE0U, cfg->base + USB_DEV_MISG2_OFFSET);

	/* G3: unmask bits [4:0] = VDMA completion for CXF + FIFO 0-3 */
	sys_write32(0xFFFFFFE0U, cfg->base + USB_DEV_MISG3_OFFSET);

	/* Enable all interrupt groups at top level */
	tmp = sys_read32(cfg->base + USB_DEV_MIGR_OFFSET);
	tmp &= ~(USB_MINT_G0 | USB_MINT_G1 | USB_MINT_G2 | USB_MINT_G3 |
		 USB_MINT_G4);
	sys_write32(tmp, cfg->base + USB_DEV_MIGR_OFFSET);

	tmp = sys_read32(cfg->base + USB_GLB_INT_OFFSET);
	tmp |= USB_MHC_INT;
	tmp |= USB_MOTG_INT;
	tmp &= ~USB_MDEV_INT;
	sys_write32(tmp, cfg->base + USB_GLB_INT_OFFSET);

	/* Disconnect all EPs from FIFOs (0xF = no EP assigned) */
	sys_write32(0xFFFFFFFFU, cfg->base + USB_DEV_EPMAP0_OFFSET);
	sys_write32(0xFFU, cfg->base + USB_DEV_EPMAP1_OFFSET);
	udc_bflb_bl808x_fmap_set(dev, USB_BL808X_FIFO_EP_NONE, 1,
				 USB_BL808X_FIFO_DIR_OUT);
	udc_bflb_bl808x_fmap_set(dev, USB_BL808X_FIFO_EP_NONE, 2,
				 USB_BL808X_FIFO_DIR_OUT);
	udc_bflb_bl808x_fmap_set(dev, USB_BL808X_FIFO_EP_NONE, 3,
				 USB_BL808X_FIFO_DIR_OUT);
	udc_bflb_bl808x_fmap_set(dev, USB_BL808X_FIFO_EP_NONE, 4,
				 USB_BL808X_FIFO_DIR_OUT);

	udc_bflb_bl808x_fifo_reset(dev, 1);
	udc_bflb_bl808x_fifo_reset(dev, 2);
	udc_bflb_bl808x_fifo_reset(dev, 3);
	udc_bflb_bl808x_fifo_reset(dev, 4);

	/* Enable VDMA (virtual DMA) for all FIFO transfers */
	tmp = sys_read32(cfg->base + USB_VDMA_CTRL_OFFSET);
	tmp |= USB_VDMA_EN;
	sys_write32(tmp, cfg->base + USB_VDMA_CTRL_OFFSET);

	/* Re-connect D+/D- (remove unplug signal) */
	tmp = sys_read32(cfg->base + USB_PHY_TST_OFFSET);
	tmp &= ~USB_UNPLUG;
	sys_write32(tmp, cfg->base + USB_PHY_TST_OFFSET);

	/* Enable global interrupt — controller is now ready */
	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp |= USB_GLINT_EN_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	priv->reset_expiration =
		sys_timepoint_calc(USB_BL808X_RESET_SETTLE_TIME);

	return 0;
}

static int udc_bflb_bl808x_disable(const struct device *const dev)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp &= ~USB_GLINT_EN_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	tmp = sys_read32(cfg->base + USB_PHY_TST_OFFSET);
	tmp |= USB_UNPLUG;
	sys_write32(tmp, cfg->base + USB_PHY_TST_OFFSET);

	return 0;
}

/*
 * Clock and PHY initialization
 *
 */

/*
 * Enable the USB PLL (48 MHz from WiFi PLL CFG10).
 * Power on the MMDIV, then toggle RSTB (1→0→1) to reset the divider.
 * 5 µs delays per SDK GLB_Set_USB_CLK_From_WIFIPLL().
 */
static void udc_bflb_bl808x_clock_init(const struct device *const dev)
{
	uint32_t tmp;

	/* Power on USB PLL multi-modulus divider */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
	tmp |= GLB_PU_USBPLL_MMDIV_MSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);

	k_usleep(5);

	/* RSTB toggle (1→0→1): reset the USB PLL divider */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
	tmp |= GLB_USBPLL_RSTB_MSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);

	k_usleep(5);

	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
	tmp &= ~GLB_USBPLL_RSTB_MSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);

	k_usleep(5);

	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
	tmp |= GLB_USBPLL_RSTB_MSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
}

/*
 * Initialize the USB 2.0 PHY via PDS registers.
 * Sequence matches SDK USB_Set_Device_Mode_Phy_Addr().
 * Delays per SDK: 1 µs for register settling, 5 ms for PHY stabilization.
 */
static void udc_bflb_bl808x_phy_init(const struct device *const dev)
{
	uint32_t tmp;

	/* Select internal crystal for PHY reference clock */
	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp &= ~PDS_REG_USB_PHY_XTLSEL_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	/* Power on USB 2.0 power switch */
	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp |= PDS_REG_PU_USB20_PSW_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	/* Assert PHY power-on reset */
	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp |= PDS_REG_USB_PHY_PONRST_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	k_usleep(1);

	/* Assert controller software reset */
	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp &= ~PDS_REG_USB_SW_RST_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	k_usleep(1);

	/* De-assert PHY suspend (activate PHY) */
	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp |= PDS_REG_USB_EXT_SUSP_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	k_msleep(5);

	/* De-assert controller software reset */
	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp |= PDS_REG_USB_SW_RST_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	k_msleep(5);
}

/*
 * UDC lifecycle — init, shutdown, preinit
 *
 */

static int udc_bflb_bl808x_init(const struct device *const dev)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	int ret;

	udc_bflb_bl808x_clock_init(dev);
	udc_bflb_bl808x_phy_init(dev);

	/* Disconnect all EPs from FIFOs: 4 data FIFOs + 1 control FIFO,
	 * mapped via EPMAP0 (EP1-4) and EPMAP1 (EP5-8). 0xF = no EP.
	 */
	sys_write32(0xFFFFFFFFU, cfg->base + USB_DEV_EPMAP0_OFFSET);
	sys_write32(0xFFU, cfg->base + USB_DEV_EPMAP1_OFFSET);
	udc_bflb_bl808x_fmap_set(dev, USB_BL808X_FIFO_EP_NONE, 1,
				 USB_BL808X_FIFO_DIR_OUT);
	udc_bflb_bl808x_fmap_set(dev, USB_BL808X_FIFO_EP_NONE, 2,
				 USB_BL808X_FIFO_DIR_OUT);
	udc_bflb_bl808x_fmap_set(dev, USB_BL808X_FIFO_EP_NONE, 3,
				 USB_BL808X_FIFO_DIR_OUT);
	udc_bflb_bl808x_fmap_set(dev, USB_BL808X_FIFO_EP_NONE, 4,
				 USB_BL808X_FIFO_DIR_OUT);

	udc_bflb_bl808x_cx_fifo_reset(dev);
	udc_bflb_bl808x_fifo_reset(dev, 1);
	udc_bflb_bl808x_fifo_reset(dev, 2);
	udc_bflb_bl808x_fifo_reset(dev, 3);
	udc_bflb_bl808x_fifo_reset(dev, 4);

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
				     USB_EP_TYPE_CONTROL, 64, 0);
	if (ret < 0) {
		LOG_ERR("Failed to enable control endpoint");
		return ret;
	}

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL,
				     64, 0);
	if (ret < 0) {
		LOG_ERR("Failed to enable control endpoint");
		return ret;
	}

	cfg->irq_enable_func(dev);

	LOG_INF("Initialized");

	return 0;
}

/* Shut down the controller completely */
static int udc_bflb_bl808x_shutdown(const struct device *const dev)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	uint32_t tmp;

	cfg->irq_disable_func(dev);

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp &= ~PDS_REG_USB_PHY_XTLSEL_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp &= ~PDS_REG_PU_USB20_PSW_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp &= ~PDS_REG_USB_PHY_PONRST_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp &= ~PDS_REG_USB_EXT_SUSP_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	return 0;
}

static int udc_bflb_bl808x_driver_preinit(const struct device *const dev)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	struct udc_data *const data = dev->data;
	uint16_t mps = 512;
	int err;

	k_mutex_init(&data->mutex);

	data->caps.rwup = true;
	data->caps.mps0 = UDC_MPS0_64;
	if (cfg->speed_idx == UDC_BUS_SPEED_HS) {
		data->caps.hs = true;
		mps = 1024;
	}

	for (int i = 0; i < USB_BL808X_NUM_BIDIR_EPS; i++) {
		cfg->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			cfg->ep_cfg_out[i].caps.control = 1;
			cfg->ep_cfg_out[i].caps.mps = 64;
		} else {
			cfg->ep_cfg_out[i].caps.bulk = 1;
			cfg->ep_cfg_out[i].caps.interrupt = 1;
			cfg->ep_cfg_out[i].caps.iso = 1;
			cfg->ep_cfg_out[i].caps.mps = mps;
		}

		cfg->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &cfg->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (int i = 0; i < USB_BL808X_NUM_BIDIR_EPS; i++) {
		cfg->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			cfg->ep_cfg_in[i].caps.control = 1;
			cfg->ep_cfg_in[i].caps.mps = 64;
		} else {
			cfg->ep_cfg_in[i].caps.bulk = 1;
			cfg->ep_cfg_in[i].caps.interrupt = 1;
			cfg->ep_cfg_in[i].caps.iso = 1;
			cfg->ep_cfg_in[i].caps.mps = mps;
		}

		cfg->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &cfg->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	LOG_INF("Device %p (max. speed %d)", dev, cfg->speed_idx);

	return 0;
}

/*
 * Interrupt service routine
 *
 */

static void udc_bflb_bl808x_isr(const struct device *const dev)
{
	const struct udc_bflb_bl808x_config *const cfg = dev->config;
	struct udc_bflb_bl808x_data *const priv = udc_get_private(dev);
	uint32_t glb_intstatus;
	uint32_t dev_intstatus;
	uint32_t group_intstatus;
	uint32_t tmp;

	glb_intstatus = sys_read32(cfg->base + USB_GLB_ISR_OFFSET);

	if (glb_intstatus & USB_DEV_INT) {
		dev_intstatus = sys_read32(cfg->base + USB_DEV_IGR_OFFSET);
		if (dev_intstatus & USB_INT_G0) {
			group_intstatus =
				sys_read32(cfg->base + USB_DEV_ISG0_OFFSET);
			group_intstatus &=
				~sys_read32(cfg->base + USB_DEV_MISG0_OFFSET);

			if (group_intstatus & USB_CX_COMABT_INT) {
				udc_submit_event(dev, UDC_EVT_ERROR, -ECANCELED);
				LOG_ERR("Control command abort");
			}

			if (group_intstatus & USB_CX_SETUP_INT) {
				/* Mask setup interrupt until processing
				 * completes. Without this, the host's setup
				 * retries flood the ISR and starve the work
				 * queue that processes them.
				 */
				tmp = sys_read32(cfg->base +
						 USB_DEV_MISG0_OFFSET);
				tmp |= USB_MCX_SETUP_INT;
				sys_write32(tmp,
					    cfg->base + USB_DEV_MISG0_OFFSET);

				/* Setup packet implies bus is active */
				if (udc_is_suspended(dev)) {
					udc_set_suspended(dev, false);
				}

				priv->ep_is_in[0] = false;
				priv->setup_received = true;
				udc_bflb_bl808x_ctrl_setup_start(dev);
			}

			if (group_intstatus & USB_CX_COMFAIL_INT) {
				udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
				LOG_ERR("Control command fail");
			}

			/* Clear G0 (control) interrupt status */
			sys_write32(group_intstatus,
				    cfg->base + USB_DEV_ISG0_OFFSET);
		}
		if (dev_intstatus & USB_INT_G1) {
			group_intstatus =
				sys_read32(cfg->base + USB_DEV_ISG1_OFFSET);
			group_intstatus &=
				~sys_read32(cfg->base + USB_DEV_MISG1_OFFSET);

			sys_write32(group_intstatus,
				    cfg->base + USB_DEV_ISG1_OFFSET);
		}
		if (dev_intstatus & USB_INT_G2) {
			group_intstatus =
				sys_read32(cfg->base + USB_DEV_ISG2_OFFSET);
			group_intstatus &=
				~sys_read32(cfg->base + USB_DEV_MISG2_OFFSET);

			/* G2: Suspend */
			if (group_intstatus & USB_SUSP_INT) {
				sys_write32(USB_SUSP_INT,
					    cfg->base + USB_DEV_ISG2_OFFSET);
				LOG_DBG("USB suspended");

				udc_set_suspended(dev, true);
				udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
			}

			/* resumed — only act on genuine resume after suspend */
			if ((group_intstatus & USB_RESM_INT) &&
			    udc_is_suspended(dev)) {
				sys_write32(USB_RESM_INT,
					    cfg->base + USB_DEV_ISG2_OFFSET);
				LOG_DBG("USB resumed");

				udc_set_suspended(dev, false);
				udc_submit_event(dev, UDC_EVT_RESUME, 0);
			} else if (group_intstatus & USB_RESM_INT) {
				/* Clear the spurious RESM status bit */
				sys_write32(USB_RESM_INT,
					    cfg->base + USB_DEV_ISG2_OFFSET);
			}

			if (group_intstatus & USBRST_INT) {
				sys_write32(USBRST_INT,
					    cfg->base + USB_DEV_ISG2_OFFSET);
				LOG_DBG("USB bus reset");

				/* Clear AFT_CONF - device returns to default
				 * state */
				tmp = sys_read32(cfg->base + USB_DEV_ADR_OFFSET);
				tmp &= ~USB_AFT_CONF;
				sys_write32(tmp, cfg->base + USB_DEV_ADR_OFFSET);

				udc_bflb_bl808x_cx_fifo_reset(dev);
				udc_bflb_bl808x_fifo_reset(dev, 1);
				udc_bflb_bl808x_fifo_reset(dev, 2);
				udc_bflb_bl808x_fifo_reset(dev, 3);
				udc_bflb_bl808x_fifo_reset(dev, 4);

				/* Clear any spurious VDMA completions from FIFO
				 * reset */
				sys_write32(0x1FU,
					    cfg->base + USB_DEV_ISG3_OFFSET);

				/* Clear stale setup state and drain any buffers
				 * left from an interrupted control transfer.
				 */
				priv->setup_received = false;
				{
					struct net_buf *stale;

					while ((stale = udc_buf_get(udc_get_ep_cfg(
							dev,
							USB_CONTROL_EP_OUT))) !=
					       NULL) {
						net_buf_unref(stale);
					}
					while ((stale = udc_buf_get(udc_get_ep_cfg(
							dev,
							USB_CONTROL_EP_IN))) !=
					       NULL) {
						net_buf_unref(stale);
					}
				}

				/* Bus reset implies bus is active */
				if (udc_is_suspended(dev)) {
					udc_set_suspended(dev, false);
				}

				tmp = sys_read32(cfg->base + USB_DEV_SMT_OFFSET);
				tmp &= ~USB_SOFMT_MASK;
				if (cfg->speed_idx == UDC_BUS_SPEED_HS) {
					tmp |= USB_BL808X_SOF_TIMER_HS;
				} else {
					tmp |= USB_BL808X_SOF_TIMER_FS;
				}
				sys_write32(tmp, cfg->base + USB_DEV_SMT_OFFSET);

				/* Re-enable setup interrupt (may have been
				 * masked) */
				tmp = sys_read32(cfg->base +
						 USB_DEV_MISG0_OFFSET);
				tmp &= ~USB_MCX_SETUP_INT;
				sys_write32(tmp,
					    cfg->base + USB_DEV_MISG0_OFFSET);

				priv->reset_expiration = sys_timepoint_calc(
					USB_BL808X_RESET_SETTLE_TIME);

				udc_submit_event(dev, UDC_EVT_RESET, 0);
			}

			if (group_intstatus & USB_ISO_SEQ_ERR_INT) {
				sys_write32(USB_ISO_SEQ_ERR_INT,
					    cfg->base + USB_DEV_ISG2_OFFSET);

				udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
				LOG_ERR("Isochronous sequence error");
			}

			if (group_intstatus & USB_ISO_SEQ_ABORT_INT) {
				sys_write32(USB_ISO_SEQ_ABORT_INT,
					    cfg->base + USB_DEV_ISG2_OFFSET);

				udc_submit_event(dev, UDC_EVT_ERROR, -ECANCELED);
				LOG_ERR("Isochronous sequence aborted");
			}
		}
		if (dev_intstatus & USB_INT_G3) {
			group_intstatus =
				sys_read32(cfg->base + USB_DEV_ISG3_OFFSET);
			group_intstatus &=
				~sys_read32(cfg->base + USB_DEV_MISG3_OFFSET);
			sys_write32(group_intstatus,
				    cfg->base + USB_DEV_ISG3_OFFSET);

			if (group_intstatus & USB_VDMA_CMPLT_CXF) {
				if (priv->ep_is_in[0]) {
					udc_bflb_bl808x_ev_submit(
						dev, USB_CONTROL_EP_IN,
						UDC_BL808X_EVT_CTRL_END,
						K_NO_WAIT);
					udc_bflb_bl808x_cx_done(dev);
				} else {
					udc_bflb_bl808x_ev_submit(
						dev, USB_CONTROL_EP_OUT,
						UDC_BL808X_EVT_CTRL_END,
						K_NO_WAIT);
				}
			}

			for (uint8_t i = 1; i < USB_BL808X_NUM_BIDIR_EPS; i++) {
				if (group_intstatus & (1U << i)) {
					if (priv->ep_is_in
						    [udc_bflb_bl808x_fifo_to_ep(
							    dev, i)]) {
						udc_bflb_bl808x_ev_submit(
							dev,
							USB_EP_DIR_IN |
								udc_bflb_bl808x_fifo_to_ep(
									dev, i),
							UDC_BL808X_EVT_END,
							K_NO_WAIT);
					} else {
						udc_bflb_bl808x_ev_submit(
							dev,
							USB_EP_DIR_OUT |
								udc_bflb_bl808x_fifo_to_ep(
									dev, i),
							UDC_BL808X_EVT_END,
							K_NO_WAIT);
					}
				}
			}
		}
		if (dev_intstatus & USB_INT_G4) {
			/* Nothing we care about in group 4 */
		}
	}
}

/*
 * API struct and device instantiation
 *
 */

static void udc_bflb_bl808x_lock(const struct device *const dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void udc_bflb_bl808x_unlock(const struct device *const dev)
{
	udc_unlock_internal(dev);
}

static const struct udc_api udc_bflb_bl808x_api = {
	.lock = udc_bflb_bl808x_lock,
	.unlock = udc_bflb_bl808x_unlock,
	.device_speed = udc_bflb_bl808x_device_speed,
	.init = udc_bflb_bl808x_init,
	.enable = udc_bflb_bl808x_enable,
	.disable = udc_bflb_bl808x_disable,
	.shutdown = udc_bflb_bl808x_shutdown,
	.set_address = udc_bflb_bl808x_set_address,
	.host_wakeup = udc_bflb_bl808x_host_wakeup,
	.ep_enable = udc_bflb_bl808x_ep_enable,
	.ep_disable = udc_bflb_bl808x_ep_disable,
	.ep_set_halt = udc_bflb_bl808x_ep_set_halt,
	.ep_clear_halt = udc_bflb_bl808x_ep_clear_halt,
	.ep_enqueue = udc_bflb_bl808x_ep_enqueue,
	.ep_dequeue = udc_bflb_bl808x_ep_dequeue,
};

#define UDC_BFLB_BL808X_DEVICE_DEFINE(n)                                        \
	static void udc_irq_enable_func##n(const struct device *const dev)      \
	{                                                                       \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),          \
			    udc_bflb_bl808x_isr, DEVICE_DT_INST_GET(n), 0);     \
                                                                                \
		irq_enable(DT_INST_IRQN(n));                                    \
	}                                                                       \
                                                                                \
	static void udc_irq_disable_func##n(const struct device *const dev)     \
	{                                                                       \
		irq_disable(DT_INST_IRQN(n));                                   \
	}                                                                       \
                                                                                \
	static struct udc_ep_config ep_cfg_out[USB_BL808X_NUM_BIDIR_EPS];       \
	static struct udc_ep_config ep_cfg_in[USB_BL808X_NUM_BIDIR_EPS];        \
                                                                                \
	static const struct udc_bflb_bl808x_config udc_bflb_bl808x_config_##n = \
		{                                                               \
			.base = DT_INST_REG_ADDR(n),                            \
			.ep_cfg_in = ep_cfg_in,                                 \
			.ep_cfg_out = ep_cfg_out,                               \
			.speed_idx =                                            \
				DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),     \
			.irq_enable_func = udc_irq_enable_func##n,              \
			.irq_disable_func = udc_irq_disable_func##n,            \
	};                                                                      \
                                                                                \
	static struct udc_bflb_bl808x_data udc_priv_##n = {                     \
		.setup_received = false,                                        \
		.wa_reset_packet_count = USB_BL808X_WA_RESET_PACKETS,           \
	};                                                                      \
                                                                                \
	static struct udc_data udc_data_##n = {                                 \
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),               \
		.priv = &udc_priv_##n,                                          \
	};                                                                      \
                                                                                \
	DEVICE_DT_INST_DEFINE(n, udc_bflb_bl808x_driver_preinit, NULL,          \
			      &udc_data_##n, &udc_bflb_bl808x_config_##n,       \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,  \
			      &udc_bflb_bl808x_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_BFLB_BL808X_DEVICE_DEFINE)
