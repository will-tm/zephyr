/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_ieee802154

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ieee802154_bflb, CONFIG_IEEE802154_DRIVER_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/random/random.h>
#include <zephyr/irq.h>

#include "ieee802154_bflb.h"
#include "lmac154.h"
#include <bflb_mac154.h>

#if defined(CONFIG_NET_L2_OPENTHREAD)
#include <zephyr/net/openthread.h>
#endif

/* RF PHY functions from libbl702l_rf.a */
extern bool bz_phy_optimize_tx_channel(uint32_t channel_mhz);
extern bool bz_phy_set_tx_power(int power_dbm);
extern void rf_tx_pwr_init(void);

/* TX/ACK timeout constants */
#define BFLB_IEEE802154_TX_TIMEOUT  K_MSEC(100)
#define BFLB_IEEE802154_ACK_TIMEOUT K_MSEC(50)

/* TX status sentinel for "not yet completed" */
#define BFLB_IEEE802154_TX_STATUS_PENDING 0xFFU

/* Hardware TX power range (dBm) */
#define BFLB_IEEE802154_TX_POWER_MIN 0
#define BFLB_IEEE802154_TX_POWER_MAX 14

/* Default channel and TX power */
#define BFLB_IEEE802154_DEFAULT_CHANNEL  11U
#define BFLB_IEEE802154_DEFAULT_TX_POWER 10

/* IRQ numbers from device tree */
#define M154_IRQ              DT_INST_IRQ_BY_IDX(0, 0, irq)
#define M154_IRQ_PRIO         DT_INST_IRQ_BY_IDX(0, 0, priority)
#define M154_ENH_ACK_IRQ      DT_INST_IRQ_BY_IDX(0, 1, irq)
#define M154_ENH_ACK_IRQ_PRIO DT_INST_IRQ_BY_IDX(0, 1, priority)

static struct bflb_ieee802154_data bflb_ieee802154_data_inst;

/* lmac154 ISR wrappers */

static lmac154_isr_t m154_isr_handler;
static lmac154_isr_t m154_enh_ack_isr_handler;

static void m154_irq_handler(const void *arg)
{
	ARG_UNUSED(arg);

	if (m154_isr_handler != NULL) {
		m154_isr_handler();
	}
}

static void m154_enh_ack_irq_handler(const void *arg)
{
	ARG_UNUSED(arg);

	if (m154_enh_ack_isr_handler != NULL) {
		m154_enh_ack_isr_handler();
	}
}

void lmac154_txDoneEvent(lmac154_tx_status_t tx_status)
{
	bflb_ieee802154_data_inst.tx_status = (uint8_t)tx_status;
	k_sem_give(&bflb_ieee802154_data_inst.tx_sem);
}

void lmac154_ackEvent(uint8_t ack_received, uint8_t frame_pending, uint8_t seq_num)
{
	bflb_ieee802154_data_inst.ack_received = ack_received;
	bflb_ieee802154_data_inst.ack_frame_pending = frame_pending;
	bflb_ieee802154_data_inst.ack_seq_num = seq_num;
	k_sem_give(&bflb_ieee802154_data_inst.ack_sem);
}

void lmac154_ackFrameEvent(uint8_t ack_received, uint8_t *rx_buf, uint8_t len)
{
	if ((ack_received != 0U) && (rx_buf != NULL) && (len > 0U)) {
		uint8_t copy_len = MIN(len, sizeof(bflb_ieee802154_data_inst.ack_frame_buf));

		(void)memcpy(bflb_ieee802154_data_inst.ack_frame_buf, rx_buf, copy_len);
		bflb_ieee802154_data_inst.ack_frame_len = copy_len;
	}
}

void lmac154_rxDoneEvent(uint8_t *rx_buf, uint8_t rx_len, uint8_t crc_fail)
{
	struct net_pkt *pkt;
	uint8_t pkt_len;

	if (bflb_ieee802154_data_inst.iface == NULL) {
		return;
	}

	if (crc_fail != 0U) {
		LOG_DBG("RX CRC fail, dropping");
		return;
	}

	/* rx_len includes 2 CRC bytes but rx_buf does NOT contain them.
	 * Actual MPDU data length = rx_len - FCS_LEN
	 */
	if (rx_len <= BFLB_IEEE802154_FCS_LEN) {
		return;
	}

	pkt_len = rx_len - BFLB_IEEE802154_FCS_LEN;

	pkt = net_pkt_rx_alloc_with_buffer(bflb_ieee802154_data_inst.iface, rx_len, AF_UNSPEC, 0,
					   K_NO_WAIT);
	if (pkt == NULL) {
		LOG_ERR("RX: no pkt buf");
		return;
	}

	/* Copy MPDU (without CRC) */
	if (net_pkt_write(pkt, rx_buf, pkt_len) != 0) {
		LOG_ERR("RX: write fail");
		net_pkt_unref(pkt);
		return;
	}

	/* For OpenThread (PKT_INCL_FCS), append the 2-byte FCS from hardware */
	if (IS_ENABLED(CONFIG_IEEE802154_L2_PKT_INCL_FCS)) {
		uint8_t fcs[BFLB_IEEE802154_FCS_LEN];

		lmac154_readRxCrc(fcs);
		if (net_pkt_write(pkt, fcs, sizeof(fcs)) != 0) {
			LOG_ERR("RX: FCS write fail");
			net_pkt_unref(pkt);
			return;
		}
	}

	/* Set RSSI and LQI in the packet control block */
	net_pkt_set_ieee802154_rssi_dbm(pkt, lmac154_getRSSI());
	net_pkt_set_ieee802154_lqi(pkt, (uint8_t)lmac154_getLQI());

	if (net_recv_data(bflb_ieee802154_data_inst.iface, pkt) < 0) {
		LOG_ERR("RX: net_recv_data fail");
		net_pkt_unref(pkt);
	}
}

void lmac154_rxStartEvent(void)
{
	/* No-op: required by lmac154 callback ABI */
}

void lmac154_hwAutoTxAckDoneEvent(void)
{
	/* No-op: required by lmac154 callback ABI */
}

void lmac154_reqEnhAckEvent(void)
{
	/* No-op for now, needed for Thread 1.2 enhanced ACK */
}

void lmac154_rxMhrEvent(uint8_t *rx_buf, uint8_t rx_len, uint8_t pkt_len)
{
	ARG_UNUSED(rx_buf);
	ARG_UNUSED(rx_len);
	ARG_UNUSED(pkt_len);
}

void lmac154_rxSecMhrEvent(uint8_t *rx_buf, uint8_t rx_len, uint8_t pkt_len)
{
	ARG_UNUSED(rx_buf);
	ARG_UNUSED(rx_len);
	ARG_UNUSED(pkt_len);
}

/* Zephyr IEEE 802.15.4 radio API */

static enum ieee802154_hw_caps bflb_ieee802154_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return IEEE802154_HW_FCS | IEEE802154_HW_FILTER | IEEE802154_HW_PROMISC |
	       IEEE802154_HW_CSMA | IEEE802154_HW_TX_RX_ACK | IEEE802154_HW_RX_TX_ACK |
	       IEEE802154_HW_ENERGY_SCAN;
}

static int bflb_ieee802154_cca(const struct device *dev)
{
	int rssi;
	uint8_t busy;

	ARG_UNUSED(dev);

	busy = lmac154_runCCA(&rssi);

	return (busy != 0U) ? -EBUSY : 0;
}

static int bflb_ieee802154_set_channel(const struct device *dev, uint16_t channel)
{
	ARG_UNUSED(dev);

	if ((channel < BFLB_IEEE802154_CHANNEL_MIN) || (channel > BFLB_IEEE802154_CHANNEL_MAX)) {
		return -EINVAL;
	}

	/* Optimize RF front-end for the target channel (vendor SDK requirement).
	 * Must be called on every channel change per ot_radio_bl70xx.c pattern.
	 */
	bz_phy_optimize_tx_channel(2405U + (5U * (channel - BFLB_IEEE802154_CHANNEL_MIN)));

	/* lmac154 channel enum: 0 = channel 11, 1 = channel 12, etc. */
	lmac154_setChannel((lmac154_channel_t)(channel - BFLB_IEEE802154_CHANNEL_MIN));
	bflb_ieee802154_data_inst.channel = channel;

	return 0;
}

static int bflb_ieee802154_filter(const struct device *dev, bool set,
				  enum ieee802154_filter_type type,
				  const struct ieee802154_filter *filter)
{
	ARG_UNUSED(dev);

	if (!set) {
		return -ENOTSUP;
	}

	switch (type) {
	case IEEE802154_FILTER_TYPE_IEEE_ADDR:
		/* lmac154 API takes non-const uint8_t*; cast required */
		lmac154_setLongAddr((uint8_t *)filter->ieee_addr);
		break;
	case IEEE802154_FILTER_TYPE_SHORT_ADDR:
		lmac154_setShortAddr(filter->short_addr);
		break;
	case IEEE802154_FILTER_TYPE_PAN_ID:
		lmac154_setPanId(filter->pan_id);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int bflb_ieee802154_set_txpower(const struct device *dev, int16_t dbm)
{
	ARG_UNUSED(dev);

	/* Clamp to hardware range */
	if (dbm < BFLB_IEEE802154_TX_POWER_MIN) {
		dbm = BFLB_IEEE802154_TX_POWER_MIN;
	} else if (dbm > BFLB_IEEE802154_TX_POWER_MAX) {
		dbm = BFLB_IEEE802154_TX_POWER_MAX;
	}

	lmac154_setTxPower((lmac154_tx_power_t)dbm);
	bz_phy_set_tx_power((int)dbm);
	bflb_ieee802154_data_inst.tx_power = dbm;

	return 0;
}

static int bflb_ieee802154_tx(const struct device *dev, enum ieee802154_tx_mode mode,
			      struct net_pkt *pkt, struct net_buf *frag)
{
	uint8_t *data = frag->data;
	uint8_t len = frag->len;
	bool csma = false;
	int ret;

	ARG_UNUSED(dev);
	ARG_UNUSED(pkt);

	if (len > BFLB_IEEE802154_MAX_PKT_LEN) {
		return -EINVAL;
	}

	switch (mode) {
	case IEEE802154_TX_MODE_DIRECT:
		csma = false;
		break;
	case IEEE802154_TX_MODE_CSMA_CA:
	case IEEE802154_TX_MODE_CCA:
		csma = true;
		break;
	default:
		return -ENOTSUP;
	}

	/* Reset semaphores */
	k_sem_reset(&bflb_ieee802154_data_inst.tx_sem);
	k_sem_reset(&bflb_ieee802154_data_inst.ack_sem);

	bflb_ieee802154_data_inst.tx_status = BFLB_IEEE802154_TX_STATUS_PENDING;
	bflb_ieee802154_data_inst.ack_received = 0U;

	lmac154_triggerTx(data, len, csma ? 1U : 0U);

	/* Wait for TX done */
	ret = k_sem_take(&bflb_ieee802154_data_inst.tx_sem, BFLB_IEEE802154_TX_TIMEOUT);
	if (ret != 0) {
		LOG_ERR("TX timeout");
		return -EIO;
	}

	if (bflb_ieee802154_data_inst.tx_status != (uint8_t)LMAC154_TX_STATUS_TX_FINISHED) {
		LOG_DBG("TX failed: status=%d", bflb_ieee802154_data_inst.tx_status);
		if (bflb_ieee802154_data_inst.tx_status == (uint8_t)LMAC154_TX_STATUS_CSMA_FAILED) {
			return -EBUSY;
		}
		return -EIO;
	}

	/* If ACK was requested, wait for ACK event */
	if (LMAC154_FRAME_IS_ACK_REQ(data[0])) {
		ret = k_sem_take(&bflb_ieee802154_data_inst.ack_sem, BFLB_IEEE802154_ACK_TIMEOUT);
		if (ret != 0) {
			LOG_DBG("ACK timeout");
			return -ENOMSG;
		}
		if (bflb_ieee802154_data_inst.ack_received == 0U) {
			LOG_DBG("No ACK received");
			return -ENOMSG;
		}
	}

	return 0;
}

static int bflb_ieee802154_start(const struct device *dev)
{
	ARG_UNUSED(dev);

	lmac154_enableRx();
	lmac154_setRxStateWhenIdle(true);
	bflb_ieee802154_data_inst.started = true;

	LOG_DBG("802.15.4 radio started");
	return 0;
}

static int bflb_ieee802154_stop(const struct device *dev)
{
	ARG_UNUSED(dev);

	lmac154_disableRx();
	/* Keep RxStateWhenIdle=true so that after TX completes,
	 * the radio returns to RX and can receive the response
	 * (e.g., beacon response, MLE Parent Response, ACK).
	 * Without this, the radio goes idle after TX and misses responses.
	 */
	bflb_ieee802154_data_inst.started = false;

	LOG_DBG("802.15.4 radio stopped");
	return 0;
}

static int bflb_ieee802154_configure(const struct device *dev, enum ieee802154_config_type type,
				     const struct ieee802154_config *config)
{
	ARG_UNUSED(dev);

	switch (type) {
	case IEEE802154_CONFIG_PROMISCUOUS:
		if (config->promiscuous) {
			lmac154_enableRxPromiscuousMode(0, 0);
			bflb_ieee802154_data_inst.promiscuous = true;
		} else {
			lmac154_disableRxPromiscuousMode();
			bflb_ieee802154_data_inst.promiscuous = false;
		}
		break;
	case IEEE802154_CONFIG_EVENT_HANDLER:
		/* Not supported yet */
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static energy_scan_done_cb_t ed_scan_done_cb;

static void ed_scan_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (ed_scan_done_cb != NULL) {
		int rssi;

		lmac154_runCCA(&rssi);
		ed_scan_done_cb(DEVICE_DT_INST_GET(0), (int16_t)rssi);
	}
}

static K_WORK_DELAYABLE_DEFINE(ed_scan_work, ed_scan_work_handler);

static int bflb_ieee802154_ed_scan(const struct device *dev, uint16_t duration,
				   energy_scan_done_cb_t done_cb)
{
	ARG_UNUSED(dev);

	/* Async callback via delayed work queue. The delay provides dwell time
	 * for the radio to receive beacon responses during active scan before
	 * moving to the next channel. Running CCA at callback time (after the
	 * dwell) gives a more accurate energy reading for the channel.
	 */
	ed_scan_done_cb = done_cb;
	k_work_schedule(&ed_scan_work, K_MSEC(duration > 0U ? duration : 100U));

	return 0;
}

static void bflb_ieee802154_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct bflb_ieee802154_data *data = dev->data;

	data->iface = iface;

	/* Set the EUI-64 address as the link layer address */
	net_if_set_link_addr(iface, data->mac, sizeof(data->mac), NET_LINK_IEEE802154);

	/* Initialize IEEE 802.15.4 L2 */
	ieee802154_init(iface);
}

static const struct ieee802154_radio_api bflb_ieee802154_radio_api = {
	.iface_api.init = bflb_ieee802154_iface_init,

	.get_capabilities = bflb_ieee802154_get_capabilities,
	.cca = bflb_ieee802154_cca,
	.set_channel = bflb_ieee802154_set_channel,
	.filter = bflb_ieee802154_filter,
	.set_txpower = bflb_ieee802154_set_txpower,
	.tx = bflb_ieee802154_tx,
	.start = bflb_ieee802154_start,
	.stop = bflb_ieee802154_stop,
	.configure = bflb_ieee802154_configure,
	.ed_scan = bflb_ieee802154_ed_scan,
};

/* Device init */

static int bflb_ieee802154_init(const struct device *dev)
{
	struct bflb_ieee802154_data *data = dev->data;

	k_sem_init(&data->tx_sem, 0, 1);
	k_sem_init(&data->ack_sem, 0, 1);

	/* Generate a random EUI-64 if not set from efuse */
	sys_rand_get(data->mac, sizeof(data->mac));
	/* Set the locally-administered bit, clear multicast bit */
	data->mac[0] = (data->mac[0] | 0x02U) & 0xFEU;

	/* Enable MAC154 peripheral clock and reset before any radio access */
	bflb_mac154_clock_init();

	/* Initialize lmac154 (triggers RF cal, which calls rf_reset_done_callback) */
	lmac154_init();

	/* Initialize TX power chain — required for RF TX to actually emit */
	rf_tx_pwr_init();

	lmac154_enable2015Feature();
	lmac154_enableHwAutoTxAck();

	/* Get the ISR handlers from lmac154 */
	m154_isr_handler = lmac154_getInterruptHandler();
	m154_enh_ack_isr_handler = lmac154_get2015InterruptHandler();

	/* Connect interrupts */
	IRQ_CONNECT(M154_IRQ, M154_IRQ_PRIO, m154_irq_handler, NULL, 0);
	IRQ_CONNECT(M154_ENH_ACK_IRQ, M154_ENH_ACK_IRQ_PRIO, m154_enh_ack_irq_handler, NULL, 0);
	irq_enable(M154_IRQ);
	irq_enable(M154_ENH_ACK_IRQ);

	/* Set default configuration with RF PHY optimization */
	bz_phy_optimize_tx_channel(2405U);
	lmac154_setChannel(LMAC154_CHANNEL_11);
	data->channel = BFLB_IEEE802154_DEFAULT_CHANNEL;
	lmac154_setTxPower(LMAC154_TX_POWER_10dBm);
	bz_phy_set_tx_power(BFLB_IEEE802154_DEFAULT_TX_POWER);
	data->tx_power = BFLB_IEEE802154_DEFAULT_TX_POWER;

	/* Accept all frame types */
	lmac154_disableFrameTypeFiltering();

	/* Accept frames with non-matching destination address or PAN ID.
	 * Beacon requests use broadcast PAN (0xFFFF) which won't match
	 * our PAN ID — without this, hardware drops beacon requests
	 * and the device is invisible to other Thread scanners.
	 * Also accept address mismatches for extended-address frames.
	 */
	lmac154_setRxAcceptPolicy(LMAC154_RX_ACCEPT_DST_PANID_MISMATCH |
				  LMAC154_RX_ACCEPT_DST_ADDR_MISMATCH);

	/* Set up address filtering, will be configured via filter() API */
	lmac154_setLongAddr(data->mac);

	LOG_INF("BFLB IEEE 802.15.4 initialized (lmac154 v%s)", lmac154_getVersionString());

	return 0;
}

#if defined(CONFIG_NET_L2_IEEE802154)
#define BFLB_IEEE802154_L2          IEEE802154_L2
#define BFLB_IEEE802154_L2_CTX_TYPE NET_L2_GET_CTX_TYPE(IEEE802154_L2)
#define BFLB_IEEE802154_MTU         IEEE802154_MTU
#elif defined(CONFIG_NET_L2_OPENTHREAD)
#define BFLB_IEEE802154_L2          OPENTHREAD_L2
#define BFLB_IEEE802154_L2_CTX_TYPE NET_L2_GET_CTX_TYPE(OPENTHREAD_L2)
#define BFLB_IEEE802154_MTU         1280
#endif

NET_DEVICE_DT_INST_DEFINE(0, bflb_ieee802154_init, NULL, &bflb_ieee802154_data_inst, NULL,
			  CONFIG_IEEE802154_BFLB_INIT_PRIO, &bflb_ieee802154_radio_api,
			  BFLB_IEEE802154_L2, BFLB_IEEE802154_L2_CTX_TYPE, BFLB_IEEE802154_MTU);
