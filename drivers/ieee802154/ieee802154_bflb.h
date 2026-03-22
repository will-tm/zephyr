/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_IEEE802154_IEEE802154_BFLB_H_
#define ZEPHYR_DRIVERS_IEEE802154_IEEE802154_BFLB_H_

#include <zephyr/net/ieee802154_radio.h>

#define BFLB_IEEE802154_MAX_PKT_LEN 127
#define BFLB_IEEE802154_FCS_LEN     2
#define BFLB_IEEE802154_CHANNEL_MIN 11
#define BFLB_IEEE802154_CHANNEL_MAX 26

struct bflb_ieee802154_data {
	struct net_if *iface;
	uint8_t mac[8];

	struct k_sem tx_sem;
	struct k_sem ack_sem;

	uint8_t tx_status;
	uint8_t ack_received;
	uint8_t ack_frame_pending;
	uint8_t ack_seq_num;

	/* Enhanced ACK frame storage */
	uint8_t ack_frame_buf[BFLB_IEEE802154_MAX_PKT_LEN];
	uint8_t ack_frame_len;

	uint16_t channel;
	int16_t tx_power;
	bool promiscuous;
	bool started;
};

#endif /* ZEPHYR_DRIVERS_IEEE802154_IEEE802154_BFLB_H_ */
