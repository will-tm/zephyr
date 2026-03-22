/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_BFLB_COMMON_BFLB_MAC154_H_
#define SOC_BFLB_COMMON_BFLB_MAC154_H_

/**
 * @brief Enable the MAC154/Zigbee peripheral clock and perform a hardware reset.
 *
 * Must be called before any lmac154 or radio register access.
 */
void bflb_mac154_clock_init(void);

#endif /* SOC_BFLB_COMMON_BFLB_MAC154_H_ */
