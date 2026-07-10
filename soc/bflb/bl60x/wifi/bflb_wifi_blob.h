/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BL602 wifi4 blob specifics: the blob's `struct ipc_shared_env_tag` is
 * 0x6f4 bytes and does not match the SDK header, so the shared-env fields
 * are addressed by raw offset here.
 */

#ifndef BFLB_WIFI_BLOB_BL60X_H_
#define BFLB_WIFI_BLOB_BL60X_H_

#include <stdint.h>

/* Per-descriptor stride in the blob's txdesc0 array: vendor txdesc_host is
 * { list_hdr 4 + host_id 4 + ready 4 + pad_txdesc[208] + pad_buf[400] },
 * padded to 624 bytes, two descriptors total.
 */
#define BFLB_WIFI_TXDESC_STRIDE 624U
#define BFLB_WIFI_TXDESC_COUNT  2U

volatile uint8_t *bflb_wifi_ipc_txdesc(uint32_t idx);

#endif /* BFLB_WIFI_BLOB_BL60X_H_ */
