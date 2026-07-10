/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BL606P/BL808 wifi4 blob specifics.  Unlike the BL602 blob, the shared
 * environment matches `struct ipc_shared_env_tag` from the BL808 SDK
 * headers exactly (verified against the blob's DWARF), so the fields are
 * accessed through the struct and the TX descriptors are managed through
 * the shared list_free / list_ongoing / list_cfm queues.
 */

#ifndef BFLB_WIFI_BLOB_BL808_H_
#define BFLB_WIFI_BLOB_BL808_H_

/* A2E doorbell bit for TX queue 0 (IPC_IRQ_A2E_TXDESC first bit). */
#define BFLB_IPC_A2E_TXDESC0 BIT(8)

#endif /* BFLB_WIFI_BLOB_BL808_H_ */
