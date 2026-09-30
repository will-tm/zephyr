/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* JSON documents of the ESP-SDR control API (config and status) */

#ifndef BL61X_SDR_IQ_CONTROL_H_
#define BL61X_SDR_IQ_CONTROL_H_

#include <stddef.h>
#include <stdint.h>

#include "capture.h"

int iq_control_build_config_json(char *text, size_t cap);
int iq_control_build_status_json(char *text, size_t cap);
/*
 * Parses a configuration document on top of the current configuration.
 * Returns NULL on success or an error message.
 */
const char *iq_control_parse_config_json(char *json, size_t len, struct capture_config *config);
/* Reads "stream_format" from a stream-start document */
uint32_t iq_control_parse_stream_format(char *json, size_t len, uint32_t fallback);

#endif /* BL61X_SDR_IQ_CONTROL_H_ */
