/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * USB transport of the ESP-SDR streaming firmware: one vendor interface.
 *
 * Endpoints:
 *   0x01 bulk OUT  control requests
 *   0x81 bulk IN   control responses
 *   0x82 bulk IN   IQ stream
 *   0x02 bulk OUT  TX upload (declared, TX replay is not supported)
 *
 * Control protocol: a 16-byte little-endian request header ("IQRQ",
 * sequence, opcode, payload_bytes) optionally followed by a JSON payload,
 * answered by a 16-byte response header ("IQRS", sequence, status,
 * payload_bytes) plus JSON.
 *
 * Stream framing: every stream frame (IQC1/IQC8/CFG1) is prefixed with the
 * 52-byte IQU1 transport header and sent unfragmented; several messages may
 * share one bulk transfer and are delimited by the header's frame size.
 */

#ifndef BL61X_SDR_IQ_USB_H_
#define BL61X_SDR_IQ_USB_H_

#include <stdbool.h>
#include <stdint.h>

#define IQ_USB_CTRL_MAX_PAYLOAD 2304U

enum {
	IQ_USB_OP_GET_STATUS = 1U,
	IQ_USB_OP_GET_CONFIG = 2U,
	IQ_USB_OP_PUT_CONFIG = 3U,
	IQ_USB_OP_STREAM_START = 4U,
	IQ_USB_OP_STREAM_STOP = 5U,
	IQ_USB_OP_TX_ARM = 6U,
	IQ_USB_OP_TX_COMMIT = 7U,
	IQ_USB_OP_TX_ABORT = 8U,
};

/* Stream-start payload field "stream_format" selects the IQ wire format */
enum {
	IQ_USB_FORMAT_FULL = 0U, /* IQC1: 32-bit dump words */
	IQ_USB_FORMAT_INT8 = 1U, /* IQC8: interleaved int8 I/Q */
	IQ_USB_FORMAT_INT4 = 2U, /* IQC4 on ESP32-S31 only; IQC1 frames here */
};

int iq_usb_init(void);
bool iq_usb_mounted(void);
bool iq_usb_stream_armed(void);
void iq_usb_stream_armed_set(bool armed);
bool iq_usb_stream_owned(void);
uint32_t iq_usb_stream_format(void);
uint32_t iq_usb_stream_epoch(void);
bool iq_usb_stream_write_active(void);
uint32_t iq_usb_frames(void);
uint32_t iq_usb_send_errors(void);

#endif /* BL61X_SDR_IQ_USB_H_ */
