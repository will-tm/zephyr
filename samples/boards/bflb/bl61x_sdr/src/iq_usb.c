/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/usb/usbd.h>

#include "capture.h"
#include "iq_control.h"
#include "iq_usb.h"
#include "stream_ring.h"

#define IQ_USB_EP_CTRL_OUT   0x01U
#define IQ_USB_EP_CTRL_IN    0x81U
#define IQ_USB_EP_STREAM_IN  0x82U
#define IQ_USB_EP_TX_OUT     0x02U
#define IQ_USB_FS_MPS        64U
#define IQ_USB_HS_MPS        512U
#define IQ_USB_NUM_EPS       4U
/* CDC-ACM instance carrying the burst protocol (composite device) */
#define CDC_ACM_CLASS_NAME   "cdc_acm_0"
#define USB_BCC_MISC_SUBCLASS_COMMON 0x02U
#define USB_BCC_MISC_PROTOCOL_IAD    0x01U

/* Zephyr project VID, reserved for Zephyr samples; host discovery uses the
 * product string
 */
#define IQ_USB_VID           0x2fe3U
#define IQ_USB_PID           0x4531U
#define IQ_USB_MAX_POWER     250U /* 2 mA units: 500 mA */

#define CTRL_HEADER_BYTES    16U
#define CTRL_REQUEST_BYTES   (CTRL_HEADER_BYTES + IQ_USB_CTRL_MAX_PAYLOAD)
/* One spare byte for short-packet padding */
#define CTRL_RESPONSE_BYTES  (CTRL_HEADER_BYTES + IQ_USB_CTRL_MAX_PAYLOAD + 1U)
#define CTRL_BUF_COUNT       2U
#define CTRL_MAGIC_BYTES     4U

#define IQ_UDP_VERSION       1U
#define IQ_UDP_HEADER_BYTES  52U
#define IQ_FRAME_MIN_BYTES   8U
#define IQ_FRAME_MAGIC_BYTES 4U
#define IQ_USB_MESSAGE_MAX_BYTES (IQ_UDP_HEADER_BYTES + sizeof(union stream_frame))
#define IQ_USB_SLOT_MAX      3U
#define IQ_USB_BATCH_FRAMES  6U
#define IQ_USB_SLOT_BYTES    (IQ_USB_BATCH_FRAMES * IQ_USB_MESSAGE_MAX_BYTES)
#define IQ_USB_SLOT_TIMEOUT_MS 250
#define QUIESCE_TIMEOUT_MS   (IQ_USB_SLOT_TIMEOUT_MS + 50)
#define QUIESCE_POLL_MS      1

#define CTRL_WQ_STACK_SIZE   4096
#define CTRL_WQ_PRIORITY     4
#define STREAM_STACK_SIZE    2048
#define STREAM_PRIORITY      2
#define STREAM_APPLY_DELAY_MS 1
#define STREAM_RETRY_DELAY_MS 1
#define STREAM_WAIT_FRAMES_MS 1U

struct iq_udp_header {
	char magic[4];
	uint16_t version;
	uint16_t header_bytes;
	uint32_t stream_epoch;
	uint32_t datagram_sequence;
	uint32_t frame_sequence;
	uint32_t source_chunk_index;
	uint32_t frame_bytes;
	uint32_t frame_crc32;
	uint32_t fragment_offset;
	uint16_t fragment_bytes;
	uint8_t fragment_index;
	uint8_t fragment_count;
	char frame_magic[4];
	uint32_t firmware_dropped_chunks;
	uint32_t header_crc32;
} __packed;

BUILD_ASSERT(sizeof(struct iq_udp_header) == IQ_UDP_HEADER_BYTES, "IQU1 header size changed");

struct iq_usb_ctrl_header {
	char magic[4];
	uint32_t sequence;
	uint32_t opcode_or_status;
	uint32_t payload_bytes;
} __packed;

BUILD_ASSERT(sizeof(struct iq_usb_ctrl_header) == CTRL_HEADER_BYTES, "control header changed");

struct iq_usb_desc {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor fs_ctrl_out;
	struct usb_ep_descriptor fs_ctrl_in;
	struct usb_ep_descriptor fs_stream_in;
	struct usb_ep_descriptor fs_tx_out;
	struct usb_ep_descriptor hs_ctrl_out;
	struct usb_ep_descriptor hs_ctrl_in;
	struct usb_ep_descriptor hs_stream_in;
	struct usb_ep_descriptor hs_tx_out;
	struct usb_desc_header nil_desc;
};

#define IQ_USB_EP_DESC(addr, mps)                                                                  \
	{                                                                                          \
		.bLength = sizeof(struct usb_ep_descriptor),                                       \
		.bDescriptorType = USB_DESC_ENDPOINT,                                              \
		.bEndpointAddress = (addr),                                                        \
		.bmAttributes = USB_EP_TYPE_BULK,                                                  \
		.wMaxPacketSize = sys_cpu_to_le16(mps),                                            \
		.bInterval = 0U,                                                                   \
	}

static struct iq_usb_desc iq_desc = {
	.if0 = {
		.bLength = sizeof(struct usb_if_descriptor),
		.bDescriptorType = USB_DESC_INTERFACE,
		.bInterfaceNumber = 0U,
		.bAlternateSetting = 0U,
		.bNumEndpoints = IQ_USB_NUM_EPS,
		.bInterfaceClass = USB_BCC_VENDOR,
		.bInterfaceSubClass = 0U,
		.bInterfaceProtocol = 0U,
		.iInterface = 0U,
	},
	.fs_ctrl_out = IQ_USB_EP_DESC(IQ_USB_EP_CTRL_OUT, IQ_USB_FS_MPS),
	.fs_ctrl_in = IQ_USB_EP_DESC(IQ_USB_EP_CTRL_IN, IQ_USB_FS_MPS),
	.fs_stream_in = IQ_USB_EP_DESC(IQ_USB_EP_STREAM_IN, IQ_USB_FS_MPS),
	.fs_tx_out = IQ_USB_EP_DESC(IQ_USB_EP_TX_OUT, IQ_USB_FS_MPS),
	.hs_ctrl_out = IQ_USB_EP_DESC(IQ_USB_EP_CTRL_OUT, IQ_USB_HS_MPS),
	.hs_ctrl_in = IQ_USB_EP_DESC(IQ_USB_EP_CTRL_IN, IQ_USB_HS_MPS),
	.hs_stream_in = IQ_USB_EP_DESC(IQ_USB_EP_STREAM_IN, IQ_USB_HS_MPS),
	.hs_tx_out = IQ_USB_EP_DESC(IQ_USB_EP_TX_OUT, IQ_USB_HS_MPS),
	.nil_desc = {.bLength = 0U, .bDescriptorType = 0U},
};

static const struct usb_desc_header *const iq_fs_desc[] = {
	(struct usb_desc_header *)&iq_desc.if0,
	(struct usb_desc_header *)&iq_desc.fs_ctrl_out,
	(struct usb_desc_header *)&iq_desc.fs_ctrl_in,
	(struct usb_desc_header *)&iq_desc.fs_stream_in,
	(struct usb_desc_header *)&iq_desc.fs_tx_out,
	(struct usb_desc_header *)&iq_desc.nil_desc,
};

static const struct usb_desc_header *const iq_hs_desc[] = {
	(struct usb_desc_header *)&iq_desc.if0,
	(struct usb_desc_header *)&iq_desc.hs_ctrl_out,
	(struct usb_desc_header *)&iq_desc.hs_ctrl_in,
	(struct usb_desc_header *)&iq_desc.hs_stream_in,
	(struct usb_desc_header *)&iq_desc.hs_tx_out,
	(struct usb_desc_header *)&iq_desc.nil_desc,
};

USBD_DEVICE_DEFINE(iq_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), IQ_USB_VID, IQ_USB_PID);
USBD_DESC_LANG_DEFINE(iq_lang);
USBD_DESC_MANUFACTURER_DEFINE(iq_mfr, "Zephyr Project");
/* Product string used for host discovery */
USBD_DESC_PRODUCT_DEFINE(iq_product, "ESP-SDR");
USBD_DESC_SERIAL_NUMBER_DEFINE(iq_sn);
USBD_DESC_STRING_DEFINE(iq_if_str, "ESP-SDR IQ stream and control", USBD_DUT_STRING_INTERFACE);
USBD_DESC_CONFIG_DEFINE(iq_fs_cfg_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(iq_hs_cfg_desc, "HS Configuration");
USBD_CONFIGURATION_DEFINE(iq_fs_config, 0U, IQ_USB_MAX_POWER, &iq_fs_cfg_desc);
USBD_CONFIGURATION_DEFINE(iq_hs_config, 0U, IQ_USB_MAX_POWER, &iq_hs_cfg_desc);

UDC_BUF_POOL_DEFINE(iq_ctrl_pool, CTRL_BUF_COUNT, CTRL_RESPONSE_BYTES, sizeof(struct udc_buf_info),
		    NULL);
UDC_BUF_POOL_DEFINE(iq_stream_pool, IQ_USB_SLOT_MAX, IQ_USB_SLOT_BYTES,
		    sizeof(struct udc_buf_info), NULL);

static struct usbd_class_data *iq_c_data;
static atomic_t mounted;
static struct k_spinlock usb_lock;
static struct net_buf *fill_buf;
static bool fill_writing;
static uint32_t inflight;
static K_SEM_DEFINE(slot_sem, IQ_USB_SLOT_MAX, IQ_USB_SLOT_MAX);
static atomic_t usb_frames;
static atomic_t usb_send_errors;
static volatile bool stream_write_active;

static struct k_spinlock stream_lock;
static atomic_t stream_owned;
static atomic_t stream_armed;
static uint32_t stream_epoch;
static uint32_t datagram_sequence;
static volatile uint32_t stream_format;

static uint8_t ctrl_request[CTRL_REQUEST_BYTES] __aligned(4);
static size_t ctrl_request_len;
static uint8_t ctrl_response[CTRL_RESPONSE_BYTES] __aligned(4);
static char json_scratch[IQ_USB_CTRL_MAX_PAYLOAD + 1U];

static void ctrl_work_handler(struct k_work *work);
static K_WORK_DEFINE(ctrl_work, ctrl_work_handler);
/* Control requests build JSON and may sleep while quiescing the stream */
static K_THREAD_STACK_DEFINE(ctrl_wq_stack, CTRL_WQ_STACK_SIZE);
static struct k_work_q ctrl_wq;

static void stream_thread(void *p1, void *p2, void *p3);
K_THREAD_DEFINE(stream_tid, STREAM_STACK_SIZE, stream_thread, NULL, NULL, NULL, STREAM_PRIORITY, 0,
		SYS_FOREVER_MS);

static bool is_hs(void);
static uint8_t ep_ctrl_out(void);
static uint8_t ep_ctrl_in(void);
static uint8_t ep_stream_in(void);
static uint16_t ep_mps(void);
static void stream_begin(void);
static void stream_arm(void);
static void stream_end(void);
static bool stream_tx_ticket(uint32_t *epoch, uint32_t *sequence);
static int ep_enqueue(struct net_buf *buf, uint8_t ep);
static void ctrl_arm_out(void);
static bool stream_submit(struct net_buf *buf);
static void stream_flush_fill(void);
static bool stream_discard_queued_data(void);
static void write_transport_header(uint8_t *packet, const uint8_t *frame, uint32_t frame_bytes,
				   uint32_t epoch, uint32_t sequence);
static void copy_words(uint8_t *dst, const uint8_t *src, size_t bytes);
static bool send_frame(const uint8_t *frame, size_t frame_bytes);
static void stream_next_frame(void);
static int iq_class_request(struct usbd_class_data *const c_data, struct net_buf *const buf,
			    const int err);
static const void *iq_class_get_desc(struct usbd_class_data *const c_data,
				     const enum usbd_speed speed);
static void iq_class_enable(struct usbd_class_data *const c_data);
static void iq_class_disable(struct usbd_class_data *const c_data);
static int iq_class_init(struct usbd_class_data *const c_data);

static const struct usbd_class_api iq_class_api = {
	.request = iq_class_request,
	.get_desc = iq_class_get_desc,
	.enable = iq_class_enable,
	.disable = iq_class_disable,
	.init = iq_class_init,
};

USBD_DEFINE_CLASS(iq_sdr, &iq_class_api, NULL, NULL);

static bool is_hs(void)
{
	return USBD_SUPPORTS_HIGH_SPEED && (usbd_bus_speed(&iq_usbd) == USBD_SPEED_HS);
}

static uint8_t ep_ctrl_out(void)
{
	return is_hs() ? iq_desc.hs_ctrl_out.bEndpointAddress
		       : iq_desc.fs_ctrl_out.bEndpointAddress;
}

static uint8_t ep_ctrl_in(void)
{
	return is_hs() ? iq_desc.hs_ctrl_in.bEndpointAddress : iq_desc.fs_ctrl_in.bEndpointAddress;
}

static uint8_t ep_stream_in(void)
{
	return is_hs() ? iq_desc.hs_stream_in.bEndpointAddress
		       : iq_desc.fs_stream_in.bEndpointAddress;
}

static uint16_t ep_mps(void)
{
	return is_hs() ? IQ_USB_HS_MPS : IQ_USB_FS_MPS;
}

/* New start wins: replacing the owner stops the previous stream */
static void stream_begin(void)
{
	k_spinlock_key_t key = k_spin_lock(&stream_lock);

	atomic_set(&stream_owned, 1);
	atomic_set(&stream_armed, 0);
	stream_epoch++;
	if (stream_epoch == 0U) {
		stream_epoch++;
	}
	datagram_sequence = 1U;
	k_spin_unlock(&stream_lock, key);
}

static void stream_arm(void)
{
	capture_set_armed(true);
}

static void stream_end(void)
{
	k_spinlock_key_t key = k_spin_lock(&stream_lock);

	atomic_set(&stream_armed, 0);
	atomic_set(&stream_owned, 0);
	k_spin_unlock(&stream_lock, key);
	capture_set_armed(false);
}

static bool stream_tx_ticket(uint32_t *epoch, uint32_t *sequence)
{
	k_spinlock_key_t key = k_spin_lock(&stream_lock);
	bool ok = (atomic_get(&stream_armed) != 0) && (atomic_get(&stream_owned) != 0);

	if (ok) {
		*epoch = stream_epoch;
		*sequence = datagram_sequence;
		datagram_sequence++;
	}
	k_spin_unlock(&stream_lock, key);

	return ok;
}

static int ep_enqueue(struct net_buf *buf, uint8_t ep)
{
	struct udc_buf_info *bi = udc_get_buf_info(buf);
	int err;

	memset(bi, 0, sizeof(*bi));
	bi->ep = ep;
	if (USB_EP_DIR_IS_IN(ep) && ((buf->len % ep_mps()) == 0U)) {
		udc_ep_buf_set_zlp(buf);
	}
	err = usbd_ep_enqueue(iq_c_data, buf);
	if (err != 0) {
		net_buf_unref(buf);
	}

	return err;
}

static void ctrl_arm_out(void)
{
	struct net_buf *buf = net_buf_alloc(&iq_ctrl_pool, K_NO_WAIT);

	if (buf != NULL) {
		(void)ep_enqueue(buf, ep_ctrl_out());
	}
}

static bool stream_submit(struct net_buf *buf)
{
	k_spinlock_key_t key = k_spin_lock(&usb_lock);

	inflight++;
	k_spin_unlock(&usb_lock, key);

	if (ep_enqueue(buf, ep_stream_in()) != 0) {
		key = k_spin_lock(&usb_lock);
		inflight--;
		k_spin_unlock(&usb_lock, key);
		k_sem_give(&slot_sem);
		(void)atomic_inc(&usb_send_errors);
		return false;
	}

	return true;
}

/* Queue the currently filling batch on the endpoint */
static void stream_flush_fill(void)
{
	k_spinlock_key_t key = k_spin_lock(&usb_lock);
	struct net_buf *buf = NULL;

	if (!fill_writing) {
		buf = fill_buf;
		fill_buf = NULL;
	}
	k_spin_unlock(&usb_lock, key);

	if (buf == NULL) {
		return;
	}
	/* Nothing may reach the endpoint after stream_discard_queued_data() */
	if ((buf->len == 0U) || (atomic_get(&stream_armed) == 0)) {
		net_buf_unref(buf);
		k_sem_give(&slot_sem);
		return;
	}
	(void)stream_submit(buf);
}

static bool stream_discard_queued_data(void)
{
	int64_t deadline = k_uptime_get() + QUIESCE_TIMEOUT_MS;
	struct net_buf *buf;
	k_spinlock_key_t key;
	bool quiet = false;

	/*
	 * stream_end() prevents new tickets; let a copy in progress finish and
	 * reclaim the batch in the same critical section as the check
	 */
	buf = NULL;
	while (k_uptime_get() < deadline) {
		key = k_spin_lock(&usb_lock);
		quiet = !fill_writing;
		if (quiet) {
			buf = fill_buf;
			fill_buf = NULL;
		}
		k_spin_unlock(&usb_lock, key);
		if (quiet) {
			break;
		}
		k_msleep(QUIESCE_POLL_MS);
	}
	if (!quiet) {
		return false;
	}

	if (buf != NULL) {
		net_buf_unref(buf);
		k_sem_give(&slot_sem);
	}

	if (atomic_get(&mounted) != 0) {
		(void)usbd_ep_dequeue(&iq_usbd, ep_stream_in());
	}
	while (k_uptime_get() < deadline) {
		key = k_spin_lock(&usb_lock);
		quiet = inflight == 0U;
		k_spin_unlock(&usb_lock, key);
		if (quiet) {
			break;
		}
		k_msleep(QUIESCE_POLL_MS);
	}

	return quiet;
}

static void write_transport_header(uint8_t *packet, const uint8_t *frame, uint32_t frame_bytes,
				   uint32_t epoch, uint32_t sequence)
{
	struct iq_udp_header *header = (struct iq_udp_header *)packet;
	bool iq_frame = (memcmp(frame, "IQC", IQ_FRAME_MAGIC_BYTES - 1U) == 0) &&
			((frame[IQ_FRAME_MAGIC_BYTES - 1U] == (uint8_t)'1') ||
			 (frame[IQ_FRAME_MAGIC_BYTES - 1U] == (uint8_t)'8'));
	uint32_t frame_sequence;
	uint32_t source_chunk = 0U;
	uint32_t dropped = 0U;
	uint32_t frame_crc;

	memcpy(&frame_sequence, &frame[offsetof(struct iq_chunk, sequence)],
	       sizeof(frame_sequence));
	if (iq_frame) {
		memcpy(&source_chunk, &frame[offsetof(struct iq_chunk, source_chunk_index)],
		       sizeof(source_chunk));
		memcpy(&dropped, &frame[offsetof(struct iq_chunk, dropped_chunks)],
		       sizeof(dropped));
		memcpy(&frame_crc, &frame[frame_bytes - sizeof(frame_crc)], sizeof(frame_crc));
	} else {
		frame_crc = crc32_ieee(frame, frame_bytes);
	}

	memset(header, 0, sizeof(*header));
	memcpy(header->magic, "IQU1", sizeof(header->magic));
	header->version = IQ_UDP_VERSION;
	header->header_bytes = sizeof(*header);
	header->stream_epoch = epoch;
	header->datagram_sequence = sequence;
	header->frame_sequence = frame_sequence;
	header->source_chunk_index = source_chunk;
	header->frame_bytes = frame_bytes;
	header->frame_crc32 = frame_crc;
	header->fragment_offset = 0U;
	header->fragment_bytes = (uint16_t)frame_bytes;
	header->fragment_index = 0U;
	header->fragment_count = 1U;
	memcpy(header->frame_magic, frame, sizeof(header->frame_magic));
	header->firmware_dropped_chunks = dropped;
	header->header_crc32 = crc32_ieee(packet, offsetof(struct iq_udp_header, header_crc32));
}

/*
 * Messages are word aligned (52-byte header, frames in whole words); the
 * size optimized libc memcpy copies byte-wise, far slower at stream rates.
 */
static void copy_words(uint8_t *dst, const uint8_t *src, size_t bytes)
{
	uint32_t *d = (uint32_t *)dst;
	const uint32_t *s = (const uint32_t *)src;

	for (size_t i = 0U; i < (bytes / sizeof(uint32_t)); i++) {
		d[i] = s[i];
	}
}

static bool send_frame(const uint8_t *frame, size_t frame_bytes)
{
	const size_t message_bytes = IQ_UDP_HEADER_BYTES + frame_bytes;
	struct net_buf *buf = NULL;
	k_spinlock_key_t key;
	uint32_t epoch;
	uint32_t sequence;
	uint8_t *packet;
	bool idle;

	if ((frame_bytes < IQ_FRAME_MIN_BYTES) || (frame_bytes > sizeof(union stream_frame)) ||
	    ((frame_bytes % sizeof(uint32_t)) != 0U) || (atomic_get(&mounted) == 0)) {
		return false;
	}
	if (!stream_tx_ticket(&epoch, &sequence)) {
		return false;
	}

	while (buf == NULL) {
		key = k_spin_lock(&usb_lock);
		/* stream_end() may run while a ticket is held */
		if (atomic_get(&stream_armed) == 0) {
			k_spin_unlock(&usb_lock, key);
			return false;
		}
		if ((fill_buf != NULL) && (net_buf_tailroom(fill_buf) >= message_bytes)) {
			fill_writing = true;
			buf = fill_buf;
		}
		k_spin_unlock(&usb_lock, key);
		if (buf != NULL) {
			break;
		}
		/* Full batch: park it for the endpoint */
		stream_flush_fill();
		if (k_sem_take(&slot_sem, K_MSEC(IQ_USB_SLOT_TIMEOUT_MS)) != 0) {
			(void)atomic_inc(&usb_send_errors);
			return false;
		}
		buf = net_buf_alloc(&iq_stream_pool, K_NO_WAIT);
		if (buf == NULL) {
			k_sem_give(&slot_sem);
			(void)atomic_inc(&usb_send_errors);
			return false;
		}
		key = k_spin_lock(&usb_lock);
		/* The stream may have ended while waiting for a slot */
		if (atomic_get(&stream_armed) == 0) {
			k_spin_unlock(&usb_lock, key);
			net_buf_unref(buf);
			k_sem_give(&slot_sem);
			return false;
		}
		fill_buf = buf;
		fill_writing = true;
		k_spin_unlock(&usb_lock, key);
	}

	packet = net_buf_add(buf, message_bytes);
	copy_words(&packet[IQ_UDP_HEADER_BYTES], frame, frame_bytes);
	write_transport_header(packet, &packet[IQ_UDP_HEADER_BYTES], frame_bytes, epoch, sequence);

	key = k_spin_lock(&usb_lock);
	fill_writing = false;
	/*
	 * Ship when the endpoint is idle; with one transfer in flight, queue the
	 * batch once it is half full so the endpoint never starves
	 */
	idle = (inflight == 0U) ||
	       ((inflight == 1U) && (buf->len >= (IQ_USB_SLOT_BYTES / 2U)));
	k_spin_unlock(&usb_lock, key);
	if (idle) {
		stream_flush_fill();
	}
	(void)atomic_inc(&usb_frames);

	return true;
}

static void stream_next_frame(void)
{
	union stream_frame *frame;
	size_t frame_size;
	bool sent = false;

	if (capture_config_applying()) {
		k_msleep(STREAM_APPLY_DELAY_MS);
		return;
	}
	frame = stream_ring_peek();
	if (frame == NULL) {
		(void)stream_ring_wait_frames(STREAM_WAIT_FRAMES_MS);
		frame = stream_ring_peek();
		if (frame == NULL) {
			return;
		}
	}

	stream_write_active = true;
	if (capture_config_applying()) {
		stream_write_active = false;
		k_msleep(STREAM_APPLY_DELAY_MS);
		return;
	}
	frame_size = stream_frame_wire_size(frame);
	if (frame_size == 0U) {
		stream_ring_pop();
		stream_write_active = false;
		return;
	}
	if (iq_usb_stream_owned()) {
		sent = send_frame((const uint8_t *)frame, frame_size);
	}
	if (sent || !iq_usb_stream_armed()) {
		stream_ring_pop();
	} else {
		/* Preserve the slot across transient USB backpressure */
		stream_write_active = false;
		k_msleep(STREAM_RETRY_DELAY_MS);
		return;
	}
	if (iq_usb_stream_owned() && (stream_ring_peek() == NULL)) {
		stream_flush_fill();
	}
	stream_write_active = false;
}

static void stream_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		capture_service_pending_config();
		stream_next_frame();
	}
}

static void ctrl_work_handler(struct k_work *work)
{
	const struct iq_usb_ctrl_header *request = (const struct iq_usb_ctrl_header *)ctrl_request;
	struct iq_usb_ctrl_header *response = (struct iq_usb_ctrl_header *)ctrl_response;
	char *payload_out = (char *)&ctrl_response[CTRL_HEADER_BYTES];
	char *payload_in = (char *)&ctrl_request[CTRL_HEADER_BYTES];
	struct capture_config pending;
	bool apply_pending = false;
	bool arm_stream = false;
	const char *error = NULL;
	struct net_buf *buf;
	int n = 0;
	uint32_t total;

	ARG_UNUSED(work);

	memcpy(response->magic, "IQRS", CTRL_MAGIC_BYTES);
	response->sequence = 0U;
	response->opcode_or_status = 1U;
	response->payload_bytes = 0U;

	if ((ctrl_request_len < CTRL_HEADER_BYTES) ||
	    (memcmp(request->magic, "IQRQ", CTRL_MAGIC_BYTES) != 0) ||
	    (request->payload_bytes > IQ_USB_CTRL_MAX_PAYLOAD) ||
	    ((CTRL_HEADER_BYTES + request->payload_bytes) > ctrl_request_len)) {
		error = "malformed control request";
	} else {
		response->sequence = request->sequence;
		switch (request->opcode_or_status) {
		case IQ_USB_OP_GET_STATUS:
			n = iq_control_build_status_json(payload_out, IQ_USB_CTRL_MAX_PAYLOAD);
			error = (n < 0) ? "status JSON too large" : NULL;
			break;
		case IQ_USB_OP_GET_CONFIG:
			n = iq_control_build_config_json(payload_out, IQ_USB_CTRL_MAX_PAYLOAD);
			error = (n < 0) ? "config JSON too large" : NULL;
			break;
		case IQ_USB_OP_PUT_CONFIG:
			memcpy(json_scratch, payload_in, request->payload_bytes);
			json_scratch[request->payload_bytes] = '\0';
			error = iq_control_parse_config_json(json_scratch, request->payload_bytes,
							     &pending);
			apply_pending = error == NULL;
			n = 0;
			break;
		case IQ_USB_OP_STREAM_START: {
			uint32_t format = IQ_USB_FORMAT_FULL;

			if (request->payload_bytes > 0U) {
				memcpy(json_scratch, payload_in, request->payload_bytes);
				json_scratch[request->payload_bytes] = '\0';
				format = iq_control_parse_stream_format(
					json_scratch, request->payload_bytes, format);
			}
			stream_end();
			if (!stream_discard_queued_data()) {
				error = "USB stream endpoint did not quiesce";
				break;
			}
			stream_format = format;
			stream_begin();
			n = iq_control_build_status_json(payload_out, IQ_USB_CTRL_MAX_PAYLOAD);
			error = (n < 0) ? "status JSON too large" : NULL;
			arm_stream = error == NULL;
			break;
		}
		case IQ_USB_OP_STREAM_STOP:
			stream_end();
			if (!stream_discard_queued_data()) {
				error = "USB stream endpoint did not quiesce";
				break;
			}
			n = iq_control_build_status_json(payload_out, IQ_USB_CTRL_MAX_PAYLOAD);
			error = (n < 0) ? "status JSON too large" : NULL;
			break;
		case IQ_USB_OP_TX_ARM:
		case IQ_USB_OP_TX_COMMIT:
		case IQ_USB_OP_TX_ABORT:
			error = "TX replay is not supported on BL61x";
			break;
		default:
			error = "unknown opcode";
			break;
		}
		if (error == NULL) {
			response->opcode_or_status = 0U;
			response->payload_bytes = (n > 0) ? (uint32_t)n : 0U;
		}
	}

	if (error != NULL) {
		size_t len = MIN(strlen(error), IQ_USB_CTRL_MAX_PAYLOAD);

		memcpy(payload_out, error, len);
		response->opcode_or_status = 1U;
		response->payload_bytes = (uint32_t)len;
	}

	/* Guarantee short-packet termination for exact-multiple responses */
	total = CTRL_HEADER_BYTES + response->payload_bytes;
	if ((total % ep_mps()) == 0U) {
		payload_out[response->payload_bytes] = ' ';
		response->payload_bytes++;
		total++;
	}

	buf = net_buf_alloc(&iq_ctrl_pool, K_NO_WAIT);
	if (buf != NULL) {
		net_buf_add_mem(buf, ctrl_response, total);
		if (ep_enqueue(buf, ep_ctrl_in()) != 0) {
			ctrl_arm_out();
		}
	} else {
		ctrl_arm_out();
	}

	/* Queue configuration and arming only after the response is owned by
	 * the hardware
	 */
	if (apply_pending) {
		capture_apply_config(&pending);
	}
	if (arm_stream) {
		stream_arm();
	}
}

static int iq_class_request(struct usbd_class_data *const c_data, struct net_buf *const buf,
			    const int err)
{
	struct udc_buf_info *bi = udc_get_buf_info(buf);
	const uint8_t ep = bi->ep;
	k_spinlock_key_t key;

	ARG_UNUSED(c_data);

	if (ep == ep_ctrl_out()) {
		if ((err == 0) && (buf->len <= sizeof(ctrl_request))) {
			memcpy(ctrl_request, buf->data, buf->len);
			ctrl_request_len = buf->len;
			net_buf_unref(buf);
			(void)k_work_submit_to_queue(&ctrl_wq, &ctrl_work);
			return 0;
		}
		net_buf_unref(buf);
		if (err == 0) {
			ctrl_arm_out();
		}
	} else if (ep == ep_ctrl_in()) {
		net_buf_unref(buf);
		/* Response delivered: accept the next request */
		if (err == 0) {
			ctrl_arm_out();
		}
	} else if (ep == ep_stream_in()) {
		net_buf_unref(buf);
		key = k_spin_lock(&usb_lock);
		inflight--;
		k_spin_unlock(&usb_lock, key);
		k_sem_give(&slot_sem);
		/* Cancellations requested by stream_discard_queued_data() */
		if ((err != 0) && (err != -ECONNABORTED)) {
			(void)atomic_inc(&usb_send_errors);
		}
	} else {
		net_buf_unref(buf);
	}

	return 0;
}

static const void *iq_class_get_desc(struct usbd_class_data *const c_data,
				     const enum usbd_speed speed)
{
	ARG_UNUSED(c_data);

	if (USBD_SUPPORTS_HIGH_SPEED && (speed == USBD_SPEED_HS)) {
		return iq_hs_desc;
	}

	return iq_fs_desc;
}

static void iq_class_enable(struct usbd_class_data *const c_data)
{
	iq_c_data = c_data;
	atomic_set(&mounted, 1);
	ctrl_arm_out();
}

static void iq_class_disable(struct usbd_class_data *const c_data)
{
	ARG_UNUSED(c_data);

	atomic_set(&mounted, 0);
	stream_end();
}

static int iq_class_init(struct usbd_class_data *const c_data)
{
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);

	iq_c_data = c_data;
	if ((iq_desc.if0.iInterface == 0U) && (usbd_add_descriptor(uds_ctx, &iq_if_str) == 0)) {
		iq_desc.if0.iInterface = usbd_str_desc_get_idx(&iq_if_str);
	}

	return 0;
}

int iq_usb_init(void)
{
	int err;

	k_work_queue_start(&ctrl_wq, ctrl_wq_stack, K_THREAD_STACK_SIZEOF(ctrl_wq_stack),
			   CTRL_WQ_PRIORITY, NULL);
	k_thread_name_set(k_work_queue_thread_get(&ctrl_wq), "iq_ctrl");

	err = usbd_add_descriptor(&iq_usbd, &iq_lang);
	if (err == 0) {
		err = usbd_add_descriptor(&iq_usbd, &iq_mfr);
	}
	if (err == 0) {
		err = usbd_add_descriptor(&iq_usbd, &iq_product);
	}
	if (err == 0) {
		err = usbd_add_descriptor(&iq_usbd, &iq_sn);
	}
	if ((err == 0) && USBD_SUPPORTS_HIGH_SPEED &&
	    (usbd_caps_speed(&iq_usbd) == USBD_SPEED_HS)) {
		err = usbd_add_configuration(&iq_usbd, USBD_SPEED_HS, &iq_hs_config);
		if (err == 0) {
			err = usbd_register_class(&iq_usbd, "iq_sdr", USBD_SPEED_HS, 1);
		}
		if (err == 0) {
			err = usbd_register_class(&iq_usbd, CDC_ACM_CLASS_NAME, USBD_SPEED_HS, 1);
		}
		if (err == 0) {
			usbd_device_set_code_triple(&iq_usbd, USBD_SPEED_HS, USB_BCC_MISCELLANEOUS,
						    USB_BCC_MISC_SUBCLASS_COMMON,
						    USB_BCC_MISC_PROTOCOL_IAD);
		}
	}
	if (err == 0) {
		err = usbd_add_configuration(&iq_usbd, USBD_SPEED_FS, &iq_fs_config);
	}
	if (err == 0) {
		err = usbd_register_class(&iq_usbd, "iq_sdr", USBD_SPEED_FS, 1);
	}
	if (err == 0) {
		err = usbd_register_class(&iq_usbd, CDC_ACM_CLASS_NAME, USBD_SPEED_FS, 1);
	}
	if (err == 0) {
		usbd_device_set_code_triple(&iq_usbd, USBD_SPEED_FS, USB_BCC_MISCELLANEOUS,
					    USB_BCC_MISC_SUBCLASS_COMMON,
					    USB_BCC_MISC_PROTOCOL_IAD);
		err = usbd_init(&iq_usbd);
	}
	if (err == 0) {
		err = usbd_enable(&iq_usbd);
	}
	if (err == 0) {
		k_thread_name_set(stream_tid, "iq_stream");
		k_thread_start(stream_tid);
	}

	return err;
}

bool iq_usb_mounted(void)
{
	return atomic_get(&mounted) != 0;
}

bool iq_usb_stream_armed(void)
{
	return atomic_get(&stream_armed) != 0;
}

void iq_usb_stream_armed_set(bool armed)
{
	k_spinlock_key_t key = k_spin_lock(&stream_lock);

	atomic_set(&stream_armed, (armed && (atomic_get(&stream_owned) != 0)) ? 1 : 0);
	k_spin_unlock(&stream_lock, key);
}

bool iq_usb_stream_owned(void)
{
	return atomic_get(&stream_owned) != 0;
}

uint32_t iq_usb_stream_format(void)
{
	return stream_format;
}

uint32_t iq_usb_stream_epoch(void)
{
	return stream_epoch;
}

bool iq_usb_stream_write_active(void)
{
	return stream_write_active;
}

uint32_t iq_usb_frames(void)
{
	return (uint32_t)atomic_get(&usb_frames);
}

uint32_t iq_usb_send_errors(void)
{
	return (uint32_t)atomic_get(&usb_send_errors);
}
