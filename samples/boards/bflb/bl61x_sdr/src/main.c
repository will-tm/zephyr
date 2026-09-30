/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * BL61x WLAN receiver raw I/Q capture.
 *
 * The RF is calibrated by bflb_rf_init(), tuned with the phyrf blob and
 * forced into receive. Two capture paths exist:
 *
 * - UART and USB CDC-ACM: finite snapshots served with the ESP-SDR burst
 *   protocol 6, so the ESPARGOS ESP-SDR web viewer can be used unmodified.
 * - USB: the continuous 128 KiB dump ring streamed with the ESP-SDR USB
 *   vendor protocol (see iq_usb.h).
 */

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

#include "capture.h"
#include "dump_ring.h"
#include "iq_format.h"
#include "iq_usb.h"
#include "resample.h"
#include "rf.h"

/* The web viewer only accepts ESP32 family identities */
#define SDR_IDENTITY         "C6SDR"
#define SDR_PROTOCOL         6U

#define IQ_WORDS             16380U
#define IQ_MIN_WORDS         256U
#define CANARY_WORDS         4U
#define FILL_PATTERN         0xa5a0055aU
#define CANARY_PATTERN       0x5a5aa5a5U

/* Software AGC iterations before a capture */
#define AGC_MAX_STEPS        8U

#define FMT_RAW              0U
#define FMT_IQ8              16U
#define FMT_IQ10             20U
#define BYTES_PER_WORD       4U
#define IQ8_BYTES            2U
#define IQ10_PAIR_BYTES      5U
#define IQ10_TAIL_BYTES      3U
#define IQ10_BITS            10U
#define IQ10_MASK            0x3ffU
#define BYTE_BITS            8U
#define BYTE_MASK            0xffU
#define NIBBLE_BITS          4U
#define TWO_BYTE_BITS        16U
#define TWELVE_BITS          12U

#define RXRUN_MAX_REPEATS    1000U
#define LINE_SIZE            128U
#define REPLY_SIZE           160U
#define PROBE_REPLY_SIZE     4096U
#define TRACE_MAX_SAMPLES    100U
#define HEX_OR_DECIMAL       0
#define PEEK_MAX_WORDS       32768U
#define MAX_ARGS             4U
#define DECIMAL              10
#define RX_RING_SIZE         256U
#define CRC32_TABLE_SIZE     256U
#define RATE_COUNT           7U
/* The 16 MS/s resampler takes 5/2 input words per sample */
#define RATE_16M_UP          2U
#define RATE_16M_DOWN        5U
#define CRC32_POLY_REFLECTED 0xedb88320U

#define UART_NODE            DT_CHOSEN(zephyr_console)
#define UART_BAUD            DT_PROP(UART_NODE, current_speed)
#define CDC_NODE             DT_NODELABEL(cdc_acm_uart0)

#define PORT_UART            0U
#define PORT_CDC             1U
#define PORT_COUNT           2U

/* One client controls the radio; silence releases it */
#define LEASE_TIMEOUT_MS     5000
/* CDC-ACM transmit stall before the reply is dropped */
#define CDC_TX_TIMEOUT_MS    3000

/* A capture rate: dump tap and sample width, then optional resampling */
struct rate {
	uint32_t hz;
	uint32_t dump_sel;
	uint32_t width;
	const struct resample *resample;
};

struct port {
	const struct device *dev;
	struct ring_buf *rx_ring;
	char line[LINE_SIZE];
	size_t line_used;
	bool line_overflow;
	bool usb;
};

BUILD_ASSERT((IQ_WORDS + CANARY_WORDS) <= RF_RING_WORDS, "snapshot must fit in the dump window");
BUILD_ASSERT(((((IQ_WORDS - 1U) * RATE_16M_DOWN) / RATE_16M_UP) + 1U + CANARY_WORDS) <=
		     RF_WINDOW_WORDS,
	     "16 MS/s snapshot must fit in the dump window");

/* Interrupt-driven RX: polling cannot keep up with 2 MBaud commands */
RING_BUF_DECLARE(uart_rx_ring, RX_RING_SIZE);
RING_BUF_DECLARE(cdc_rx_ring, RX_RING_SIZE);
static K_SEM_DEFINE(rx_sem, 0, 1);

static struct port ports[PORT_COUNT] = {
	[PORT_UART] = {
		.dev = DEVICE_DT_GET(UART_NODE),
		.rx_ring = &uart_rx_ring,
		.usb = false,
	},
	[PORT_CDC] = {
		.dev = DEVICE_DT_GET(CDC_NODE),
		.rx_ring = &cdc_rx_ring,
		.usb = true,
	},
};
/* Port of the command being handled, replies go there */
static struct port *active_port = &ports[PORT_UART];

/*
 * CDC-ACM transmit runs from its callback: poll_out only hands one bulk
 * packet per millisecond to the stack.
 */
static K_MUTEX_DEFINE(cdc_tx_lock);
static K_SEM_DEFINE(cdc_tx_sem, 0, 1);
static const uint8_t *cdc_tx_ptr;
static size_t cdc_tx_len;
/* RX interrupt paused while the command ring is full */
static atomic_t cdc_rx_paused;

static char probe_reply[PROBE_REPLY_SIZE];
/* Packed snapshot in cached RAM: the largest payload is raw words */
static __noinit uint32_t snap_out[IQ_WORDS];
static uint32_t crc32_table[CRC32_TABLE_SIZE];

/* ESP-SDR rate indices: 80, 40, 20, 10, 8, 4, 16 MS/s; 0 Hz is unsupported */
static const struct rate rates[RATE_COUNT] = {
	[0] = {80000000U, RF_DUMP_SEL_80M, RF_DUMP_WIDTH_80M, NULL},
	[1] = {40000000U, RF_DUMP_SEL_40M, RF_DUMP_WIDTH_40M, NULL},
	[2] = {20000000U, RF_DUMP_SEL_40M, RF_DUMP_WIDTH_40M, &resample_40m_20m},
	[6] = {16000000U, RF_DUMP_SEL_40M, RF_DUMP_WIDTH_40M, &resample_40m_16m},
};
/* GAIN HARDWARE: software AGC before every capture */
static bool agc_enabled = true;

static void uart_isr(const struct device *dev, void *user_data);
static void cdc_isr(const struct device *dev, void *user_data);
static void cdc_send(const struct device *dev, const uint8_t *data, size_t len);
static void send(const void *data, size_t len);
static void reply(const char *str);
static int poll_line(struct port *p);
static int port_init(struct port *p, uart_irq_callback_user_data_t cb);
static int parse_args(const char *str, uint32_t *args, size_t max_args);
static uint32_t iq20(uint32_t word);
static void crc32_init(void);
static uint32_t crc32(const uint8_t *data, size_t len);
static size_t pack(const uint32_t *src, uint32_t n, uint32_t fmt, uint32_t width, uint8_t *out);
static bool rate_valid(uint32_t rate);
static bool capture_owned(uint32_t n, uint32_t fmt, uint32_t rate);
static bool capture(uint32_t n, uint32_t fmt, uint32_t rate);
static bool capture_cmd(const char *args, uint32_t fmt);
static void rxrun_cmd(const char *args);
static bool ring_command(const char *line);
static int parse_nums(const char *str, uint32_t *args, size_t max_args);
static bool debug_command(const char *line);
static void handle_command(const char *line);

static void uart_isr(const struct device *dev, void *user_data)
{
	uint8_t c;

	ARG_UNUSED(user_data);

	uart_irq_update(dev);
	while (uart_irq_rx_ready(dev) != 0) {
		/* One byte per ready check: the driver's fifo_read returns 0 */
		(void)uart_fifo_read(dev, &c, 1);
		(void)ring_buf_put(&uart_rx_ring, &c, 1U);
	}
	k_sem_give(&rx_sem);
}

/* Runs in the CDC-ACM workqueue */
static void cdc_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uart_irq_update(dev);
	while (uart_irq_rx_ready(dev) != 0) {
		uint8_t *dst;
		uint32_t space = ring_buf_put_ptr(&cdc_rx_ring, &dst, 0U);
		int n;

		if (space == 0U) {
			/* The command loop resumes RX once it drained the ring */
			uart_irq_rx_disable(dev);
			(void)atomic_set(&cdc_rx_paused, 1);
			break;
		}
		n = uart_fifo_read(dev, dst, (int)space);
		if (n <= 0) {
			break;
		}
		ring_buf_commit(&cdc_rx_ring, (size_t)n);
	}
	k_sem_give(&rx_sem);

	(void)k_mutex_lock(&cdc_tx_lock, K_FOREVER);
	if (cdc_tx_len > 0U) {
		int n = uart_fifo_fill(dev, cdc_tx_ptr, (int)MIN(cdc_tx_len, (size_t)INT_MAX));

		if (n > 0) {
			cdc_tx_ptr = &cdc_tx_ptr[n];
			cdc_tx_len -= (size_t)n;
			k_sem_give(&cdc_tx_sem);
		}
	}
	if (cdc_tx_len == 0U) {
		uart_irq_tx_disable(dev);
	}
	(void)k_mutex_unlock(&cdc_tx_lock);
}

/* Blocks until the stack took all data, drops the rest when the host stalls */
static void cdc_send(const struct device *dev, const uint8_t *data, size_t len)
{
	size_t left = len;

	k_sem_reset(&cdc_tx_sem);
	(void)k_mutex_lock(&cdc_tx_lock, K_FOREVER);
	cdc_tx_ptr = data;
	cdc_tx_len = len;
	(void)k_mutex_unlock(&cdc_tx_lock);
	uart_irq_tx_enable(dev);

	while (left > 0U) {
		int ret = k_sem_take(&cdc_tx_sem, K_MSEC(CDC_TX_TIMEOUT_MS));

		(void)k_mutex_lock(&cdc_tx_lock, K_FOREVER);
		if (ret != 0) {
			cdc_tx_len = 0U;
			uart_irq_tx_disable(dev);
		}
		left = cdc_tx_len;
		(void)k_mutex_unlock(&cdc_tx_lock);
	}
}

static void send(const void *data, size_t len)
{
	const uint8_t *p = data;

	if (active_port->usb) {
		cdc_send(active_port->dev, p, len);
		return;
	}
	for (size_t i = 0U; i < len; i++) {
		uart_poll_out(active_port->dev, p[i]);
	}
}

static void reply(const char *str)
{
	send(str, strlen(str));
}

/*
 * Returns 1 for a complete line, 0 when incomplete, -1 when too long.
 * Empty lines are skipped.
 */
static int poll_line(struct port *p)
{
	int ret = 0;
	uint8_t c;

	while ((ret == 0) && (ring_buf_get(p->rx_ring, &c, 1U) == 1U)) {
		if ((c == (uint8_t)'\n') || (c == (uint8_t)'\r')) {
			if (p->line_overflow) {
				ret = -1;
			} else if (p->line_used > 0U) {
				ret = 1;
			} else {
				continue;
			}
			p->line[p->line_used] = '\0';
			p->line_used = 0U;
			p->line_overflow = false;
		} else if (p->line_used < (LINE_SIZE - 1U)) {
			p->line[p->line_used] = (char)c;
			p->line_used++;
		} else {
			p->line_overflow = true;
		}
	}

	if (p->usb && atomic_cas(&cdc_rx_paused, 1, 0)) {
		uart_irq_rx_enable(p->dev);
	}

	return ret;
}

static int port_init(struct port *p, uart_irq_callback_user_data_t cb)
{
	int ret;

	if (!device_is_ready(p->dev)) {
		return -ENODEV;
	}
	ret = uart_irq_callback_user_data_set(p->dev, cb, NULL);
	if (ret != 0) {
		return ret;
	}
	uart_irq_rx_enable(p->dev);

	return 0;
}

/* Parses space separated decimal values, returns the count or -1 */
static int parse_args(const char *str, uint32_t *args, size_t max_args)
{
	const char *p = str;
	size_t count = 0U;

	while (*p != '\0') {
		char *end;
		unsigned long val;

		if ((*p < '0') || (*p > '9') || (count == max_args)) {
			return -1;
		}
		errno = 0;
		val = strtoul(p, &end, DECIMAL);
		if ((errno != 0) || (val > UINT32_MAX) || ((*end != ' ') && (*end != '\0'))) {
			return -1;
		}
		args[count] = (uint32_t)val;
		count++;
		p = (*end == ' ') ? (end + 1) : end;
	}

	return (int)count;
}

/* Interleaved 10-bit pair of any layout: I in bits 9:0, Q in bits 19:10 */
static uint32_t iq20(uint32_t word)
{
	return ((uint32_t)iq_format_i10(word) & IQ10_MASK) |
	       (((uint32_t)iq_format_q10(word) & IQ10_MASK) << IQ10_BITS);
}

static void crc32_init(void)
{
	for (uint32_t i = 0U; i < CRC32_TABLE_SIZE; i++) {
		uint32_t c = i;

		for (uint32_t k = 0U; k < BYTE_BITS; k++) {
			c = ((c & 1U) != 0U) ? ((c >> 1U) ^ CRC32_POLY_REFLECTED) : (c >> 1U);
		}
		crc32_table[i] = c;
	}
}

/* CRC-32 (IEEE), one table lookup per byte */
static uint32_t crc32(const uint8_t *data, size_t len)
{
	uint32_t crc = UINT32_MAX;

	for (size_t i = 0U; i < len; i++) {
		crc = (crc >> BYTE_BITS) ^ crc32_table[(crc ^ data[i]) & BYTE_MASK];
	}

	return ~crc;
}

/*
 * Packs n dump words into out, returns the payload size. width is the sample
 * width of the default layout, or 0 for the layout set with IQFMT. out may be
 * src: every word is read before its bytes are overwritten.
 */
static size_t pack(const uint32_t *src, uint32_t n, uint32_t fmt, uint32_t width, uint8_t *out)
{
	uint32_t w = ((width == 0U) && iq_format_is_default()) ? RF_DUMP_WIDTH_80M : width;
	uint8_t *p = out;

	if (fmt == FMT_IQ8) {
		for (uint32_t j = 0U; j < n; j++) {
			uint16_t iq = (w != 0U) ? iq_format_iq8_width(src[j], w)
						: iq_format_iq8(src[j]);

			p[IQ8_BYTES * j] = (uint8_t)(iq & BYTE_MASK);
			p[(IQ8_BYTES * j) + 1U] = (uint8_t)(iq >> BYTE_BITS);
		}
		return (size_t)n * IQ8_BYTES;
	}

	if (fmt == FMT_IQ10) {
		for (uint32_t j = 0U; j < n; j += 2U) {
			uint32_t w0 = src[j];
			uint32_t w1 = ((j + 1U) < n) ? src[j + 1U] : 0U;
			uint32_t a = (w != 0U) ? iq_format_iq20_width(w0, w) : iq20(w0);
			uint32_t b = (w != 0U) ? iq_format_iq20_width(w1, w) : iq20(w1);

			p[0] = (uint8_t)(a & BYTE_MASK);
			p[1] = (uint8_t)((a >> BYTE_BITS) & BYTE_MASK);
			p[2] = (uint8_t)(((a >> TWO_BYTE_BITS) | (b << NIBBLE_BITS)) & BYTE_MASK);
			if ((j + 1U) < n) {
				p[3] = (uint8_t)((b >> NIBBLE_BITS) & BYTE_MASK);
				p[4] = (uint8_t)((b >> TWELVE_BITS) & BYTE_MASK);
			}
			p = &p[IQ10_PAIR_BYTES];
		}
		return ((size_t)(n / 2U) * IQ10_PAIR_BYTES) +
		       (((n % 2U) != 0U) ? IQ10_TAIL_BYTES : 0U);
	}

	for (uint32_t j = 0U; j < n; j++) {
		uint32_t word = src[j];

		p[BYTES_PER_WORD * j] = (uint8_t)(word & BYTE_MASK);
		p[(BYTES_PER_WORD * j) + 1U] = (uint8_t)((word >> BYTE_BITS) & BYTE_MASK);
		p[(BYTES_PER_WORD * j) + 2U] = (uint8_t)((word >> TWO_BYTE_BITS) & BYTE_MASK);
		p[(BYTES_PER_WORD * j) + 3U] = (uint8_t)(word >> (TWO_BYTE_BITS + BYTE_BITS));
	}

	return (size_t)n * BYTES_PER_WORD;
}

static bool rate_valid(uint32_t rate)
{
	return (rate < RATE_COUNT) && (rates[rate].hz != 0U);
}

/* Runs with the dump window claimed */
static bool capture_owned(uint32_t n, uint32_t fmt, uint32_t rate)
{
	const struct rate *r = &rates[rate];
	/* The native rate keeps the DUMPSEL and IQFMT bring-up settings */
	uint32_t sel = (rate == RF_RATE_INDEX) ? rf_dump_sel_get() : r->dump_sel;
	uint32_t width = (rate == RF_RATE_INDEX) ? 0U : r->width;
	uint32_t words = n;
	/* Non-cached view, idle once the finite dump completed */
	volatile uint32_t *buf = rf_snapshot_buf();
	uint8_t *out = (uint8_t *)snap_out;
	const uint32_t *src;
	char hdr[REPLY_SIZE];
	uint32_t t0;
	uint32_t elapsed;
	size_t bytes;
	bool done;

	if (r->resample != NULL) {
		/* The 16 MS/s filter reaches a few words past the window edges */
		words = MIN(resample_input_words(r->resample, n), RF_WINDOW_WORDS - CANARY_WORDS);
	}

	for (uint32_t s = 0U; agc_enabled && (s < AGC_MAX_STEPS); s++) {
		if (rf_agc_step()) {
			break;
		}
	}

	/* The engine writes in order: an overwritten last word means a full dump */
	buf[words - 1U] = FILL_PATTERN;
	for (uint32_t j = 0U; j < CANARY_WORDS; j++) {
		buf[words + j] = CANARY_PATTERN ^ j;
	}

	t0 = k_cycle_get_32();
	done = rf_snapshot_sel(words, sel);
	elapsed = k_cyc_to_us_floor32(k_cycle_get_32() - t0);

	if (!done) {
		reply("ERR capture_timeout\n");
		return false;
	}
	if (buf[words - 1U] == FILL_PATTERN) {
		reply("ERR capture_timeout\n");
		return false;
	}
	for (uint32_t j = 0U; j < CANARY_WORDS; j++) {
		if (buf[words + j] != (CANARY_PATTERN ^ j)) {
			reply("ERR capture_overrun\n");
			return false;
		}
	}

	src = rf_snapshot_cached(words);
	if (r->resample != NULL) {
		resample_run(r->resample, src, words, snap_out, n, width);
		src = snap_out;
	}
	bytes = pack(src, n, fmt, width, out);
	(void)snprintf(hdr, sizeof(hdr), "DATA %u %08x %u\n", n, crc32(out, bytes), elapsed);
	reply(hdr);
	send(out, bytes);

	return true;
}

static bool capture(uint32_t n, uint32_t fmt, uint32_t rate)
{
	bool ok;

	if (!capture_uart_claim()) {
		reply("ERR busy\n");
		return false;
	}
	ok = capture_owned(n, fmt, rate);
	capture_uart_release();

	return ok;
}

/* CAP/CAP16/CAP20 <samples> <rate-index> */
static bool capture_cmd(const char *args, uint32_t fmt)
{
	uint32_t v[MAX_ARGS];

	if ((parse_args(args, v, MAX_ARGS) != 2) || (v[0] < IQ_MIN_WORDS) || (v[0] > IQ_WORDS) ||
	    !rate_valid(v[1])) {
		reply("ERR command\n");
		return false;
	}

	return capture(v[0], fmt, v[1]);
}

/* RXRUN <samples> <rate-index> <repeats> <format> */
static void rxrun_cmd(const char *args)
{
	uint32_t v[MAX_ARGS];
	bool ok = true;

	if ((parse_args(args, v, MAX_ARGS) != 4) || (v[0] < IQ_MIN_WORDS) || (v[0] > IQ_WORDS) ||
	    !rate_valid(v[1]) || (v[2] == 0U) || (v[2] > RXRUN_MAX_REPEATS) ||
	    ((v[3] != FMT_IQ8) && (v[3] != FMT_IQ10))) {
		reply("ERR command\n");
		return;
	}

	for (uint32_t j = 0U; (j < v[2]) && ok; j++) {
		ok = capture(v[0], v[3], v[1]);
		k_yield();
	}
	if (ok) {
		reply("END\n");
	}
}

/* Parses space separated decimal or 0x-prefixed values */
static int parse_nums(const char *str, uint32_t *args, size_t max_args)
{
	const char *p = str;
	size_t count = 0U;

	while (*p != '\0') {
		char *end;
		unsigned long val;

		if ((*p < '0') || (*p > '9') || (count == max_args)) {
			return -1;
		}
		errno = 0;
		val = strtoul(p, &end, HEX_OR_DECIMAL);
		if ((errno != 0) || ((*end != ' ') && (*end != '\0'))) {
			return -1;
		}
		args[count] = (uint32_t)val;
		count++;
		p = (*end == ' ') ? (end + 1) : end;
	}

	return (int)count;
}

/* Bring-up access to memory and RF registers */
static bool debug_command(const char *line)
{
	char buf[REPLY_SIZE];
	uint32_t v[MAX_ARGS];
	uint32_t val;

	if (strncmp(line, "PEEK ", strlen("PEEK ")) == 0) {
		if ((parse_nums(&line[strlen("PEEK ")], v, MAX_ARGS) == 2) &&
		    ((v[0] % BYTES_PER_WORD) == 0U) && (v[1] > 0U) && (v[1] <= PEEK_MAX_WORDS)) {
			const uint8_t *mem = (const uint8_t *)(uintptr_t)v[0];
			size_t bytes = (size_t)v[1] * BYTES_PER_WORD;

			(void)snprintf(buf, sizeof(buf), "DATA %u %08x 0\n", v[1],
				       crc32(mem, bytes));
			reply(buf);
			send(mem, bytes);
		} else {
			reply("ERR command\n");
		}
	} else if (strncmp(line, "POKE ", strlen("POKE ")) == 0) {
		if ((parse_nums(&line[strlen("POKE ")], v, MAX_ARGS) == 2) &&
		    ((v[0] % BYTES_PER_WORD) == 0U)) {
			*(volatile uint32_t *)(uintptr_t)v[0] = v[1];
			reply("OK\n");
		} else {
			reply("ERR command\n");
		}
	} else if (strncmp(line, "FILL ", strlen("FILL ")) == 0) {
		if ((parse_nums(&line[strlen("FILL ")], v, MAX_ARGS) == 3) &&
		    ((v[0] % BYTES_PER_WORD) == 0U) && (v[1] <= PEEK_MAX_WORDS)) {
			volatile uint32_t *mem = (volatile uint32_t *)(uintptr_t)v[0];

			for (uint32_t i = 0U; i < v[1]; i++) {
				mem[i] = v[2];
			}
			reply("OK\n");
		} else {
			reply("ERR command\n");
		}
	} else if (strncmp(line, "CW ", strlen("CW ")) == 0) {
		if ((parse_nums(&line[strlen("CW ")], v, MAX_ARGS) == 2) &&
		    capture_uart_claim()) {
			/* Power is in dBm, 0..20 */
			rf_cw_start(v[0], (int32_t)v[1]);
			capture_uart_release();
			reply("OK\n");
		} else {
			reply("ERR command\n");
		}
	} else if (strcmp(line, "CWSTOP") == 0) {
		if (capture_uart_claim()) {
			rf_cw_stop();
			capture_uart_release();
			reply("OK\n");
		} else {
			reply("ERR busy\n");
		}
	} else if (strncmp(line, "RFRD ", strlen("RFRD ")) == 0) {
		if ((parse_nums(&line[strlen("RFRD ")], v, MAX_ARGS) == 1) &&
		    rf_reg_read(v[0], &val)) {
			(void)snprintf(buf, sizeof(buf), "RF %03x %08x\n", v[0], val);
			reply(buf);
		} else {
			reply("ERR command\n");
		}
	} else if (strncmp(line, "RFWR ", strlen("RFWR ")) == 0) {
		if ((parse_nums(&line[strlen("RFWR ")], v, MAX_ARGS) == 2) &&
		    rf_reg_write(v[0], v[1])) {
			reply("OK\n");
		} else {
			reply("ERR command\n");
		}
	} else {
		return false;
	}

	return true;
}

/* Bring-up commands for the continuous ring */
static bool ring_command(const char *line)
{
	char buf[REPLY_SIZE];
	uint32_t v[MAX_ARGS];
	uint32_t a;
	uint32_t b;
	uint32_t c;

	if (strcmp(line, "RINGPROBE") == 0) {
		if (!capture_uart_claim()) {
			reply("ERR busy\n");
		} else {
			int len = rf_ring_probe(probe_reply, sizeof(probe_reply));

			capture_uart_release();
			reply((len > 0) ? probe_reply : "ERR probe\n");
		}
	} else if (strncmp(line, "RINGTRACE ", strlen("RINGTRACE ")) == 0) {
		uint32_t t[MAX_ARGS];

		if ((parse_args(&line[strlen("RINGTRACE ")], t, MAX_ARGS) != 2) ||
		    (t[0] == 0U) || (t[0] > TRACE_MAX_SAMPLES)) {
			reply("ERR command\n");
		} else if (!capture_uart_claim()) {
			reply("ERR busy\n");
		} else {
			int len = rf_ring_trace(probe_reply, sizeof(probe_reply), t[0], t[1]);

			capture_uart_release();
			reply((len > 0) ? probe_reply : "ERR probe\n");
			reply("END\n");
		}
	} else if (strcmp(line, "RINGMODE?") == 0) {
		rf_ring_mode_get(&a, &b);
		(void)snprintf(buf, sizeof(buf), "RINGMODE %u %u\n", a, b);
		reply(buf);
	} else if (strncmp(line, "RINGMODE ", strlen("RINGMODE ")) == 0) {
		if (parse_args(&line[strlen("RINGMODE ")], v, MAX_ARGS) == 2) {
			rf_ring_mode_set(v[0], v[1]);
			reply("OK\n");
		} else {
			reply("ERR command\n");
		}
	} else if (strcmp(line, "RINGPTR?") == 0) {
		rf_ring_ptr_get(&a, &b, &c);
		(void)snprintf(buf, sizeof(buf), "RINGPTR %u %u %u\n", a, b, c);
		reply(buf);
	} else if (strncmp(line, "RINGPTR ", strlen("RINGPTR ")) == 0) {
		if (parse_args(&line[strlen("RINGPTR ")], v, MAX_ARGS) == 3) {
			rf_ring_ptr_set(v[0], v[1], v[2]);
			rf_ring_ptr_get(&a, &b, &c);
			reply(((a == v[0]) && (b == v[1]) && (c == v[2])) ? "OK\n"
									  : "ERR command\n");
		} else {
			reply("ERR command\n");
		}
	} else if (strcmp(line, "RING?") == 0) {
		(void)snprintf(buf, sizeof(buf), "RING %u %u %u %u\n", rf_ring_running() ? 1U : 0U,
			       (uint32_t)dump_ring_abs_write(), dump_ring_source_chunk_index(),
			       dump_ring_dropped_chunks());
		reply(buf);
	} else {
		return false;
	}

	return true;
}

static void handle_command(const char *line)
{
	char buf[REPLY_SIZE];
	uint32_t v[MAX_ARGS];

	if (line[0] == '\0') {
		return;
	}
	if (ring_command(line) || debug_command(line)) {
		return;
	}

	if (strcmp(line, "TRANSPORT?") == 0) {
		if (active_port->usb) {
			reply("TRANSPORT USB 0\n");
		} else {
			(void)snprintf(buf, sizeof(buf), "TRANSPORT UART %u\n", UART_BAUD);
			reply(buf);
		}
	} else if (strncmp(line, "SYNC ", strlen("SYNC ")) == 0) {
		const char *nonce = &line[strlen("SYNC ")];

		if ((nonce[0] != '\0') && (strspn(nonce, "0123456789") == strlen(nonce))) {
			(void)snprintf(buf, sizeof(buf), "SYNC %s\n", nonce);
			reply(buf);
		} else {
			reply("ERR command\n");
		}
	} else if (strcmp(line, "INFO") == 0) {
		(void)snprintf(buf, sizeof(buf), SDR_IDENTITY " %u burst %u\n", SDR_PROTOCOL,
			       IQ_WORDS);
		reply(buf);
	} else if (strcmp(line, "CAPS") == 0) {
		reply("CAPS RXLIMITS SERIALLEASE DUALSERIAL GAIN HWAGC IQ8 TUNEEXT\n");
	} else if (strcmp(line, "RANGE?") == 0) {
		(void)snprintf(buf, sizeof(buf), "RANGE %u %u 1\n", RF_FREQ_MIN_MHZ,
			       RF_FREQ_MAX_MHZ);
		reply(buf);
	} else if (strcmp(line, "LIMITS?") == 0) {
		int len = snprintf(buf, sizeof(buf),
				   "LIMITS {\"gain\":[%u,%u,1],\"bandwidth\":null,\"rates\":[", 0U,
				   RF_GAIN_STEPS - 1U);
		const char *sep = "";

		for (uint32_t i = 0U; i < RATE_COUNT; i++) {
			if (rates[i].hz != 0U) {
				len += snprintf(&buf[len], sizeof(buf) - (size_t)len, "%s%u", sep,
						rates[i].hz);
				sep = ",";
			}
		}
		(void)snprintf(&buf[len], sizeof(buf) - (size_t)len, "],\"bits\":[8,10]}\n");
		reply(buf);
	} else if (strcmp(line, "GAIN?") == 0) {
		(void)snprintf(buf, sizeof(buf), "GAIN %s %u\n",
			       agc_enabled ? "HARDWARE" : "MANUAL", rf_gain_get());
		reply(buf);
	} else if (strcmp(line, "GAIN HARDWARE") == 0) {
		agc_enabled = true;
		reply("OK\n");
	} else if (strncmp(line, "GAIN MANUAL ", strlen("GAIN MANUAL ")) == 0) {
		if ((parse_args(&line[strlen("GAIN MANUAL ")], v, MAX_ARGS) != 1) ||
		    (v[0] >= RF_GAIN_STEPS)) {
			reply("ERR gain_args\n");
		} else if (!capture_uart_claim()) {
			reply("ERR busy\n");
		} else {
			agc_enabled = false;
			rf_gain_set(v[0]);
			capture_uart_release();
			reply("OK\n");
		}
	} else if (strncmp(line, "FREQ ", strlen("FREQ ")) == 0) {
		if ((parse_args(&line[strlen("FREQ ")], v, MAX_ARGS) != 1) ||
		    (v[0] < RF_FREQ_MIN_MHZ) || (v[0] > RF_FREQ_MAX_MHZ)) {
			reply("ERR command\n");
		} else if (!capture_uart_claim()) {
			reply("ERR busy\n");
		} else {
			rf_tune(v[0]);
			capture_uart_release();
			reply("OK\n");
		}
	} else if (strncmp(line, "CAP16 ", strlen("CAP16 ")) == 0) {
		(void)capture_cmd(&line[strlen("CAP16 ")], FMT_IQ8);
	} else if (strncmp(line, "CAP20 ", strlen("CAP20 ")) == 0) {
		(void)capture_cmd(&line[strlen("CAP20 ")], FMT_IQ10);
	} else if (strncmp(line, "CAP ", strlen("CAP ")) == 0) {
		(void)capture_cmd(&line[strlen("CAP ")], FMT_RAW);
	} else if (strncmp(line, "RXRUN ", strlen("RXRUN ")) == 0) {
		rxrun_cmd(&line[strlen("RXRUN ")]);
	} else if (strcmp(line, "DUMPSEL?") == 0) {
		(void)snprintf(buf, sizeof(buf), "DUMPSEL %u\n", rf_dump_sel_get());
		reply(buf);
	} else if (strncmp(line, "DUMPSEL ", strlen("DUMPSEL ")) == 0) {
		if ((parse_args(&line[strlen("DUMPSEL ")], v, MAX_ARGS) == 1) &&
		    (v[0] <= RF_DUMP_SEL_MAX)) {
			rf_dump_sel_set(v[0]);
			reply("OK\n");
		} else {
			reply("ERR command\n");
		}
	} else if (strcmp(line, "IQFMT?") == 0) {
		uint32_t i_lsb;
		uint32_t q_lsb;
		uint32_t width;

		iq_format_get(&i_lsb, &q_lsb, &width);
		(void)snprintf(buf, sizeof(buf), "IQFMT %u %u %u\n", i_lsb, q_lsb, width);
		reply(buf);
	} else if (strncmp(line, "IQFMT ", strlen("IQFMT ")) == 0) {
		if ((parse_args(&line[strlen("IQFMT ")], v, MAX_ARGS) == 3) &&
		    iq_format_set(v[0], v[1], v[2])) {
			reply("OK\n");
		} else {
			reply("ERR command\n");
		}
	} else {
		reply("ERR command\n");
	}
}

int main(void)
{
	struct port *owner = NULL;
	k_timepoint_t lease = sys_timepoint_calc(K_NO_WAIT);
	int ret;

	crc32_init();
	ret = port_init(&ports[PORT_UART], uart_isr);
	if (ret != 0) {
		return ret;
	}

	ret = rf_init();
	if (ret != 0) {
		printk("bflb_rf_init failed: %d\n", ret);
		return ret;
	}

	dump_ring_init();
	ret = iq_usb_init();
	if (ret != 0) {
		printk("USB init failed: %d\n", ret);
	} else {
		ret = port_init(&ports[PORT_CDC], cdc_isr);
		if (ret != 0) {
			printk("CDC-ACM init failed: %d\n", ret);
		}
	}

	for (;;) {
		bool idle = true;

		for (uint32_t i = 0U; i < PORT_COUNT; i++) {
			ret = poll_line(&ports[i]);
			if (ret == 0) {
				continue;
			}
			idle = false;
			active_port = &ports[i];
			if (sys_timepoint_expired(lease)) {
				owner = NULL;
			}
			if ((owner != NULL) && (owner != active_port)) {
				reply("ERR busy\n");
				continue;
			}
			if (ret < 0) {
				reply("ERR command_length\n");
				continue;
			}
			owner = active_port;
			if (strcmp(active_port->line, "RELEASE") == 0) {
				reply("OK\n");
				owner = NULL;
			} else {
				handle_command(active_port->line);
			}
			/* The lease covers the whole binary transfer */
			lease = sys_timepoint_calc(K_MSEC(LEASE_TIMEOUT_MS));
		}
		if (idle) {
			(void)k_sem_take(&rx_sem, K_FOREVER);
		}
	}

	return 0;
}
