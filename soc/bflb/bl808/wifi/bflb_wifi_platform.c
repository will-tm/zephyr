/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BL808 WiFi platform support: GLB clock ungates, PHY/RF calibration via
 * the wl API, the firmware scheduler task and the OS-level symbols the
 * BL606P wifi4 MAC firmware blob imports.
 *
 * Unlike BL602 the blob's wifi_main() performs the complete MAC/PHY
 * bring-up itself (RF calibration through phyrf's rf_init, mpif clock,
 * sysctrl, IPC and kernel init) before entering its scheduler loop, so it
 * runs unwrapped as a plain Zephyr thread.
 */

#include <stdarg.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#include <bflb_soc.h>
#include <glb_reg.h>

#include <bouffalolab/bl808/wifi/utils_list.h>

LOG_MODULE_REGISTER(bflb_wifi_plat, CONFIG_WIFI_LOG_LEVEL);

/* GLB clock gates for the WiFi subsystem (register offsets in glb_reg.h). */
#define GLB_CGEN_CFG0_WIFI_PHY    BIT(7)
#define GLB_CGEN_CFG2_WIFI        BIT(4)
#define GLB_WIFI_CFG0_OFFSET      0x3B0U
#define GLB_WIFI_MAC_CORE_DIV_MSK 0xFU
#define GLB_WIFI_MAC_CORE_DIV_1   1U

/* The blob's wifi_main has no ready callback; give its RF calibration and
 * kernel init a fixed head start before the host starts sending commands
 * (the vendor driver waits 1 s).
 */
#define BFLB_WIFI_FW_STARTUP_MS 1000

#define BFLB_CRC32_STATE_INIT 0xFFFFFFFFU

/* CRC32 stream (used by the blob for beacon integrity checks). */
struct utils_crc32_stream {
	uint32_t state;
};

/* struct wifi_bt_coex_ctx in the coex blob member, zero-initialised
 * (28 bytes); normally provided by the BT controller blob.
 */
#define BFLB_COEX_CTX_WORDS 7U

/* Entry points into the firmware blob. */
extern void wifi_main(void *arg);
extern int bflb_rf_init(void);

/* Linker anchors the blob walks for its static configuration entries. */
uint8_t _ld_bl_static_cfg_entry_start[0] Z_GENERIC_SECTION(.bl_static_cfg_entry);
uint8_t _ld_bl_static_cfg_entry_end[0] Z_GENERIC_SECTION(.bl_static_cfg_entry);

/* BT/WiFi coexistence context the blob's coex module references; the BT
 * controller blob is not linked, a zeroed context disables coexistence.
 */
uint32_t coex_timing_control_ctx[BFLB_COEX_CTX_WORDS];

static K_KERNEL_STACK_DEFINE(wifi_task_stack, CONFIG_BFLB_WIFI_FW_TASK_STACK_SIZE);
static struct k_thread wifi_task_thread;

static void bflb_wifi_clock_enable(void);
static void wifi_task_entry(void *p1, void *p2, void *p3);

/* Ungate the WiFi subsystem clocks: PHY (CGEN_CFG0), MAC (CGEN_CFG2) and
 * the MAC core divider.
 */
static void bflb_wifi_clock_enable(void)
{
	uint32_t v;

	v = sys_read32(GLB_BASE + GLB_CGEN_CFG0_OFFSET);
	v |= GLB_CGEN_CFG0_WIFI_PHY;
	sys_write32(v, GLB_BASE + GLB_CGEN_CFG0_OFFSET);

	v = sys_read32(GLB_BASE + GLB_CGEN_CFG2_OFFSET);
	v |= GLB_CGEN_CFG2_WIFI;
	sys_write32(v, GLB_BASE + GLB_CGEN_CFG2_OFFSET);

	v = sys_read32(GLB_BASE + GLB_WIFI_CFG0_OFFSET);
	v = (v & ~GLB_WIFI_MAC_CORE_DIV_MSK) | GLB_WIFI_MAC_CORE_DIV_1;
	sys_write32(v, GLB_BASE + GLB_WIFI_CFG0_OFFSET);
}

/* WiFi firmware thread: the blob's wifi_main() is a cooperative scheduler
 * that never returns.  Cooperative priority keeps the PHY calibration it
 * runs at startup from being preempted mid register-poll.
 */
static void wifi_task_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	wifi_main(NULL);
}

int bflb_wifi_hw_init(void)
{
	int ret;

	bflb_wifi_clock_enable();

	ret = bflb_rf_init();
	if (ret != 0) {
		LOG_ERR("bflb_rf_init failed: %d", ret);
		return ret;
	}

	return 0;
}

void wifi_task_create(void)
{
	k_thread_create(&wifi_task_thread, wifi_task_stack, K_THREAD_STACK_SIZEOF(wifi_task_stack),
			wifi_task_entry, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_BFLB_WIFI_FW_TASK_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&wifi_task_thread, "bl808_wifi_fw");
}

int wifi_task_wait_ready(k_timeout_t timeout)
{
	ARG_UNUSED(timeout);

	k_msleep(BFLB_WIFI_FW_STARTUP_MS);
	return 0;
}

/* Drop the blob's bare printf output (PHY calibration and power-save
 * traces) without disabling printf for the rest of the image: suppress
 * only when called from the firmware thread, where UART latency would
 * break the PHY calibration timing.
 */
int __wrap_printf(const char *fmt, ...)
{
	va_list ap;
	int r;

	if (k_current_get() == &wifi_task_thread) {
		return 0;
	}

	va_start(ap, fmt);
	r = vprintf(fmt, ap);
	va_end(ap);

	return r;
}

/* Timing primitive the blobs import. */
void arch_delay_us(uint32_t us)
{
	k_busy_wait(us);
}

/* Host symbols the firmware blob imports but that need no work here. */
void bl_main_event_handle(int param, void *tx_fc_field)
{
	ARG_UNUSED(param);
	ARG_UNUSED(tx_fc_field);
}

int bl_supplicant_init(void *arg)
{
	ARG_UNUSED(arg);
	return 0;
}

void bl_utils_dump(void)
{
	/* Stub */
}

int bl_printf(const char *fmt, ...)
{
	ARG_UNUSED(fmt);
	return 0;
}

uint32_t bl_os_clock_gettime_ms(void)
{
	return (uint32_t)k_uptime_get();
}

/* Antenna/FEM control GPIOs -- not wired on supported boards. */
void bl_gpio_enable_output(uint8_t pin, uint8_t pullup, uint8_t pulldown)
{
	ARG_UNUSED(pin);
	ARG_UNUSED(pullup);
	ARG_UNUSED(pulldown);
}

void bl_gpio_output_set(uint8_t pin, uint8_t value)
{
	ARG_UNUSED(pin);
	ARG_UNUSED(value);
}

/* Singly-linked list used by the firmware TX descriptor queues. */
void utils_list_push_back(struct utils_list *l, struct utils_list_hdr *h)
{
	h->next = NULL;
	if (l->last != NULL) {
		((struct utils_list_hdr *)l->last)->next = h;
	} else {
		l->first = h;
	}
	l->last = h;
}

struct utils_list_hdr *utils_list_pop_front(struct utils_list *l)
{
	struct utils_list_hdr *h = (struct utils_list_hdr *)l->first;

	if (h != NULL) {
		l->first = h->next;
		if (l->first == NULL) {
			l->last = NULL;
		}
	}
	return h;
}

void utils_list_init(struct utils_list *l)
{
	l->first = NULL;
	l->last = NULL;
}

int utils_crc32_stream_init(struct utils_crc32_stream *s)
{
	if (s != NULL) {
		s->state = BFLB_CRC32_STATE_INIT;
	}
	return 0;
}

int utils_crc32_stream_feed_block(struct utils_crc32_stream *s, const void *data, uint32_t len)
{
	if (s == NULL) {
		return -EINVAL;
	}
	s->state = crc32_ieee_update(s->state, data, len);
	return 0;
}

/* Single struct-ptr signature: callers only set a0, a two-arg form would
 * read garbage in a1 and trash random memory.
 */
uint32_t utils_crc32_stream_results(struct utils_crc32_stream *s)
{
	return (s != NULL) ? ~s->state : BFLB_CRC32_STATE_INIT;
}

/* TLV util (RF parameters) -- no RF blob TLV in flash. */
int utils_tlv_bl_unpack_auto(void *tlv, uint32_t len, void *out)
{
	ARG_UNUSED(tlv);
	ARG_UNUSED(len);
	ARG_UNUSED(out);
	return 0;
}

/* HOSAL power-management hooks -- all no-op. */
int wifi_hosal_rf_turn_on(void *a)
{
	ARG_UNUSED(a);
	return 0;
}

int wifi_hosal_rf_turn_off(void *a)
{
	ARG_UNUSED(a);
	return 0;
}

int wifi_hosal_pm_state_run(void)
{
	return 0;
}

int wifi_hosal_pm_post_event(int ev, uint32_t code, uint32_t *r)
{
	ARG_UNUSED(ev);
	ARG_UNUSED(code);
	ARG_UNUSED(r);
	return 0;
}

int wifi_hosal_pm_event_register(int ev, uint32_t code, uint32_t cap_bit, uint16_t prio, void *ops,
				 void *arg, int enable)
{
	ARG_UNUSED(ev);
	ARG_UNUSED(code);
	ARG_UNUSED(cap_bit);
	ARG_UNUSED(prio);
	ARG_UNUSED(ops);
	ARG_UNUSED(arg);
	ARG_UNUSED(enable);
	return 0;
}
