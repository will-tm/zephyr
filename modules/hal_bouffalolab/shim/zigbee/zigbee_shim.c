/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zigbee stack glue for Zephyr. Provides the platform functions that
 * libzbstack.a / libmacphy154_bl702l.a expect from the BL IoT SDK.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#include <stdint.h>
#include <string.h>

LOG_MODULE_REGISTER(zigbee_shim, CONFIG_BFLB_ZIGBEE_LOG_LEVEL);

#define FLASH_SECTOR_SIZE      4096U
#define ZIGBEE_SHIM_INIT_PRIO  90

#define AON_BASE               0x4000F800UL
#define AON_XTAL_CFG_OFFSET    0x0034U
#define AON_XTAL_CAP_MASK      0x3FU
#define AON_XTAL_CAP_IN_SHIFT  16U
#define AON_XTAL_CAP_OUT_SHIFT 22U

#define US_PER_MS              1000U

static const struct device *flash_dev;

static int flash_init(void)
{
	if (flash_dev != NULL) {
		return 0;
	}
	flash_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
	if (!device_is_ready(flash_dev)) {
		LOG_ERR("flash device not ready");
		return -ENODEV;
	}
	return 0;
}

int bl_flash_read(uint32_t addr, uint8_t *dst, int len)
{
	if (flash_init() != 0) {
		return -1;
	}
	int ret = flash_read(flash_dev, addr, dst, len);

	if (ret != 0) {
		LOG_ERR("flash_read(0x%x, %d) = %d", addr, len, ret);
	}
	return ret;
}

int bl_flash_write(uint32_t addr, const uint8_t *src, int len)
{
	if (flash_init() != 0) {
		return -1;
	}
	return flash_write(flash_dev, addr, src, len);
}

int bl_flash_erase(uint32_t addr, int len)
{
	if (flash_init() != 0) {
		return -1;
	}
	LOG_INF("flash_erase(0x%x, %d)", addr, len);
	if (len < FLASH_SECTOR_SIZE) {
		len = FLASH_SECTOR_SIZE;
	}
	return flash_erase(flash_dev, addr, len);
}

int bl_flash_read_byxip(uint32_t addr, uint8_t *dst, int len)
{
	return bl_flash_read(addr, dst, len);
}

typedef void (*bl_irq_handler_t)(void);
static bl_irq_handler_t irq_handlers[CONFIG_NUM_IRQS];

static void generic_irq_wrapper(const void *arg)
{
	int irqn = (int)(uintptr_t)arg;

	if (irqn < CONFIG_NUM_IRQS && irq_handlers[irqn] != NULL) {
		irq_handlers[irqn]();
	}
}

int bl_irq_register(int irqn, void *handler)
{
	if (irqn >= CONFIG_NUM_IRQS) {
		return -1;
	}
	LOG_INF("irq_register %d handler=%p", irqn, handler);
	irq_handlers[irqn] = (bl_irq_handler_t)handler;
	irq_connect_dynamic(irqn, 0, generic_irq_wrapper,
			    (const void *)(uintptr_t)irqn, 0);
	return 0;
}

int bl_irq_unregister(int irqn, void *handler)
{
	ARG_UNUSED(handler);
	if (irqn < CONFIG_NUM_IRQS) {
		irq_handlers[irqn] = NULL;
	}
	irq_disable(irqn);
	return 0;
}

int bl_irq_enable(int irqn)
{
	irq_enable(irqn);
	return 0;
}

int bl_irq_disable(int irqn)
{
	irq_disable(irqn);
	return 0;
}

int bl_irq_pending_clear(int irqn)
{
	ARG_UNUSED(irqn);
	return 0;
}

uint32_t bl_timer_now_us(void)
{
	return (uint32_t)k_cyc_to_us_floor64(k_cycle_get_32());
}

uint32_t bl_sec_get_random_word(void)
{
	uint32_t val;
	const struct device *entropy = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));

	if (device_is_ready(entropy)) {
		entropy_get_entropy(entropy, (uint8_t *)&val, sizeof(val));
	} else {
		val = k_cycle_get_32();
	}
	return val;
}

int bl_wireless_mac_addr_get(uint8_t mac[6])
{
	ssize_t len = hwinfo_get_device_id(mac, 6);

	if (len < 6) {
		memset(mac, 0xFF, 6);
		return -1;
	}
	return 0;
}

int bl_wireless_default_tx_power_get(void)
{
	return 10;
}

void bl_wdt_init(int ms)
{
	ARG_UNUSED(ms);
}

void bl_wdt_feed(void)
{
}

#define AON_BASE 0x4000F800UL

uint8_t AON_Get_Xtal_CapCode(void)
{
	volatile uint32_t *reg = (volatile uint32_t *)(AON_BASE + AON_XTAL_CFG_OFFSET);

	return (uint8_t)((*reg >> AON_XTAL_CAP_IN_SHIFT) & AON_XTAL_CAP_MASK);
}

void AON_Set_Xtal_CapCode(uint8_t cap_in, uint8_t cap_out)
{
	volatile uint32_t *reg = (volatile uint32_t *)(AON_BASE + AON_XTAL_CFG_OFFSET);
	uint32_t val = *reg;

	val &= ~(AON_XTAL_CAP_MASK << AON_XTAL_CAP_IN_SHIFT);
	val |= ((uint32_t)(cap_in & AON_XTAL_CAP_MASK)) << AON_XTAL_CAP_IN_SHIFT;
	val &= ~(AON_XTAL_CAP_MASK << AON_XTAL_CAP_OUT_SHIFT);
	val |= ((uint32_t)(cap_out & AON_XTAL_CAP_MASK)) << AON_XTAL_CAP_OUT_SHIFT;
	*reg = val;
}

int hal_boot2_partition_addr_active(const char *name, uint32_t *addr,
				    uint32_t *size)
{
	ARG_UNUSED(name);
	*addr = 0;
	*size = 0;
	return -1;
}

/* rf_get_rf_state and rf_init_lp provided by libbl702l_rf.a */

void BL702L_Delay_US(uint32_t cnt)
{
	k_busy_wait(cnt);
}

void BL702L_Delay_MS(uint32_t cnt)
{
	k_busy_wait(cnt * US_PER_MS);
}

void arch_memcpy4(uint32_t *dst, const uint32_t *src, uint32_t n)
{
	memcpy(dst, src, n);
}

void *bl_irq_handler_get(int irqn)
{
	if (irqn < CONFIG_NUM_IRQS) {
		return (void *)irq_handlers[irqn];
	}
	return NULL;
}

uint32_t bl_irq_save(void)
{
	return irq_lock();
}

void bl_irq_restore(uint32_t flags)
{
	irq_unlock(flags);
}

/* macphy_hal_init, macphy_get_*, macphy_set_*, registerPhyIsr,
 * unregisterPhyIsr provided by zb_macphy_api.o in libmacphy154_bl702l.a
 */

static struct k_sem zigbee_task_sem;
static volatile void (*zigbee_pending_func)(void *);
static volatile void *zigbee_pending_arg;

void wakeupZigbeeTask(void)
{
	k_sem_give(&zigbee_task_sem);
}

void executeOnZigBeeTask(void (*func)(void *), void *arg)
{
	LOG_INF("executeOnZigBeeTask func=%p arg=%p", func, arg);
	zigbee_pending_func = func;
	zigbee_pending_arg = arg;
	wakeupZigbeeTask();
	func(arg);
}

int zb_hasEnoughMemory(void)
{
	return 1;
}

/* lowFreeHeapFailureCount provided by libzbstack.a */

int32_t TrapNetCounter;

void factoryInit(void)
{
}

void zb_initFlashCache(void)
{
}

void zb_freeFlashCache(void)
{
}

void zb_resetBtrCacheTable(void)
{
}

void zb_zsedCancelParentMaintenanceTimer(void)
{
}

#define ZB_TIMER_US_PER_TICK 16
#define ZB_TIMER_CH_NUM      6

typedef void (*zb_timer_cb_t)(void);

static struct k_timer zb_timers[ZB_TIMER_CH_NUM];
static zb_timer_cb_t zb_timer_cbs[ZB_TIMER_CH_NUM];
static uint32_t zb_timer_base_ticks;

static void zb_timer_expiry(struct k_timer *timer)
{
	int ch = (int)(timer - zb_timers);

	if (ch >= 0 && ch < ZB_TIMER_CH_NUM && zb_timer_cbs[ch]) {
		zb_timer_cbs[ch]();
	}
}

static int zb_timer_inited;

static void zb_timer_ensure_init(void)
{
	if (zb_timer_inited) {
		return;
	}
	for (int i = 0; i < ZB_TIMER_CH_NUM; i++) {
		k_timer_init(&zb_timers[i], zb_timer_expiry, NULL);
	}
	zb_timer_inited = 1;
}

uint32_t zb_timer_get_current_time(void)
{
	return (uint32_t)(k_cyc_to_us_floor64(k_cycle_get_32()) /
			  ZB_TIMER_US_PER_TICK) - zb_timer_base_ticks;
}

uint64_t zb_timer_get_current_time_us(void)
{
	return k_cyc_to_us_floor64(k_cycle_get_32());
}

uint32_t zb_timer_get_remaining_time(uint8_t ch)
{
	if (ch >= ZB_TIMER_CH_NUM) {
		return 0;
	}
	uint32_t remaining_ms = k_timer_remaining_get(&zb_timers[ch]);

	return (remaining_ms * 1000) / ZB_TIMER_US_PER_TICK;
}

void zb_timer_start(uint8_t ch, uint32_t target_time, zb_timer_cb_t cb)
{
	zb_timer_ensure_init();
	if (ch >= ZB_TIMER_CH_NUM) {
		return;
	}
	zb_timer_cbs[ch] = cb;

	uint32_t now = zb_timer_get_current_time();
	int32_t delta_ticks = (int32_t)(target_time - now);

	if (delta_ticks <= 0) {
		delta_ticks = 1;
	}
	uint32_t delta_us = (uint32_t)delta_ticks * ZB_TIMER_US_PER_TICK;

	k_timer_start(&zb_timers[ch], K_USEC(delta_us), K_NO_WAIT);
}

void *zb_timer_stop(uint8_t ch)
{
	if (ch >= ZB_TIMER_CH_NUM) {
		return NULL;
	}
	k_timer_stop(&zb_timers[ch]);
	zb_timer_cb_t cb = zb_timer_cbs[ch];

	zb_timer_cbs[ch] = NULL;
	return (void *)cb;
}

void zb_timer_irq(uint32_t intStatus)
{
	ARG_UNUSED(intStatus);
}

void zb_timer_cfg(uint32_t init_time)
{
	zb_timer_ensure_init();
	zb_timer_base_ticks = (uint32_t)(k_cyc_to_us_floor64(k_cycle_get_32()) /
					 ZB_TIMER_US_PER_TICK) - init_time;
}

void zb_timer_cfg_us(uint64_t init_time)
{
	ARG_UNUSED(init_time);
	zb_timer_ensure_init();
}

void zb_timer_store(void)
{
}

void zb_timer_store_time(void)
{
}

void zb_timer_store_events(void)
{
}

void zb_timer_restore(uint32_t jump_time, uint8_t run_expired)
{
	ARG_UNUSED(jump_time);
	ARG_UNUSED(run_expired);
}

void zb_timer_restore_time(uint32_t jump_time)
{
	ARG_UNUSED(jump_time);
}

void zb_timer_restore_events(uint8_t run_expired)
{
	ARG_UNUSED(run_expired);
}

/* rxOnWhenIdle, theTimerService, phy, psb, mac provided by blobs */
void *stub;

void convert_arrayToU64(const uint8_t *arr, uint64_t *val)
{
	memcpy(val, arr, sizeof(uint64_t));
}

void convert_u64ToArray(uint64_t val, uint8_t *arr)
{
	memcpy(arr, &val, sizeof(uint64_t));
}

/* zb_getRole, zb_getIeeeAddr, zb_getParentShortAddr, zb_getChildTableEntryCount,
 * zb_isRxForceOn, zb_isTouchlinkInProgress, zb_rx_enabled provided by blobs */

/* zb_getPendingFrameCount also in blobs */

void zb_zsedUpdateParentLink(void *pkt, uint8_t a, uint8_t b)
{
	ARG_UNUSED(pkt);
	ARG_UNUSED(a);
	ARG_UNUSED(b);
}

/* ZCL stubs for symbols the blob references but we don't link ZCL yet */
void *p_cusClustBasic;
void *p_cusClustColor;
void *p_cusClustIASZone;
void *p_cusClustLevel;
void *p_cusClustTouchlink;

int zcl_enableTouchlinkTar(void)
{
	return 0;
}

int zcl_touchlinkIniInProgress(void)
{
	return 0;
}

int zcl_touchlinkTarInProgress(void)
{
	return 0;
}

/* time/localtime provided by picolibc */

/* Init function called at boot */
static int zigbee_shim_init(void)
{
	k_sem_init(&zigbee_task_sem, 0, 1);
	zb_timer_ensure_init();
	LOG_INF("zigbee shim initialized");
	return 0;
}

SYS_INIT(zigbee_shim_init, POST_KERNEL, ZIGBEE_SHIM_INIT_PRIO);
