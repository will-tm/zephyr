/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * BL808 WiFi OS adapter — maps g_bl_ops_funcs to Zephyr APIs.
 * Based on Bouffalo Lab SDK bl_os_hal.c (Apache-2.0).
 */

#include <stdarg.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#include <bl_os_adapter/bl_os_adapter.h>

LOG_MODULE_REGISTER(bl_os_adapter, LOG_LEVEL_ERR);

/* Wrapper structs — heap-allocated, returned as opaque pointers */

struct timer_adpt {
	struct k_timer timer;
	void (*func)(void *arg);
	void *arg;
};

struct irq_adpt {
	void (*func)(void *arg);
	void *arg;
};

struct task_notify {
	struct k_sem sem;
};

/*
 * Per-thread notification table.
 *
 * The LMAC blob uses a FreeRTOS pattern: it calls _task_get_current_task()
 * to get the thread handle, then passes that handle to _task_notify() and
 * _task_wait().  In FreeRTOS, task notification is built into the TCB.
 * In Zephyr, k_thread has no built-in notification, so we maintain a
 * side table mapping thread pointers to semaphores.
 */
struct thread_notify_entry {
	void *handle;
	struct k_sem sem;
	bool in_use;
};

#define MAX_THREAD_NOTIFIES 4
static struct thread_notify_entry thread_notify_table[MAX_THREAD_NOTIFIES];

static struct k_sem *find_thread_sem(void *handle)
{
	for (int i = 0; i < MAX_THREAD_NOTIFIES; i++) {
		if (thread_notify_table[i].in_use && thread_notify_table[i].handle == handle) {
			return &thread_notify_table[i].sem;
		}
	}
	return NULL;
}

static struct k_sem *register_thread_sem(void *handle)
{
	for (int i = 0; i < MAX_THREAD_NOTIFIES; i++) {
		if (!thread_notify_table[i].in_use) {
			thread_notify_table[i].handle = handle;
			thread_notify_table[i].in_use = true;
			k_sem_init(&thread_notify_table[i].sem, 0, 1);
			return &thread_notify_table[i].sem;
		}
	}
	return NULL;
}

struct mq_adpt {
	struct k_msgq msgq;
	char *buf;
};

/* Forward declarations */
static void bl_os_assert_func(const char *file, int line, const char *func, const char *expr);
static int bl_os_api_init(void);
static uint32_t bl_os_enter_critical_impl(void);
static void bl_os_exit_critical_impl(uint32_t level);
static int bl_os_msleep_impl(long ms);
static int bl_os_sleep_impl(unsigned int seconds);
static BL_EventGroup_t bl_os_event_create(void);
static void bl_os_event_delete(BL_EventGroup_t event);
static uint32_t bl_os_event_send(BL_EventGroup_t event, uint32_t bits);
static uint32_t bl_os_event_wait(BL_EventGroup_t event, uint32_t bits_to_wait_for,
				 int clear_on_exit, int wait_for_all_bits,
				 uint32_t block_time_tick);
static int bl_os_event_register_impl(int type, void *cb, void *arg);
static int bl_os_event_notify_impl(int evt, int val);
static int bl_os_task_create_impl(const char *name, void *entry, uint32_t stack_depth, void *param,
				  uint32_t prio, BL_TaskHandle_t task_handle);
static void bl_os_task_delete_impl(BL_TaskHandle_t task_handle);
static BL_TaskHandle_t bl_os_task_get_current_impl(void);
static BL_TaskHandle_t bl_os_task_notify_create_impl(void);
static void bl_os_task_notify_impl(BL_TaskHandle_t task_handle);
static void bl_os_task_wait_impl(BL_TaskHandle_t task_handle, uint32_t tick);
static void bl_os_lock_gaint_impl(void);
static void bl_os_unlock_gaint_impl(void);
static void bl_os_irq_attach_impl(int32_t n, void *f, void *arg);
static void bl_os_irq_enable_impl(int32_t n);
static void bl_os_irq_disable_impl(int32_t n);
static void *bl_os_workqueue_create_impl(void);
static int bl_os_workqueue_submit_hp_impl(void *work, void *worker, void *argv, long tick);
static int bl_os_workqueue_submit_lp_impl(void *work, void *worker, void *argv, long tick);
static BL_Timer_t bl_os_timer_create_impl(void *func, void *argv);
static int bl_os_timer_delete_impl(BL_Timer_t timerid, uint32_t tick);
static int bl_os_timer_start_once_impl(BL_Timer_t timerid, long t_sec, long t_nsec);
static int bl_os_timer_start_periodic_impl(BL_Timer_t timerid, long t_sec, long t_nsec);
static BL_Sem_t bl_os_sem_create_impl(uint32_t init);
static void bl_os_sem_delete_impl(BL_Sem_t semphr);
static int32_t bl_os_sem_take_impl(BL_Sem_t semphr, uint32_t tick);
static int32_t bl_os_sem_give_impl(BL_Sem_t semphr);
static BL_Mutex_t bl_os_mutex_create_impl(void);
static void bl_os_mutex_delete_impl(BL_Mutex_t mutex);
static int32_t bl_os_mutex_lock_impl(BL_Mutex_t mutex);
static int32_t bl_os_mutex_unlock_impl(BL_Mutex_t mutex);
static BL_MessageQueue_t bl_os_mq_create_impl(uint32_t queue_len, uint32_t item_size);
static void bl_os_mq_delete_impl(BL_MessageQueue_t queue);
static int bl_os_mq_send_wait_impl(BL_MessageQueue_t queue, void *item, uint32_t len,
				   uint32_t ticks, int prio);
static int bl_os_mq_send_impl(BL_MessageQueue_t queue, void *item, uint32_t len);
static int bl_os_mq_recv_impl(BL_MessageQueue_t queue, void *item, uint32_t len, uint32_t tick);
static void *bl_os_malloc_impl(unsigned int size);
static void bl_os_free_impl(void *p);
static void *bl_os_zalloc_impl(unsigned int size);
static uint64_t bl_os_get_time_ms_impl(void);
static uint32_t bl_os_get_tick_impl(void);
static void bl_os_log_write_impl(uint32_t level, const char *tag, const char *file, int line,
				 const char *format, ...);
static void bl_os_printf_impl(const char *fmt, ...);
static void bl_os_puts_impl(const char *s);
static int bl_os_task_notify_isr_impl(BL_TaskHandle_t task_handle);
static void bl_os_yield_from_isr_impl(int xYield);
static unsigned int bl_os_ms_to_tick_impl(unsigned int ms);
static BL_TimeOut_t bl_os_set_timeout_impl(void);
static int bl_os_check_timeout_impl(BL_TimeOut_t xTimeOut, BL_TickType_t *xTicksToWait);

/*
 * The global function table consumed by libwifi.a / libbl606p_phyrf.a
 */
bl_ops_funcs_t g_bl_ops_funcs = {
	._version = BL_OS_ADAPTER_VERSION,
	._printf = bl_os_printf_impl,
	._puts = bl_os_puts_impl,
	._assert = bl_os_assert_func,
	._init = bl_os_api_init,
	._enter_critical = bl_os_enter_critical_impl,
	._exit_critical = bl_os_exit_critical_impl,
	._msleep = bl_os_msleep_impl,
	._sleep = bl_os_sleep_impl,
	._event_group_create = bl_os_event_create,
	._event_group_delete = bl_os_event_delete,
	._event_group_send = bl_os_event_send,
	._event_group_wait = bl_os_event_wait,
	._event_register = bl_os_event_register_impl,
	._event_notify = bl_os_event_notify_impl,
	._task_create = bl_os_task_create_impl,
	._task_delete = bl_os_task_delete_impl,
	._task_get_current_task = bl_os_task_get_current_impl,
	._task_notify_create = bl_os_task_notify_create_impl,
	._task_notify = bl_os_task_notify_impl,
	._task_wait = bl_os_task_wait_impl,
	._lock_gaint = bl_os_lock_gaint_impl,
	._unlock_gaint = bl_os_unlock_gaint_impl,
	._irq_attach = bl_os_irq_attach_impl,
	._irq_enable = bl_os_irq_enable_impl,
	._irq_disable = bl_os_irq_disable_impl,
	._workqueue_create = bl_os_workqueue_create_impl,
	._workqueue_submit_hp = bl_os_workqueue_submit_hp_impl,
	._workqueue_submit_lp = bl_os_workqueue_submit_lp_impl,
	._timer_create = bl_os_timer_create_impl,
	._timer_delete = bl_os_timer_delete_impl,
	._timer_start_once = bl_os_timer_start_once_impl,
	._timer_start_periodic = bl_os_timer_start_periodic_impl,
	._sem_create = bl_os_sem_create_impl,
	._sem_delete = bl_os_sem_delete_impl,
	._sem_take = bl_os_sem_take_impl,
	._sem_give = bl_os_sem_give_impl,
	._mutex_create = bl_os_mutex_create_impl,
	._mutex_delete = bl_os_mutex_delete_impl,
	._mutex_lock = bl_os_mutex_lock_impl,
	._mutex_unlock = bl_os_mutex_unlock_impl,
	._queue_create = bl_os_mq_create_impl,
	._queue_delete = bl_os_mq_delete_impl,
	._queue_send_wait = bl_os_mq_send_wait_impl,
	._queue_send = bl_os_mq_send_impl,
	._queue_recv = bl_os_mq_recv_impl,
	._malloc = bl_os_malloc_impl,
	._free = bl_os_free_impl,
	._zalloc = bl_os_zalloc_impl,
	._get_time_ms = bl_os_get_time_ms_impl,
	._get_tick = bl_os_get_tick_impl,
	._log_write = bl_os_log_write_impl,
	._task_notify_isr = bl_os_task_notify_isr_impl,
	._yield_from_isr = bl_os_yield_from_isr_impl,
	._ms_to_tick = bl_os_ms_to_tick_impl,
	._set_timeout = bl_os_set_timeout_impl,
	._check_timeout = bl_os_check_timeout_impl,
};

/*
 * Assert / init
 */

static void bl_os_assert_func(const char *file, int line, const char *func, const char *expr)
{
	printk("ASSERT %s:%d (%s): %s\n", file, line, func, expr);
	k_panic();
}

static int bl_os_api_init(void)
{
	return 0;
}

/*
 * Critical section (maps to irq_lock/unlock)
 */

static uint32_t bl_os_enter_critical_impl(void)
{
	return irq_lock();
}

static void bl_os_exit_critical_impl(uint32_t level)
{
	irq_unlock(level);
}

/*
 * Sleep
 */

static int bl_os_msleep_impl(long ms)
{
	k_msleep(ms);
	return 0;
}

static int bl_os_sleep_impl(unsigned int seconds)
{
	k_sleep(K_SECONDS(seconds));
	return 0;
}

/*
 * Event group (maps to k_event)
 */

static BL_EventGroup_t bl_os_event_create(void)
{
	struct k_event *evt = k_malloc(sizeof(struct k_event));

	if (evt == NULL) {
		LOG_ERR("event create: out of memory");
		return NULL;
	}
	k_event_init(evt);
	return evt;
}

static void bl_os_event_delete(BL_EventGroup_t event)
{
	k_free(event);
}

static uint32_t bl_os_event_send(BL_EventGroup_t event, uint32_t bits)
{
	struct k_event *evt = event;

	if (evt == NULL) {
		printk("[WF] BUG: event_send called with NULL event!\n");
		return 0;
	}
	k_event_post(evt, bits);
	return k_event_test(evt, 0xFFFFFFFF);
}

static uint32_t bl_os_event_wait(BL_EventGroup_t event, uint32_t bits_to_wait_for,
				 int clear_on_exit, int wait_for_all_bits, uint32_t block_time_tick)
{
	struct k_event *evt = event;
	k_timeout_t timeout;
	uint32_t result;

	if (evt == NULL) {
		printk("[WF] BUG: event_wait called with NULL event!\n");
		return 0;
	}

	if (block_time_tick == BL_OS_WAITING_FOREVER) {
		timeout = K_FOREVER;
	} else {
		timeout = K_TICKS(block_time_tick);
	}

	if (wait_for_all_bits) {
		result = k_event_wait_all(evt, bits_to_wait_for, clear_on_exit, timeout);
	} else {
		result = k_event_wait(evt, bits_to_wait_for, clear_on_exit, timeout);
	}

	return result;
}

static int bl_os_event_register_impl(int type, void *cb, void *arg)
{
	/* WiFi event registration — stub for now */
	return 0;
}

static int bl_os_event_notify_impl(int evt, int val)
{
	/* WiFi event notification — stub for now */
	return 0;
}

/*
 * Task (maps to k_thread)
 */

struct thread_wrapper {
	struct k_thread thread;
	k_thread_stack_t *stack;
	uint32_t stack_size;
};

static int bl_os_task_create_impl(const char *name, void *entry, uint32_t stack_depth, void *param,
				  uint32_t prio, BL_TaskHandle_t task_handle)
{
	struct thread_wrapper *tw;
	k_thread_stack_t *stack;

	/* stack_depth is in bytes (SDK divides by 4 for FreeRTOS words) */
	tw = k_malloc(sizeof(*tw));
	if (tw == NULL) {
		return -1;
	}

	stack = k_malloc(stack_depth);
	if (stack == NULL) {
		k_free(tw);
		return -1;
	}

	tw->stack = stack;
	tw->stack_size = stack_depth;

	k_thread_create(&tw->thread, stack, stack_depth, (k_thread_entry_t)entry, param, NULL, NULL,
			K_PRIO_COOP(prio), 0, K_NO_WAIT);

	if (name != NULL) {
		k_thread_name_set(&tw->thread, name);
	}

	return 0;
}

static void bl_os_task_delete_impl(BL_TaskHandle_t task_handle)
{
	/* Thread abort — the wrapper memory leaks but that matches SDK behavior */
	if (task_handle != NULL) {
		struct thread_wrapper *tw = task_handle;
		k_thread_abort(&tw->thread);
	}
}

static BL_TaskHandle_t bl_os_task_get_current_impl(void)
{
	void *handle = k_current_get();

	/* Ensure this thread has a notification semaphore registered.
	 * The blob (ipc_emb_init) calls _task_get_current_task() and then
	 * uses the returned handle with _task_notify/_task_wait.
	 */
	if (!find_thread_sem(handle)) {
		register_thread_sem(handle);
	}

	return handle;
}

/*
 * Task notify (maps to k_sem with limit=1)
 */

static BL_TaskHandle_t bl_os_task_notify_create_impl(void)
{
	struct task_notify *tn = k_malloc(sizeof(*tn));

	if (tn == NULL) {
		printk("[WF] task_notify_create: OOM!\n");
		return NULL;
	}

	k_sem_init(&tn->sem, 0, 1);
	return tn;
}

static void bl_os_task_notify_impl(BL_TaskHandle_t task_handle)
{
	struct k_sem *sem;

	if (task_handle == NULL) {
		printk("[WF] task_notify: NULL handle!\n");
		return;
	}

	/* Check thread notify table first (FreeRTOS pattern) */
	sem = find_thread_sem(task_handle);
	if (sem) {
		k_sem_give(sem);
		return;
	}

	/* Fallback: treat as struct task_notify * (standalone pattern) */
	struct task_notify *tn = task_handle;

	k_sem_give(&tn->sem);
}

static void bl_os_task_wait_impl(BL_TaskHandle_t task_handle, uint32_t tick)
{
	struct k_sem *sem;
	k_timeout_t timeout;

	if (task_handle == NULL) {
		return;
	}

	/* Check thread notify table first (FreeRTOS pattern) */
	sem = find_thread_sem(task_handle);
	if (!sem) {
		/* Fallback: treat as struct task_notify * */
		struct task_notify *tn = task_handle;

		sem = &tn->sem;
	}

	if (tick == BL_OS_WAITING_FOREVER) {
		timeout = K_FOREVER;
	} else {
		timeout = K_TICKS(tick);
	}

	k_sem_take(sem, timeout);
}

/*
 * Giant lock (no-op, same as SDK)
 */

static void bl_os_lock_gaint_impl(void)
{
}

static void bl_os_unlock_gaint_impl(void)
{
}

/*
 * IRQ
 */

static struct irq_adpt *irq_adapters[CONFIG_NUM_IRQS];

static void bl_os_irq_handler(const void *arg)
{
	const struct irq_adpt *adapter = arg;

	if (adapter && adapter->func) {
		adapter->func(adapter->arg);
	}
}

static void bl_os_irq_attach_impl(int32_t n, void *f, void *arg)
{
	struct irq_adpt *adapter;

	printk("[WF] irq_attach: n=%d f=%p arg=%p\n", n, f, arg);

	adapter = k_malloc(sizeof(*adapter));
	if (adapter == NULL) {
		LOG_ERR("irq attach: out of memory");
		return;
	}

	adapter->func = f;
	adapter->arg = arg;

	if (n < CONFIG_NUM_IRQS) {
		irq_adapters[n] = adapter;
	}

	irq_connect_dynamic(n, 1, bl_os_irq_handler, adapter, 0);
	irq_enable(n);
}

static void bl_os_irq_enable_impl(int32_t n)
{
	irq_enable(n);
}

static void bl_os_irq_disable_impl(int32_t n)
{
	irq_disable(n);
}

/*
 * Workqueue (simplified — just notify, same as SDK)
 */

static void *bl_os_workqueue_create_impl(void)
{
	return bl_os_task_notify_create_impl();
}

static int bl_os_workqueue_submit_hp_impl(void *work, void *worker, void *argv, long tick)
{
	bl_os_task_notify_impl(work);
	return 0;
}

static int bl_os_workqueue_submit_lp_impl(void *work, void *worker, void *argv, long tick)
{
	bl_os_task_notify_impl(work);
	return 0;
}

/*
 * Timer (maps to k_timer)
 */

static void bl_os_timer_expiry(struct k_timer *timer)
{
	struct timer_adpt *adpt = CONTAINER_OF(timer, struct timer_adpt, timer);

	if (adpt->func) {
		adpt->func(adpt->arg);
	}
}

static BL_Timer_t bl_os_timer_create_impl(void *func, void *argv)
{
	struct timer_adpt *adpt = k_malloc(sizeof(*adpt));

	if (adpt == NULL) {
		LOG_ERR("timer create: out of memory");
		return NULL;
	}

	adpt->func = func;
	adpt->arg = argv;
	k_timer_init(&adpt->timer, bl_os_timer_expiry, NULL);

	return adpt;
}

static int bl_os_timer_delete_impl(BL_Timer_t timerid, uint32_t tick)
{
	struct timer_adpt *adpt = timerid;

	ARG_UNUSED(tick);

	if (adpt == NULL) {
		return -1;
	}

	k_timer_stop(&adpt->timer);
	k_free(adpt);

	return 0;
}

static k_timeout_t timer_sec_nsec_to_timeout(long t_sec, long t_nsec)
{
	int64_t ms = (t_sec * 1000) + (t_nsec > 1000000 ? t_nsec / 1000000 : 0);

	if (ms <= 0) {
		ms = 1;
	}
	return K_MSEC(ms);
}

static int bl_os_timer_start_once_impl(BL_Timer_t timerid, long t_sec, long t_nsec)
{
	struct timer_adpt *adpt = timerid;

	if (adpt == NULL) {
		return -1;
	}

	k_timer_start(&adpt->timer, timer_sec_nsec_to_timeout(t_sec, t_nsec), K_NO_WAIT);
	return 0;
}

static int bl_os_timer_start_periodic_impl(BL_Timer_t timerid, long t_sec, long t_nsec)
{
	struct timer_adpt *adpt = timerid;
	k_timeout_t period;

	if (adpt == NULL) {
		return -1;
	}

	period = timer_sec_nsec_to_timeout(t_sec, t_nsec);
	k_timer_start(&adpt->timer, period, period);
	return 0;
}

/*
 * Semaphore
 */

static BL_Sem_t bl_os_sem_create_impl(uint32_t init)
{
	struct k_sem *sem = k_malloc(sizeof(*sem));

	if (sem == NULL) {
		LOG_ERR("sem create: out of memory");
		return NULL;
	}

	k_sem_init(sem, init, K_SEM_MAX_LIMIT);
	return sem;
}

static void bl_os_sem_delete_impl(BL_Sem_t semphr)
{
	k_free(semphr);
}

static int32_t bl_os_sem_take_impl(BL_Sem_t semphr, uint32_t tick)
{
	struct k_sem *sem = semphr;
	k_timeout_t timeout;
	int ret;

	if (sem == NULL) {
		return -1;
	}

	if (tick == BL_OS_WAITING_FOREVER) {
		timeout = K_FOREVER;
	} else if (tick == BL_OS_NO_WAITING) {
		timeout = K_NO_WAIT;
	} else {
		timeout = K_TICKS(tick);
	}

	ret = k_sem_take(sem, timeout);
	return (ret == 0) ? 0 : 1;
}

static int32_t bl_os_sem_give_impl(BL_Sem_t semphr)
{
	struct k_sem *sem = semphr;

	if (sem == NULL) {
		return -1;
	}

	k_sem_give(sem);
	return 0;
}

/*
 * Mutex
 */

static BL_Mutex_t bl_os_mutex_create_impl(void)
{
	struct k_mutex *mutex = k_malloc(sizeof(*mutex));

	if (mutex == NULL) {
		LOG_ERR("mutex create: out of memory");
		return NULL;
	}

	k_mutex_init(mutex);
	return mutex;
}

static void bl_os_mutex_delete_impl(BL_Mutex_t mutex)
{
	k_free(mutex);
}

static int32_t bl_os_mutex_lock_impl(BL_Mutex_t mutex)
{
	struct k_mutex *m = mutex;
	int ret;

	if (m == NULL) {
		return -1;
	}

	ret = k_mutex_lock(m, K_FOREVER);
	return (ret == 0) ? 0 : 1;
}

static int32_t bl_os_mutex_unlock_impl(BL_Mutex_t mutex)
{
	struct k_mutex *m = mutex;
	int ret;

	if (m == NULL) {
		return -1;
	}

	ret = k_mutex_unlock(m);
	return (ret == 0) ? 0 : 1;
}

/*
 * Message queue
 */

static BL_MessageQueue_t bl_os_mq_create_impl(uint32_t queue_len, uint32_t item_size)
{
	struct mq_adpt *mq = k_malloc(sizeof(*mq));

	if (mq == NULL) {
		return NULL;
	}

	mq->buf = k_malloc(queue_len * item_size);
	if (mq->buf == NULL) {
		k_free(mq);
		return NULL;
	}

	k_msgq_init(&mq->msgq, mq->buf, item_size, queue_len);
	return mq;
}

static void bl_os_mq_delete_impl(BL_MessageQueue_t queue)
{
	struct mq_adpt *mq = queue;

	if (mq != NULL) {
		k_free(mq->buf);
		k_free(mq);
	}
}

static int bl_os_mq_send_wait_impl(BL_MessageQueue_t queue, void *item, uint32_t len,
				   uint32_t ticks, int prio)
{
	struct mq_adpt *mq = queue;
	k_timeout_t timeout;
	int ret;

	ARG_UNUSED(len);
	ARG_UNUSED(prio);

	if (mq == NULL) {
		return -1;
	}

	if (ticks == BL_OS_WAITING_FOREVER) {
		timeout = K_FOREVER;
	} else if (ticks == 0) {
		timeout = K_NO_WAIT;
	} else {
		timeout = K_TICKS(ticks);
	}

	ret = k_msgq_put(&mq->msgq, item, timeout);
	return (ret == 0) ? 0 : 1;
}

static int bl_os_mq_send_impl(BL_MessageQueue_t queue, void *item, uint32_t len)
{
	return bl_os_mq_send_wait_impl(queue, item, len, 0, 0);
}

static int bl_os_mq_recv_impl(BL_MessageQueue_t queue, void *item, uint32_t len, uint32_t tick)
{
	struct mq_adpt *mq = queue;
	k_timeout_t timeout;
	int ret;

	ARG_UNUSED(len);

	if (mq == NULL) {
		return -1;
	}

	if (tick == BL_OS_WAITING_FOREVER) {
		timeout = K_FOREVER;
	} else if (tick == 0) {
		timeout = K_NO_WAIT;
	} else {
		timeout = K_TICKS(tick);
	}

	ret = k_msgq_get(&mq->msgq, item, timeout);
	return (ret == 0) ? 0 : 1;
}

/*
 * Memory
 */

static void *bl_os_malloc_impl(unsigned int size)
{
	void *p = k_malloc(size);

	if (p == NULL) {
		printk("[WF] malloc(%u) FAILED!\n", size);
	}
	return p;
}

static void bl_os_free_impl(void *p)
{
	k_free(p);
}

static void *bl_os_zalloc_impl(unsigned int size)
{
	return k_calloc(1, size);
}

/*
 * Time
 */

static uint64_t bl_os_get_time_ms_impl(void)
{
	return k_uptime_get();
}

static uint32_t bl_os_get_tick_impl(void)
{
	return (uint32_t)k_uptime_ticks();
}

/*
 * Logging / printf
 */

static void bl_os_printf_impl(const char *fmt, ...)
{
#if 0
	va_list ap;

	va_start(ap, fmt);
	vprintk(fmt, ap);
	va_end(ap);
#endif
}

static void bl_os_puts_impl(const char *s)
{
#if 0
	printk("%s", s);
#endif
}

static void bl_os_log_write_impl(uint32_t level, const char *tag, const char *file, int line,
				 const char *format, ...)
{
#if 0
	va_list ap;

	va_start(ap, format);
	printk("[%s] ", tag ? tag : "?");
	vprintk(format, ap);
	printk("\n");
	va_end(ap);
#endif
}

/*
 * ISR helpers
 */

static int bl_os_task_notify_isr_impl(BL_TaskHandle_t task_handle)
{
	struct k_sem *sem;

	if (task_handle == NULL) {
		return -1;
	}

	/* Check thread notify table first (FreeRTOS pattern) */
	sem = find_thread_sem(task_handle);
	if (sem) {
		k_sem_give(sem);
		return 0;
	}

	/* Fallback: treat as struct task_notify * */
	struct task_notify *tn = task_handle;

	k_sem_give(&tn->sem);
	return 0;
}

static void bl_os_yield_from_isr_impl(int xYield)
{
	ARG_UNUSED(xYield);
	/* Zephyr handles rescheduling on ISR exit automatically */
}

/*
 * Tick conversion / timeout
 */

static unsigned int bl_os_ms_to_tick_impl(unsigned int ms)
{
	return k_ms_to_ticks_ceil32(ms);
}

static BL_TimeOut_t bl_os_set_timeout_impl(void)
{
	/* Return current tick as opaque timeout reference */
	uint32_t *t = k_malloc(sizeof(uint32_t));

	if (t) {
		*t = (uint32_t)k_uptime_ticks();
	}
	return t;
}

static int bl_os_check_timeout_impl(BL_TimeOut_t xTimeOut, BL_TickType_t *xTicksToWait)
{
	uint32_t *start = xTimeOut;
	uint32_t elapsed;

	if (start == NULL || xTicksToWait == NULL) {
		return 1;
	}

	elapsed = (uint32_t)k_uptime_ticks() - *start;
	if (elapsed >= *xTicksToWait) {
		*xTicksToWait = 0;
		return 1; /* timed out */
	}

	*xTicksToWait -= elapsed;
	*start = (uint32_t)k_uptime_ticks();
	return 0;
}
