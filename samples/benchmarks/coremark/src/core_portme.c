/*
 * Zephyr RTOS port for CoreMark
 * SPDX-License-Identifier: Apache-2.0
 */

#include "coremark.h"
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#if VALIDATION_RUN
volatile ee_s32 seed1_volatile = 0x3415;
volatile ee_s32 seed2_volatile = 0x3415;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PERFORMANCE_RUN
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PROFILE_RUN
volatile ee_s32 seed1_volatile = 0x8;
volatile ee_s32 seed2_volatile = 0x8;
volatile ee_s32 seed3_volatile = 0x8;
#endif
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;

#define EE_TICKS_PER_SEC  1000
#define TIMER_RES_DIVIDER 1
#define NSECS_PER_SEC     EE_TICKS_PER_SEC

static CORE_TICKS start_time_val, stop_time_val;

#define CPU_MHZ (DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency) / 1000000)

ee_u32 default_num_contexts = 1;
uint32_t clock_frequency_mhz = CPU_MHZ;


void start_time(void)
{
	start_time_val = k_uptime_get_32();
}

void stop_time(void)
{
	stop_time_val = k_uptime_get_32();
}

CORE_TICKS get_time(void)
{
	return stop_time_val - start_time_val;
}

secs_ret time_in_secs(CORE_TICKS ticks)
{
	return ((secs_ret)ticks) / (secs_ret)EE_TICKS_PER_SEC;
}

void portable_init(core_portable *p, int *argc, char *argv[])
{
	(void)argc;
	(void)argv;

	ee_printf("\n--- CoreMark on Zephyr ---\n");
	ee_printf("CPU: %u MHz\n\n", clock_frequency_mhz);
	p->portable_id = 1;
}

void portable_fini(core_portable *p)
{
	p->portable_id = 0;
}
