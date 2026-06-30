/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Extra C++ runtime stubs for zigbee stack blobs.
 * operator new/delete, __cxa_pure_virtual, __cxa_atexit are provided
 * by Zephyr's minimal C++ lib (CONFIG_MINIMAL_LIBCPP).
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

#define CXA_GUARD_INIT_BIT BIT(0)

static K_MUTEX_DEFINE(guard_mutex);

void _ZdlPvj(void *ptr, size_t size)
{
	(void)size;
	free(ptr);
}

int __cxa_guard_acquire(uint32_t *guard)
{
	k_mutex_lock(&guard_mutex, K_FOREVER);
	if (*guard & CXA_GUARD_INIT_BIT) {
		k_mutex_unlock(&guard_mutex);
		return 0;
	}
	return 1;
}

void __cxa_guard_release(uint32_t *guard)
{
	*guard = CXA_GUARD_INIT_BIT;
	k_mutex_unlock(&guard_mutex);
}

void __cxa_guard_abort(uint32_t *guard)
{
	(void)guard;
	k_mutex_unlock(&guard_mutex);
}

void _ZSt17__throw_bad_allocv(void)
{
	k_panic();
}

void _ZSt19__throw_logic_errorPKc(const char *msg)
{
	(void)msg;
	k_panic();
}

void _ZSt20__throw_length_errorPKc(const char *msg)
{
	(void)msg;
	k_panic();
}

void _ZSt25__throw_bad_function_callv(void)
{
	k_panic();
}

const char _ZSt7nothrow = 0;

#define HEAP_FREE_ESTIMATE_DIVISOR 2

size_t xPortGetFreeHeapSize(void)
{
	return CONFIG_BFLB_SHIM_HEAP_SIZE / HEAP_FREE_ESTIMATE_DIVISOR;
}
