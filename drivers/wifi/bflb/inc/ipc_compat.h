/**
 * @file ipc_compat.h
 * Copyright (C) Bouffalo Lab 2016-2018
 */

#ifndef _IPC_H_
#define _IPC_H_

#include "bl_os_private.h"

#undef __WARN
#define __WARN() do { } while (0)

#define WARN_ON(condition) (!!(condition))
#define WARN_ON_ONCE(condition) (!!(condition))

#define __round_mask(x, y) ((__typeof__(x))((y)-1))
#define round_up(x, y) ((((x)-1) | __round_mask(x, y))+1)
#define round_down(x, y) ((x) & ~__round_mask(x, y))

#define ASSERT_ERR(condition) do { } while (0)

#endif /* _IPC_H_ */
