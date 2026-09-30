/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BL61X_SDR_DUMP_RING_H_
#define BL61X_SDR_DUMP_RING_H_

#include <stdbool.h>
#include <stdint.h>

void dump_ring_init(void);
void dump_ring_restart_tracking(void);
void dump_ring_stream_state_reset(void);
bool dump_ring_producer_active(void);
uint32_t dump_ring_dropped_chunks(void);
uint32_t dump_ring_source_chunk_index(void);
uint32_t dump_ring_agc_gain_changes(void);
/* Latest absolute write position in words */
uint64_t dump_ring_abs_write(void);

#endif /* BL61X_SDR_DUMP_RING_H_ */
