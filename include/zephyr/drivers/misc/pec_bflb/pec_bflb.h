/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Public API for the Bouffalo Lab PEC (Programmable Engine/Controller), a
 * programmable I/O state-machine block on the BL61x SoCs. Four state machines
 * share a 32-entry instruction memory. This is a low-level API to load
 * programs, configure and run a state machine, and move data through its FIFOs.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_PEC_BFLB_PEC_BFLB_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_PEC_BFLB_PEC_BFLB_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Number of state machines in a PEC instance. */
#define PEC_BFLB_NUM_STATE_MACHINES 4U
/* Number of 16-bit words in the shared instruction memory. */
#define PEC_BFLB_INSTRUCTION_COUNT  32U

/* Instruction word layout (16-bit encoding). */
#define PEC_BFLB_INSTR_OP_MASK         0xE000U /* opcode, bits [15:13] */
#define PEC_BFLB_INSTR_HI_SHIFT        5U      /* operand "hi" position, bits [7:5] */
#define PEC_BFLB_INSTR_HI_MASK         0x01E0U
#define PEC_BFLB_INSTR_LO_MASK         0x001FU /* operand "lo", bits [4:0] */
#define PEC_BFLB_INSTR_DELAYSIDE_SHIFT 8U      /* delay/side-set, bits [12:8] */
#define PEC_BFLB_INSTR_DELAYSIDE_WIDTH 5U

/* Opcode base values. */
#define PEC_BFLB_OP_JMP  0x0000U
#define PEC_BFLB_OP_WAIT 0x2000U
#define PEC_BFLB_OP_IN   0x4000U
#define PEC_BFLB_OP_OUT  0x6000U
#define PEC_BFLB_OP_PUSH 0x8000U
#define PEC_BFLB_OP_PULL 0x8080U
#define PEC_BFLB_OP_MOV  0xA000U
#define PEC_BFLB_OP_IRQ  0xC000U
#define PEC_BFLB_OP_SET  0xE000U

/* Operand sub-fields (within the 8-bit operand). */
#define PEC_BFLB_WAIT_POL_HIGH 0x4U /* WAIT polarity (above the 2-bit source) */
#define PEC_BFLB_FIFO_BLOCK    0x1U /* PUSH/PULL block bit */
#define PEC_BFLB_FIFO_COND     0x2U /* PUSH ifFull / PULL ifEmpty bit */
#define PEC_BFLB_PULL_SELECT   0x4U /* selects PULL over PUSH (instruction bit 7) */
#define PEC_BFLB_IRQ_WAIT      0x1U /* IRQ wait bit */
#define PEC_BFLB_IRQ_CLEAR     0x2U /* IRQ clear (vs raise) bit */
#define PEC_BFLB_MOV_OP_SHIFT  3U   /* MOV operation (above the 3-bit source) */

/* JMP condition selector (instruction bits [7:5]). */
enum pec_bflb_jmp_cond {
	PEC_BFLB_JMP_ALWAYS = 0,        /* unconditional */
	PEC_BFLB_JMP_X_ZERO = 1,        /* scratch X == 0 */
	PEC_BFLB_JMP_X_DEC = 2,         /* scratch X != 0, post-decrement */
	PEC_BFLB_JMP_Y_ZERO = 3,        /* scratch Y == 0 */
	PEC_BFLB_JMP_Y_DEC = 4,         /* scratch Y != 0, post-decrement */
	PEC_BFLB_JMP_X_NEQ_Y = 5,       /* scratch X != scratch Y */
	PEC_BFLB_JMP_PIN = 6,           /* branch on the configured input pin */
	PEC_BFLB_JMP_OSR_NOT_EMPTY = 7, /* output shift register not empty */
};

/* WAIT source selector. */
enum pec_bflb_wait_src {
	PEC_BFLB_WAIT_GPIO = 0, /* system GPIO selected by index */
	PEC_BFLB_WAIT_PIN = 1,  /* mapped input pin selected by index */
	PEC_BFLB_WAIT_IRQ = 2,  /* PEC IRQ flag selected by index */
};

/* IN source selector. */
enum pec_bflb_in_src {
	PEC_BFLB_IN_PINS = 0,
	PEC_BFLB_IN_X = 1,
	PEC_BFLB_IN_Y = 2,
	PEC_BFLB_IN_NULL = 3,
	PEC_BFLB_IN_ISR = 6,
	PEC_BFLB_IN_OSR = 7,
};

/* OUT destination selector. */
enum pec_bflb_out_dest {
	PEC_BFLB_OUT_PINS = 0,
	PEC_BFLB_OUT_X = 1,
	PEC_BFLB_OUT_Y = 2,
	PEC_BFLB_OUT_NULL = 3,
	PEC_BFLB_OUT_PINDIRS = 4,
	PEC_BFLB_OUT_PC = 5,
	PEC_BFLB_OUT_ISR = 6,
	PEC_BFLB_OUT_EXEC = 7,
};

/* MOV source selector. */
enum pec_bflb_mov_src {
	PEC_BFLB_MOV_SRC_PINS = 0,
	PEC_BFLB_MOV_SRC_X = 1,
	PEC_BFLB_MOV_SRC_Y = 2,
	PEC_BFLB_MOV_SRC_NULL = 3,
	PEC_BFLB_MOV_SRC_STATUS = 5,
	PEC_BFLB_MOV_SRC_ISR = 6,
	PEC_BFLB_MOV_SRC_OSR = 7,
};

/* MOV destination selector. */
enum pec_bflb_mov_dest {
	PEC_BFLB_MOV_DEST_PINS = 0,
	PEC_BFLB_MOV_DEST_X = 1,
	PEC_BFLB_MOV_DEST_Y = 2,
	PEC_BFLB_MOV_DEST_EXEC = 4,
	PEC_BFLB_MOV_DEST_PC = 5,
	PEC_BFLB_MOV_DEST_ISR = 6,
	PEC_BFLB_MOV_DEST_OSR = 7,
};

/* MOV bit operation. */
enum pec_bflb_mov_op {
	PEC_BFLB_MOV_OP_NONE = 0,
	PEC_BFLB_MOV_OP_INVERT = 1,
	PEC_BFLB_MOV_OP_BITREV = 2,
};

/* SET destination selector. */
enum pec_bflb_set_dest {
	PEC_BFLB_SET_DEST_PINS = 0,
	PEC_BFLB_SET_DEST_X = 1,
	PEC_BFLB_SET_DEST_Y = 2,
	PEC_BFLB_SET_DEST_PINDIRS = 4,
};

/*
 * Instruction encoders. Build raw 16-bit instruction words; OR a base encoder
 * with pec_bflb_encode_delay() to add delay cycles.
 */

/* Encode the delay/side-set field of an instruction (bits [12:8]). */
static inline uint16_t pec_bflb_encode_delayside(uint8_t side, uint8_t delay, uint8_t side_bits)
{
	uint8_t shift = (uint8_t)(PEC_BFLB_INSTR_DELAYSIDE_WIDTH - side_bits);
	uint8_t delay_mask = (uint8_t)((1U << shift) - 1U);
	uint8_t side_mask = (uint8_t)((1U << side_bits) - 1U);
	uint8_t field = (uint8_t)((delay & delay_mask) | (uint8_t)((side & side_mask) << shift));

	return (uint16_t)((uint16_t)field << PEC_BFLB_INSTR_DELAYSIDE_SHIFT);
}

/* Encode a plain delay (no side-set), bits [12:8]. */
static inline uint16_t pec_bflb_encode_delay(uint8_t cycles)
{
	return pec_bflb_encode_delayside(0U, cycles, 0U);
}

/* Assemble opcode | operands. */
static inline uint16_t pec_bflb_encode_op(uint16_t opcode, uint8_t hi, uint8_t lo)
{
	return (uint16_t)((opcode & PEC_BFLB_INSTR_OP_MASK) |
			  (uint16_t)(((uint16_t)hi << PEC_BFLB_INSTR_HI_SHIFT) &
				     PEC_BFLB_INSTR_HI_MASK) |
			  (uint16_t)(lo & PEC_BFLB_INSTR_LO_MASK));
}

/* Encode a JMP instruction. */
static inline uint16_t pec_bflb_encode_jmp(enum pec_bflb_jmp_cond cond, uint8_t addr)
{
	return pec_bflb_encode_op(PEC_BFLB_OP_JMP, (uint8_t)cond, addr);
}

/* Encode a WAIT instruction. */
static inline uint16_t pec_bflb_encode_wait(enum pec_bflb_wait_src src, bool polarity,
					    uint8_t index)
{
	uint8_t hi = (uint8_t)((uint8_t)src | (polarity ? PEC_BFLB_WAIT_POL_HIGH : 0U));

	return pec_bflb_encode_op(PEC_BFLB_OP_WAIT, hi, index);
}

/* Encode an IN instruction (bit_count of 32 is encoded as 0). */
static inline uint16_t pec_bflb_encode_in(enum pec_bflb_in_src src, uint8_t bit_count)
{
	return pec_bflb_encode_op(PEC_BFLB_OP_IN, (uint8_t)src, bit_count);
}

/* Encode an OUT instruction (bit_count of 32 is encoded as 0). */
static inline uint16_t pec_bflb_encode_out(enum pec_bflb_out_dest dest, uint8_t bit_count)
{
	return pec_bflb_encode_op(PEC_BFLB_OP_OUT, (uint8_t)dest, bit_count);
}

/* Encode a PUSH instruction. */
static inline uint16_t pec_bflb_encode_push(bool if_full, bool block)
{
	uint8_t hi =
		(uint8_t)((if_full ? PEC_BFLB_FIFO_COND : 0U) | (block ? PEC_BFLB_FIFO_BLOCK : 0U));

	return pec_bflb_encode_op(PEC_BFLB_OP_PUSH, hi, 0U);
}

/* Encode a PULL instruction. */
static inline uint16_t pec_bflb_encode_pull(bool if_empty, bool block)
{
	uint8_t hi = (uint8_t)((if_empty ? PEC_BFLB_FIFO_COND : 0U) |
			       (block ? PEC_BFLB_FIFO_BLOCK : 0U) | PEC_BFLB_PULL_SELECT);

	return pec_bflb_encode_op(PEC_BFLB_OP_PULL, hi, 0U);
}

/* Encode a MOV instruction. */
static inline uint16_t pec_bflb_encode_mov(enum pec_bflb_mov_dest dest, enum pec_bflb_mov_op op,
					   enum pec_bflb_mov_src src)
{
	uint8_t lo = (uint8_t)(((uint8_t)op << PEC_BFLB_MOV_OP_SHIFT) | (uint8_t)src);

	return pec_bflb_encode_op(PEC_BFLB_OP_MOV, (uint8_t)dest, lo);
}

/* Encode an IRQ set instruction (optionally waiting for the flag to clear). */
static inline uint16_t pec_bflb_encode_irq_set(bool wait, uint8_t index)
{
	uint8_t hi = (uint8_t)(wait ? PEC_BFLB_IRQ_WAIT : 0U);

	return pec_bflb_encode_op(PEC_BFLB_OP_IRQ, hi, index);
}

/* Encode an IRQ clear instruction. */
static inline uint16_t pec_bflb_encode_irq_clear(uint8_t index)
{
	return pec_bflb_encode_op(PEC_BFLB_OP_IRQ, PEC_BFLB_IRQ_CLEAR, index);
}

/* Encode a SET instruction. */
static inline uint16_t pec_bflb_encode_set(enum pec_bflb_set_dest dest, uint8_t data)
{
	return pec_bflb_encode_op(PEC_BFLB_OP_SET, (uint8_t)dest, data);
}

/* Encode a NOP (implemented as MOV Y, Y). */
static inline uint16_t pec_bflb_encode_nop(void)
{
	return pec_bflb_encode_mov(PEC_BFLB_MOV_DEST_Y, PEC_BFLB_MOV_OP_NONE, PEC_BFLB_MOV_SRC_Y);
}

/* A PEC program to load into the shared instruction memory. */
struct pec_bflb_program {
	const uint16_t *instructions; /* encoded 16-bit instructions */
	uint8_t length;               /* 1..PEC_BFLB_INSTRUCTION_COUNT */
	/*
	 * Required load offset, or -1 to let the allocator pick a free slot.
	 * With -1, JMP targets are relocated to the load offset; with >= 0 the
	 * program is loaded verbatim.
	 */
	int8_t origin;
};

/*
 * State-machine configuration. Initialise with pec_bflb_sm_config_default()
 * then override the fields you need. Mirrors the hardware EXECCTRL / SHIFTCTRL /
 * PINCTRL / CLKDIV registers.
 */
struct pec_bflb_sm_config {
	uint16_t clkdiv_int; /* integer part of the clock divider (0 means 65536) */
	uint8_t clkdiv_frac; /* fractional part (1/256ths) */

	uint8_t wrap_bottom; /* address executed after reaching wrap_top */
	uint8_t wrap_top;    /* address that triggers the wrap */

	uint8_t jmp_pin;       /* pin used by the JMP PIN condition */
	bool sideset_pindirs;  /* side-set targets pin directions, not values */
	bool sideset_optional; /* side-set is optional (extra bit as enable) */

	bool in_shift_right;    /* shift ISR right (LSB first) when true */
	bool out_shift_right;   /* shift OSR right (LSB first) when true */
	bool autopush;          /* push the ISR when push_threshold is reached */
	bool autopull;          /* pull the OSR when pull_threshold is reached */
	uint8_t push_threshold; /* ISR bit count for push (0 = 32) */
	uint8_t pull_threshold; /* OSR bit count for pull (0 = 32) */

	uint8_t out_base;      /* first pin asserted by OUT */
	uint8_t out_count;     /* pins asserted by OUT (0..32) */
	uint8_t set_base;      /* first pin asserted by SET */
	uint8_t set_count;     /* pins asserted by SET (0..5) */
	uint8_t sideset_base;  /* first pin asserted by side-set */
	uint8_t sideset_count; /* side-set bits, incl. optional enable (0..5) */
	uint8_t in_base;       /* pin corresponding to IN bit 0 */

	uint8_t tx_fifo_threshold; /* TX FIFO interrupt/DMA threshold (0..7) */
	uint8_t rx_fifo_threshold; /* RX FIFO interrupt/DMA threshold (0..7) */
};

/* Populate a state-machine configuration with hardware-reset defaults. */
void pec_bflb_sm_config_default(struct pec_bflb_sm_config *config);

/*
 * Load a program into the shared instruction memory, returning the load address
 * in *offset. Returns 0, or -ENOMEM/-EBUSY/-EINVAL on failure.
 */
int pec_bflb_add_program(const struct device *dev, const struct pec_bflb_program *prog,
			 uint8_t *offset);

/* Release the instruction-memory region used by a program. */
void pec_bflb_remove_program(const struct device *dev, const struct pec_bflb_program *prog,
			     uint8_t offset);

/*
 * Initialise state machine sm (0..3) and set its program counter to initial_pc.
 * The state machine is left disabled; call pec_bflb_sm_set_enabled() to run it.
 * Returns 0 or -EINVAL.
 */
int pec_bflb_sm_init(const struct device *dev, uint8_t sm, uint8_t initial_pc,
		     const struct pec_bflb_sm_config *config);

/* Enable or disable a state machine. */
void pec_bflb_sm_set_enabled(const struct device *dev, uint8_t sm, bool enable);

/* Restart a state machine's internal state (shift counters, etc.). */
void pec_bflb_sm_restart(const struct device *dev, uint8_t sm);

/* Restart a state machine's clock divider, realigning its phase. */
void pec_bflb_sm_clkdiv_restart(const struct device *dev, uint8_t sm);

/* Set the clock divider of a state machine (div_int 0 means 65536). */
void pec_bflb_sm_set_clkdiv(const struct device *dev, uint8_t sm, uint16_t div_int,
			    uint8_t div_frac);

/* Make a state machine execute an instruction immediately, out of band. */
void pec_bflb_sm_exec(const struct device *dev, uint8_t sm, uint16_t instr);

/* Return true if the most recent pec_bflb_sm_exec() instruction is stalled. */
bool pec_bflb_sm_is_exec_stalled(const struct device *dev, uint8_t sm);

/* Return true if the TX FIFO of a state machine is full. */
bool pec_bflb_sm_tx_fifo_full(const struct device *dev, uint8_t sm);

/* Return true if the RX FIFO of a state machine is empty. */
bool pec_bflb_sm_rx_fifo_empty(const struct device *dev, uint8_t sm);

/* Push a word to a state machine's TX FIFO. Returns 0 or -EBUSY if full. */
int pec_bflb_sm_put(const struct device *dev, uint8_t sm, uint32_t data);

/* Push a word to a state machine's TX FIFO, blocking until space frees. */
void pec_bflb_sm_put_blocking(const struct device *dev, uint8_t sm, uint32_t data);

/* Pop a word from a state machine's RX FIFO. Returns 0 or -EAGAIN if empty. */
int pec_bflb_sm_get(const struct device *dev, uint8_t sm, uint32_t *data);

/* Pop a word from a state machine's RX FIFO, blocking until one arrives. */
uint32_t pec_bflb_sm_get_blocking(const struct device *dev, uint8_t sm);

/* Clear both FIFOs of a state machine. */
void pec_bflb_sm_clear_fifos(const struct device *dev, uint8_t sm);

/* Claim a free state machine; returns its index or -EBUSY if none are free. */
int pec_bflb_claim_sm(const struct device *dev);

/* Release a state machine previously claimed with pec_bflb_claim_sm(). */
void pec_bflb_unclaim_sm(const struct device *dev, uint8_t sm);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_PEC_BFLB_PEC_BFLB_H_ */
