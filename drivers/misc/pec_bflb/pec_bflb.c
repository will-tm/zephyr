/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Driver for the Bouffalo Lab PEC (Programmable Engine/Controller) found on the
 * BL61x SoCs. PEC is a programmable I/O state-machine block. The
 * register-access sequences below were validated by disassembling the vendor's
 * prebuilt libpec_instance.a and cross-checking against the SoC register map.
 */

#define DT_DRV_COMPAT bflb_bl61x_pec

#include <zephyr/device.h>
#include <zephyr/drivers/misc/pec_bflb/pec_bflb.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pec_bflb, CONFIG_PEC_BFLB_LOG_LEVEL);

#include <bflb_soc.h>
#include <glb_reg.h>
#include <pec_reg.h>

/* AHB clock-gate bit for PEC in GLB_CGEN_CFG2 (see GLB_PER_Clock_UnGate). */
#define PEC_BFLB_CGEN2_PEC_POS 25U

/* Per-state-machine register block layout: SMn = SM0 + n * stride. */
#define PEC_BFLB_SM_STRIDE 0x18U

/* Spacing of the per-SM FIFO data registers and of the instruction-memory
 * words (both are mapped on 32-bit word boundaries).
 */
#define PEC_BFLB_REG_WORD 4U

struct pec_bflb_config {
	uint32_t base;
	const struct pinctrl_dev_config *pincfg;
};

struct pec_bflb_data {
	struct k_spinlock lock;
	uint32_t imem_used; /* bitmap of used instruction-memory slots */
	uint8_t sm_claimed; /* bitmap of claimed state machines */
};

/* Register helpers */

static inline uint32_t pec_rd(const struct device *dev, uint32_t off)
{
	const struct pec_bflb_config *cfg = dev->config;

	return sys_read32(cfg->base + off);
}

static inline void pec_wr(const struct device *dev, uint32_t off, uint32_t val)
{
	const struct pec_bflb_config *cfg = dev->config;

	sys_write32(val, cfg->base + off);
}

static inline uint32_t pec_sm_reg_off(uint8_t sm, uint32_t sm0_off)
{
	return sm0_off + ((uint32_t)sm * PEC_BFLB_SM_STRIDE);
}

static inline uint32_t pec_sm_rd(const struct device *dev, uint8_t sm, uint32_t sm0_off)
{
	return pec_rd(dev, pec_sm_reg_off(sm, sm0_off));
}

static inline void pec_sm_wr(const struct device *dev, uint8_t sm, uint32_t sm0_off, uint32_t val)
{
	pec_wr(dev, pec_sm_reg_off(sm, sm0_off), val);
}

/* Build a register field value: (val << pos) masked to the field. */
static inline uint32_t pec_field(uint32_t val, uint32_t pos, uint32_t msk)
{
	return (val << pos) & msk;
}

/* Clock */

static void pec_bflb_clock_enable(void)
{
	uint32_t tmp;

	/* Ungate the PEC AHB clock. */
	tmp = sys_read32(GLB_BASE + GLB_CGEN_CFG2_OFFSET);
	tmp |= BIT(PEC_BFLB_CGEN2_PEC_POS);
	sys_write32(tmp, GLB_BASE + GLB_CGEN_CFG2_OFFSET);

	/* Configure the PEC functional clock: source MUXPLL 160 MHz, divider 0.
	 * Mirrors GLB_Set_PEC_CLK(): clear enable, program div/sel, set enable.
	 */
	tmp = sys_read32(GLB_BASE + GLB_PEC_CFG0_OFFSET);
	tmp &= GLB_PEC_CLK_EN_UMSK;
	sys_write32(tmp, GLB_BASE + GLB_PEC_CFG0_OFFSET);

	tmp = sys_read32(GLB_BASE + GLB_PEC_CFG0_OFFSET);
	tmp &= GLB_PEC_CLK_DIV_UMSK & GLB_PEC_CLK_SEL_UMSK;
	sys_write32(tmp, GLB_BASE + GLB_PEC_CFG0_OFFSET);

	tmp = sys_read32(GLB_BASE + GLB_PEC_CFG0_OFFSET);
	tmp |= GLB_PEC_CLK_EN_MSK;
	sys_write32(tmp, GLB_BASE + GLB_PEC_CFG0_OFFSET);
}

/* Program memory */

/*
 * Relocate a single instruction when loaded at a non-zero offset: JMP targets
 * are absolute 5-bit addresses, so add the load offset to the address field.
 * Keeps position-independent programs relocatable.
 */
static uint16_t pec_bflb_instr_fixup(uint16_t instr, uint8_t offset)
{
	if ((instr & PEC_BFLB_INSTR_OP_MASK) == PEC_BFLB_OP_JMP) {
		instr = (uint16_t)((instr & (uint16_t)~PEC_BFLB_INSTR_LO_MASK) |
				   (uint16_t)((instr + offset) & PEC_BFLB_INSTR_LO_MASK));
	}

	return instr;
}

static void pec_bflb_write_instr(const struct device *dev, uint8_t addr, uint16_t code)
{
	pec_wr(dev, PEC_INSTR_MEM0_OFFSET + ((uint32_t)addr * PEC_BFLB_REG_WORD), code);
}

int pec_bflb_add_program(const struct device *dev, const struct pec_bflb_program *prog,
			 uint8_t *offset)
{
	struct pec_bflb_data *data = dev->data;
	bool relocatable;
	uint32_t mask;
	int found = -1;

	if ((prog == NULL) || (offset == NULL) || (prog->instructions == NULL) ||
	    (prog->length == 0U) || (prog->length > PEC_BFLB_INSTRUCTION_COUNT)) {
		return -EINVAL;
	}

	relocatable = (prog->origin < 0);
	mask = (prog->length == PEC_BFLB_INSTRUCTION_COUNT) ? UINT32_MAX
							    : (BIT(prog->length) - 1U);

	K_SPINLOCK(&data->lock) {
		if (prog->origin >= 0) {
			uint8_t o = (uint8_t)prog->origin;

			if (((uint32_t)o + prog->length <= PEC_BFLB_INSTRUCTION_COUNT) &&
			    ((data->imem_used & (mask << o)) == 0U)) {
				found = (int)o;
			}
		} else {
			for (int o = (int)PEC_BFLB_INSTRUCTION_COUNT - (int)prog->length;
			     o >= 0; o--) {
				if ((data->imem_used & (mask << (uint32_t)o)) == 0U) {
					found = o;
					break;
				}
			}
		}

		if (found >= 0) {
			for (uint8_t i = 0U; i < prog->length; i++) {
				uint16_t code = prog->instructions[i];

				if (relocatable) {
					code = pec_bflb_instr_fixup(code, (uint8_t)found);
				}
				pec_bflb_write_instr(dev, (uint8_t)((uint32_t)found + i), code);
			}
			data->imem_used |= (mask << (uint32_t)found);
		}
	}

	if (found < 0) {
		return (prog->origin >= 0) ? -EBUSY : -ENOMEM;
	}

	*offset = (uint8_t)found;

	return 0;
}

void pec_bflb_remove_program(const struct device *dev, const struct pec_bflb_program *prog,
			     uint8_t offset)
{
	struct pec_bflb_data *data = dev->data;
	uint32_t mask;

	if ((prog == NULL) || (prog->length == 0U) ||
	    (prog->length > PEC_BFLB_INSTRUCTION_COUNT)) {
		return;
	}

	mask = (prog->length == PEC_BFLB_INSTRUCTION_COUNT) ? UINT32_MAX
							    : (BIT(prog->length) - 1U);

	K_SPINLOCK(&data->lock) {
		data->imem_used &= ~(mask << offset);
	}
}

/* State-machine control */

void pec_bflb_sm_set_enabled(const struct device *dev, uint8_t sm, bool enable)
{
	struct pec_bflb_data *data = dev->data;

	__ASSERT_NO_MSG(sm < PEC_BFLB_NUM_STATE_MACHINES);

	K_SPINLOCK(&data->lock) {
		uint32_t ctrl = pec_rd(dev, PEC_CTRL_OFFSET);

		if (enable) {
			ctrl |= BIT(PEC_CR_SM_EN_POS + sm);
		} else {
			ctrl &= ~BIT(PEC_CR_SM_EN_POS + sm);
		}
		pec_wr(dev, PEC_CTRL_OFFSET, ctrl);
	}
}

void pec_bflb_sm_restart(const struct device *dev, uint8_t sm)
{
	struct pec_bflb_data *data = dev->data;

	__ASSERT_NO_MSG(sm < PEC_BFLB_NUM_STATE_MACHINES);

	/* SM_RESET is a self-clearing strobe (one bit per SM). */
	K_SPINLOCK(&data->lock) {
		pec_wr(dev, PEC_CTRL_OFFSET,
		       pec_rd(dev, PEC_CTRL_OFFSET) | BIT(PEC_CR_SM_RESET_POS + sm));
	}
}

void pec_bflb_sm_clkdiv_restart(const struct device *dev, uint8_t sm)
{
	struct pec_bflb_data *data = dev->data;

	__ASSERT_NO_MSG(sm < PEC_BFLB_NUM_STATE_MACHINES);

	/* CLKDIV_RESET is a self-clearing strobe (one bit per SM). */
	K_SPINLOCK(&data->lock) {
		pec_wr(dev, PEC_CTRL_OFFSET,
		       pec_rd(dev, PEC_CTRL_OFFSET) | BIT(PEC_CR_CLKDIV_RESET_POS + sm));
	}
}

void pec_bflb_sm_set_clkdiv(const struct device *dev, uint8_t sm, uint16_t div_int,
			    uint8_t div_frac)
{
	uint32_t val = pec_field(div_int, PEC_CR_SM0_INT_POS, PEC_CR_SM0_INT_MSK) |
		       pec_field(div_frac, PEC_CR_SM0_FRAC_POS, PEC_CR_SM0_FRAC_MSK);

	__ASSERT_NO_MSG(sm < PEC_BFLB_NUM_STATE_MACHINES);

	pec_sm_wr(dev, sm, PEC_SM0_CLKDIV_OFFSET, val);
}

void pec_bflb_sm_exec(const struct device *dev, uint8_t sm, uint16_t instr)
{
	__ASSERT_NO_MSG(sm < PEC_BFLB_NUM_STATE_MACHINES);

	pec_sm_wr(dev, sm, PEC_SM0_INSTR_OFFSET, instr);
}

bool pec_bflb_sm_is_exec_stalled(const struct device *dev, uint8_t sm)
{
	__ASSERT_NO_MSG(sm < PEC_BFLB_NUM_STATE_MACHINES);

	return (pec_sm_rd(dev, sm, PEC_SM0_EXECCTRL_OFFSET) & PEC_ST_SM0_EXEC_STALLED_MSK) != 0U;
}

static void pec_bflb_sm_set_pc_origin(const struct device *dev, uint8_t sm, uint8_t pc)
{
	struct pec_bflb_data *data = dev->data;
	/* CTRL PC_ORIGIN: a PEC_CR_SM0_PC_ORIGIN_LEN-bit field per SM. */
	uint32_t shift = (uint32_t)PEC_CR_SM0_PC_ORIGIN_POS +
			 ((uint32_t)sm * PEC_CR_SM0_PC_ORIGIN_LEN);
	uint32_t mask = (BIT(PEC_CR_SM0_PC_ORIGIN_LEN) - 1U) << shift;

	K_SPINLOCK(&data->lock) {
		uint32_t ctrl = pec_rd(dev, PEC_CTRL_OFFSET);

		ctrl = (ctrl & ~mask) | (((uint32_t)pc << shift) & mask);
		pec_wr(dev, PEC_CTRL_OFFSET, ctrl);
	}
}

void pec_bflb_sm_config_default(struct pec_bflb_sm_config *config)
{
	*config = (struct pec_bflb_sm_config){
		.clkdiv_int = 1U,
		.clkdiv_frac = 0U,
		.wrap_bottom = 0U,
		.wrap_top = PEC_BFLB_INSTRUCTION_COUNT - 1U,
		/* Hardware reset defaults: shift left, thresholds of 32. */
		.in_shift_right = false,
		.out_shift_right = false,
		.push_threshold = 0U, /* 0 encodes a threshold of 32 */
		.pull_threshold = 0U,
	};
}

int pec_bflb_sm_init(const struct device *dev, uint8_t sm, uint8_t initial_pc,
		     const struct pec_bflb_sm_config *config)
{
	uint32_t execctrl;
	uint32_t shiftctrl;
	uint32_t pinctrl;

	if ((sm >= PEC_BFLB_NUM_STATE_MACHINES) || (config == NULL)) {
		return -EINVAL;
	}

	pec_bflb_sm_set_enabled(dev, sm, false);

	pec_bflb_sm_set_clkdiv(dev, sm, config->clkdiv_int, config->clkdiv_frac);

	execctrl = pec_field(config->wrap_bottom, PEC_CR_SM0_WRAP_BOTTOM_POS,
			     PEC_CR_SM0_WRAP_BOTTOM_MSK) |
		   pec_field(config->wrap_top, PEC_CR_SM0_WRAP_TOP_POS, PEC_CR_SM0_WRAP_TOP_MSK) |
		   pec_field(config->jmp_pin, PEC_CR_SM0_JMP_PIN_POS, PEC_CR_SM0_JMP_PIN_MSK);
	if (config->sideset_pindirs) {
		execctrl |= PEC_CR_SM0_SIDE_PINDIR_MSK;
	}
	if (config->sideset_optional) {
		execctrl |= PEC_CR_SM0_SIDE_EN_MSK;
	}
	pec_sm_wr(dev, sm, PEC_SM0_EXECCTRL_OFFSET, execctrl);

	shiftctrl = pec_field(config->tx_fifo_threshold, PEC_CR_SM0_TXF_THR_POS,
			      PEC_CR_SM0_TXF_THR_MSK) |
		    pec_field(config->rx_fifo_threshold, PEC_CR_SM0_RXF_THR_POS,
			      PEC_CR_SM0_RXF_THR_MSK) |
		    pec_field(config->push_threshold, PEC_CR_SM0_PUSH_THRESH_POS,
			      PEC_CR_SM0_PUSH_THRESH_MSK) |
		    pec_field(config->pull_threshold, PEC_CR_SM0_PULL_THRESH_POS,
			      PEC_CR_SM0_PULL_THRESH_MSK);
	if (config->autopush) {
		shiftctrl |= PEC_CR_SM0_AUTOPUSH_MSK;
	}
	if (config->autopull) {
		shiftctrl |= PEC_CR_SM0_AUTOPULL_MSK;
	}
	if (config->in_shift_right) {
		shiftctrl |= PEC_CR_SM0_IN_SHIFTDIR_MSK;
	}
	if (config->out_shift_right) {
		shiftctrl |= PEC_CR_SM0_OUT_SHIFTDIR_MSK;
	}
	pec_sm_wr(dev, sm, PEC_SM0_SHIFTCTRL_OFFSET, shiftctrl);

	pinctrl = pec_field(config->out_base, PEC_CR_SM0_OUT_BASE_POS, PEC_CR_SM0_OUT_BASE_MSK) |
		  pec_field(config->set_base, PEC_CR_SM0_SET_BASE_POS, PEC_CR_SM0_SET_BASE_MSK) |
		  pec_field(config->sideset_base, PEC_CR_SM0_SIDESET_BASE_POS,
			    PEC_CR_SM0_SIDESET_BASE_MSK) |
		  pec_field(config->in_base, PEC_CR_SM0_IN_BASE_POS, PEC_CR_SM0_IN_BASE_MSK) |
		  pec_field(config->out_count, PEC_CR_SM0_OUT_COUNT_POS, PEC_CR_SM0_OUT_COUNT_MSK) |
		  pec_field(config->set_count, PEC_CR_SM0_SET_COUNT_POS, PEC_CR_SM0_SET_COUNT_MSK) |
		  pec_field(config->sideset_count, PEC_CR_SM0_SIDESET_COUNT_POS,
			    PEC_CR_SM0_SIDESET_COUNT_MSK);
	pec_sm_wr(dev, sm, PEC_SM0_PINCTRL_OFFSET, pinctrl);

	pec_bflb_sm_set_pc_origin(dev, sm, initial_pc);
	pec_bflb_sm_clear_fifos(dev, sm);
	pec_bflb_sm_restart(dev, sm);
	pec_bflb_sm_clkdiv_restart(dev, sm);

	return 0;
}

/* FIFOs */

bool pec_bflb_sm_tx_fifo_full(const struct device *dev, uint8_t sm)
{
	__ASSERT_NO_MSG(sm < PEC_BFLB_NUM_STATE_MACHINES);

	return (pec_rd(dev, PEC_FSTAT_OFFSET) & BIT(PEC_ST_TXFULL_POS + sm)) != 0U;
}

bool pec_bflb_sm_rx_fifo_empty(const struct device *dev, uint8_t sm)
{
	__ASSERT_NO_MSG(sm < PEC_BFLB_NUM_STATE_MACHINES);

	return (pec_rd(dev, PEC_FSTAT_OFFSET) & BIT(PEC_ST_RXEMPTY_POS + sm)) != 0U;
}

int pec_bflb_sm_put(const struct device *dev, uint8_t sm, uint32_t data)
{
	if (pec_bflb_sm_tx_fifo_full(dev, sm)) {
		return -EBUSY;
	}

	pec_wr(dev, PEC_TXF0_OFFSET + ((uint32_t)sm * PEC_BFLB_REG_WORD), data);

	return 0;
}

void pec_bflb_sm_put_blocking(const struct device *dev, uint8_t sm, uint32_t data)
{
	/* Busy-wait: the caller must ensure the SM drains the FIFO, otherwise
	 * this never returns.
	 */
	while (pec_bflb_sm_tx_fifo_full(dev, sm)) {
	}

	pec_wr(dev, PEC_TXF0_OFFSET + ((uint32_t)sm * PEC_BFLB_REG_WORD), data);
}

int pec_bflb_sm_get(const struct device *dev, uint8_t sm, uint32_t *data)
{
	__ASSERT_NO_MSG(data != NULL);

	if (pec_bflb_sm_rx_fifo_empty(dev, sm)) {
		return -EAGAIN;
	}

	*data = pec_rd(dev, PEC_RXF0_OFFSET + ((uint32_t)sm * PEC_BFLB_REG_WORD));

	return 0;
}

uint32_t pec_bflb_sm_get_blocking(const struct device *dev, uint8_t sm)
{
	/* Busy-wait: the caller must ensure the SM produces data, otherwise
	 * this never returns.
	 */
	while (pec_bflb_sm_rx_fifo_empty(dev, sm)) {
	}

	return pec_rd(dev, PEC_RXF0_OFFSET + ((uint32_t)sm * PEC_BFLB_REG_WORD));
}

void pec_bflb_sm_clear_fifos(const struct device *dev, uint8_t sm)
{
	struct pec_bflb_data *data = dev->data;

	__ASSERT_NO_MSG(sm < PEC_BFLB_NUM_STATE_MACHINES);

	/* TXF_CLR / RXF_CLR are self-clearing strobes within SHIFTCTRL, which
	 * also holds configuration, so guard the read-modify-write.
	 */
	K_SPINLOCK(&data->lock) {
		uint32_t shiftctrl = pec_sm_rd(dev, sm, PEC_SM0_SHIFTCTRL_OFFSET);

		pec_sm_wr(dev, sm, PEC_SM0_SHIFTCTRL_OFFSET,
			  shiftctrl | PEC_CR_SM0_TXF_CLR_MSK | PEC_CR_SM0_RXF_CLR_MSK);
	}
}

/* Allocation */

int pec_bflb_claim_sm(const struct device *dev)
{
	struct pec_bflb_data *data = dev->data;
	int sm = -EBUSY;

	K_SPINLOCK(&data->lock) {
		for (uint8_t i = 0U; i < PEC_BFLB_NUM_STATE_MACHINES; i++) {
			if ((data->sm_claimed & BIT(i)) == 0U) {
				data->sm_claimed |= BIT(i);
				sm = (int)i;
				break;
			}
		}
	}

	return sm;
}

void pec_bflb_unclaim_sm(const struct device *dev, uint8_t sm)
{
	struct pec_bflb_data *data = dev->data;

	__ASSERT_NO_MSG(sm < PEC_BFLB_NUM_STATE_MACHINES);

	K_SPINLOCK(&data->lock) {
		data->sm_claimed &= ~BIT(sm);
	}
}

/* Init */

static int pec_bflb_init(const struct device *dev)
{
	const struct pec_bflb_config *cfg = dev->config;
	int ret;

	pec_bflb_clock_enable();

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("failed to apply pinctrl (%d)", ret);
		return ret;
	}

	/* Disable all state machines and mask all interrupts. */
	pec_wr(dev, PEC_CTRL_OFFSET, 0U);
	pec_wr(dev, PEC_IRQ0_INTE_OFFSET, 0U);
	pec_wr(dev, PEC_IRQ1_INTE_OFFSET, 0U);

	return 0;
}

#define PEC_BFLB_INIT(n)							\
	PINCTRL_DT_INST_DEFINE(n);						\
	static const struct pec_bflb_config pec_bflb_config_##n = {		\
		.base = DT_INST_REG_ADDR(n),					\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
	};									\
	static struct pec_bflb_data pec_bflb_data_##n;				\
	DEVICE_DT_INST_DEFINE(n, pec_bflb_init, NULL, &pec_bflb_data_##n,	\
			      &pec_bflb_config_##n, POST_KERNEL,		\
			      CONFIG_PEC_BFLB_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PEC_BFLB_INIT)
