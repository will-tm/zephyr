/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Drives a square wave on a GPIO using the BL61x PEC, entirely from a
 * state-machine program. Once started, the CPU does no work: the PEC toggles
 * the pin autonomously. Point the PEC pin (see the board overlay and
 * PEC_BLINK_PIN) at an LED to see it blink.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/misc/pec_bflb/pec_bflb.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pec_blink, LOG_LEVEL_INF);

#define PEC_NODE DT_NODELABEL(pec0)

/* PEC virtual pin index == GPIO number; must match the board overlay pinmux.
 * GPIO16 is used here because GPIO0-3 are the JTAG pins on bl618g0.
 */
#define PEC_BLINK_PIN 16

/*
 * Program (loaded at offset 0, wraps 0 -> 5):
 *
 *   0: set  pins, 1          ; drive the pin high
 *   1: set  x, 31
 *   2: jmp  x--, 2  [31]     ; ~1024-cycle high delay
 *   3: set  pins, 0          ; drive the pin low
 *   4: set  x, 31
 *   5: jmp  x--, 5  [31]     ; ~1024-cycle low delay
 *
 * With the clock divider at its maximum (160 MHz / 65536 ~= 2.4 kHz) this
 * produces a blink of roughly 1.2 Hz.
 */
#define PEC_BLINK_PROGRAM_LEN 6U

int main(void)
{
	const struct device *pec = DEVICE_DT_GET(PEC_NODE);
	uint16_t program[PEC_BLINK_PROGRAM_LEN];
	struct pec_bflb_sm_config cfg;
	struct pec_bflb_program prog;
	uint8_t offset;
	int sm;
	int ret;

	if (!device_is_ready(pec)) {
		LOG_ERR("PEC device not ready");
		return -ENODEV;
	}

	program[0] = pec_bflb_encode_set(PEC_BFLB_SET_DEST_PINS, 1);
	program[1] = pec_bflb_encode_set(PEC_BFLB_SET_DEST_X, 31);
	program[2] = pec_bflb_encode_jmp(PEC_BFLB_JMP_X_DEC, 2) | pec_bflb_encode_delay(31);
	program[3] = pec_bflb_encode_set(PEC_BFLB_SET_DEST_PINS, 0);
	program[4] = pec_bflb_encode_set(PEC_BFLB_SET_DEST_X, 31);
	program[5] = pec_bflb_encode_jmp(PEC_BFLB_JMP_X_DEC, 5) | pec_bflb_encode_delay(31);

	prog.instructions = program;
	prog.length = PEC_BLINK_PROGRAM_LEN;
	prog.origin = -1;

	ret = pec_bflb_add_program(pec, &prog, &offset);
	if (ret < 0) {
		LOG_ERR("failed to load program (%d)", ret);
		return ret;
	}

	sm = pec_bflb_claim_sm(pec);
	if (sm < 0) {
		LOG_ERR("no free state machine");
		return sm;
	}

	pec_bflb_sm_config_default(&cfg);
	cfg.clkdiv_int = 0U; /* 0 == maximum divider (65536) */
	cfg.clkdiv_frac = 0U;
	cfg.set_base = PEC_BLINK_PIN;
	cfg.set_count = 1U;
	cfg.wrap_bottom = offset;
	cfg.wrap_top = offset + PEC_BLINK_PROGRAM_LEN - 1U;

	ret = pec_bflb_sm_init(pec, (uint8_t)sm, offset, &cfg);
	if (ret < 0) {
		LOG_ERR("sm_init failed (%d)", ret);
		return ret;
	}

	/* Make the SET pin an output, then let the state machine run on its own. */
	pec_bflb_sm_exec(pec, (uint8_t)sm, pec_bflb_encode_set(PEC_BFLB_SET_DEST_PINDIRS, 1));
	pec_bflb_sm_set_enabled(pec, (uint8_t)sm, true);

	LOG_INF("PEC blinking pin %d from state machine %d (offset %d)", PEC_BLINK_PIN, sm, offset);

	return 0;
}
