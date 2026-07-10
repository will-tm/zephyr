/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BL808 RF parameter setup for the BL606P PHY/RF blob (libbl606p_phyrf.a).
 *
 * The WiFi firmware performs the RF calibration itself (wifi_main calls
 * the PHY's rf_init); the host only feeds the factory RF parameters the
 * vendor rfparam layer would read from flash: the XTAL capacitor codes
 * (AON register) and the TX power-vs-rate tables (trpc_update_power_*).
 * The values come from the wifi0 devicetree node, mirroring the vendor
 * bl_factory_params data.
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <bflb_soc.h>
#include <aon_reg.h>

#define WIFI_NODE DT_INST(0, bflb_bl808_wifi)

#define AON_XTAL_CFG_ADDR (AON_BASE + AON_XTAL_CFG_OFFSET)
#define CAPCODE_IN_POS    22U
#define CAPCODE_OUT_POS   16U
#define CAPCODE_MSK       0x3FU

#define BFLB_PWR_11B_LEN 4
#define BFLB_PWR_11G_LEN 8
#define BFLB_PWR_11N_LEN 8

#define DT_COPY_I8(node, prop, dst, n)                                                             \
	do {                                                                                       \
		static const int32_t _v[] = {                                                      \
			DT_FOREACH_PROP_ELEM_SEP(node, prop,              \
				DT_PROP_BY_IDX, (,))};                   \
		BUILD_ASSERT(ARRAY_SIZE(_v) == (n), #prop " must have " #n " entries");            \
		for (int _i = 0; _i < (n); _i++) {                                                 \
			(dst)[_i] = (int8_t)_v[_i];                                                \
		}                                                                                  \
	} while (0)

/* TX power tables inside the PHY blob (dBm, one entry per rate). */
extern void trpc_update_power_11b(int8_t *pwr);
extern void trpc_update_power_11g(int8_t *pwr);
extern void trpc_update_power_11n(int8_t *pwr);

static void capcode_set(uint8_t capcode_in, uint8_t capcode_out);

static void capcode_set(uint8_t capcode_in, uint8_t capcode_out)
{
	uint32_t val = sys_read32(AON_XTAL_CFG_ADDR);

	val &= ~((CAPCODE_MSK << CAPCODE_IN_POS) | (CAPCODE_MSK << CAPCODE_OUT_POS));
	val |= ((uint32_t)(capcode_in & CAPCODE_MSK) << CAPCODE_IN_POS) |
	       ((uint32_t)(capcode_out & CAPCODE_MSK) << CAPCODE_OUT_POS);
	sys_write32(val, AON_XTAL_CFG_ADDR);
}

int bflb_rf_init(void)
{
#if DT_NODE_HAS_PROP(WIFI_NODE, pwr_table_11b)
	{
		static int8_t pwr_11b[BFLB_PWR_11B_LEN];

		DT_COPY_I8(WIFI_NODE, pwr_table_11b, pwr_11b, BFLB_PWR_11B_LEN);
		trpc_update_power_11b(pwr_11b);
	}
#endif
#if DT_NODE_HAS_PROP(WIFI_NODE, pwr_table_11g)
	{
		static int8_t pwr_11g[BFLB_PWR_11G_LEN];

		DT_COPY_I8(WIFI_NODE, pwr_table_11g, pwr_11g, BFLB_PWR_11G_LEN);
		trpc_update_power_11g(pwr_11g);
	}
#endif
#if DT_NODE_HAS_PROP(WIFI_NODE, pwr_table_11n_ht20)
	{
		static int8_t pwr_11n[BFLB_PWR_11N_LEN];

		DT_COPY_I8(WIFI_NODE, pwr_table_11n_ht20, pwr_11n, BFLB_PWR_11N_LEN);
		trpc_update_power_11n(pwr_11n);
	}
#endif

#if DT_NODE_HAS_PROP(WIFI_NODE, xtal_capcode_in) && DT_NODE_HAS_PROP(WIFI_NODE, xtal_capcode_out)
	capcode_set(DT_PROP(WIFI_NODE, xtal_capcode_in), DT_PROP(WIFI_NODE, xtal_capcode_out));
#endif

	return 0;
}
