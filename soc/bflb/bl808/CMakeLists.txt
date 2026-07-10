# Copyright The Zephyr Project Contributors
#
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_sources(soc.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")

if(CONFIG_WIFI_BFLB)
  zephyr_include_directories(
    ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/${SOC_SERIES}/wifi
    ${ZEPHYR_BASE}/drivers/wifi/bflb/wifi4
    wifi
  )
  zephyr_sources(
    bflb_rf.c
    wifi/bflb_wifi_platform.c
    wifi/bflb_wifi_blob.c
    wifi/bflb_wifi_tx.c
  )

  zephyr_linker_sources(SECTIONS wifi/bflb_wifi_sections.ld)
endif()
