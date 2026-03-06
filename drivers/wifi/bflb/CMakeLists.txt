# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_WIFI_BFLB)
  zephyr_library()
  zephyr_library_sources(bflb_wifi_drv.c)
  zephyr_include_directories(
    ${ZEPHYR_BASE}/soc/bflb/bl808x/wifi/include
  )
endif()
