# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_WIFI_BFLB)

zephyr_library()
zephyr_library_link_libraries_ifdef(CONFIG_MBEDTLS mbedTLS)

zephyr_library_include_directories(inc)
zephyr_library_compile_definitions(
  __ZEPHYR__=1
)
# Force-include wifi_cfg.h and suppress SDK code warnings
zephyr_library_compile_options(
  -include ${CMAKE_CURRENT_SOURCE_DIR}/inc/wifi_cfg.h
  -Wno-error -w -fcommon
)
# Zephyr WiFi mgmt driver + wrapper/shim code
zephyr_library_sources(
  bflb_wifi_drv.c
  bl_os_adapter.c
  bl_tx.c
  wifi_fw_data.c
  wifi_hosal.c
  wifi_stubs.c
  wpa_supplicant.c
)

# Host driver (pinevoice SDK sources — minimal modifications)
zephyr_library_sources(
  src/bl_cmds.c
  src/bl_irqs.c
  src/bl_main.c
  src/bl_mod_params.c
  src/bl_msg_tx.c
  src/bl_platform.c
  src/bl_rx.c
  src/bl_utils.c
  src/ipc_host.c
  src/utils_crc.c
  src/utils_tlv_bl.c
)

# Link binary blobs
set(BLOB_DIR ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/zephyr/blobs/lib/bl808x)

zephyr_library_import(wifi ${BLOB_DIR}/libbl606p_firmware_prebuild.a)
zephyr_library_import(bl606p_phyrf ${BLOB_DIR}/libbl606p_phyrf.a)

# Linker symbols required by firmware blob for static config entries
zephyr_linker_sources(DATA_SECTIONS SORT_KEY wifi
  ${CMAKE_CURRENT_SOURCE_DIR}/wifi_static_cfg.ld)

endif()
