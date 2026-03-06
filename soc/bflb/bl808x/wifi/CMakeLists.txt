# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_WIFI_BL808X)

zephyr_library()
zephyr_library_link_libraries_ifdef(CONFIG_MBEDTLS mbedTLS)

zephyr_library_sources(
  bl_os_adapter.c
  wifi_hosal.c
  utils_crc.c
  utils_tlv_bl.c
  wifi_stubs.c
  ipc_host.c
  bl_platform.c
  bl_irqs.c
  bl_utils.c
  bl_cmds.c
  bl_main.c
  bl_msg_tx.c
  bl_mod_params.c
  bl_tx.c
  wifi_fw_data.c
  wpa_supplicant.c
)

zephyr_include_directories(
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/${SOC_SERIES}/wifi4/driver
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/${SOC_SERIES}/wifi4/firmware
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/${SOC_SERIES}/wifi4/manager/include
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/${SOC_SERIES}/wifi4/wifi_hosal
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/${SOC_SERIES}/wifi4/wifi_bt_coex
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/${SOC_SERIES}/bl_os_adapter
)

# Link binary blobs
set(BLOB_DIR ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/zephyr/blobs/lib/${SOC_SERIES})

zephyr_library_import(wifi ${BLOB_DIR}/libwifi.a)
zephyr_library_import(bl606p_phyrf ${BLOB_DIR}/libbl606p_phyrf.a)

# Linker symbols required by libwifi.a for static config entries
zephyr_linker_sources(DATA_SECTIONS SORT_KEY wifi
  ${CMAKE_CURRENT_SOURCE_DIR}/wifi_static_cfg.ld)

endif() # CONFIG_WIFI_BL808X
