# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_BFLB_FREERTOS_SHIM freertos_shim.c)
zephyr_sources_ifdef(CONFIG_BFLB_BTBLE_PORT_OS_SHIM btblecontroller_port_os.c)

if(CONFIG_BFLB_ZIGBEE)
  set(ZIGBEE_BLOB_DIR ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/zephyr/blobs/lib/bl70xl/zigbee)

  zephyr_sources(
    zigbee/zigbee_shim.c
    zigbee/cpp_runtime.c
  )

  zephyr_sources(
    zigbee/libstdcpp_shim.cpp
  )

  zephyr_include_directories(
    ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/bl70xl/zigbee
    ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/bl70xl/zigbee/core
    ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/bl70xl/zigbee/zcl
  )

  zephyr_link_libraries(
    ${ZIGBEE_BLOB_DIR}/libzbstack.a
    ${ZIGBEE_BLOB_DIR}/libzbstackex.a
    ${ZIGBEE_BLOB_DIR}/libmacphy154_bl702l.a
    ${ZIGBEE_BLOB_DIR}/libecdsa.a
    ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/zephyr/blobs/lib/bl70xl/libbl702l_rf.a
  )

  # Linker symbols for zigbee stack configuration
  zephyr_linker_sources(SECTIONS zigbee/zigbee_linker.ld)
endif()

