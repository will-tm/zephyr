# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

set(BLOBS_DIR ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/zephyr/blobs/lib/bl808x)

if(CONFIG_WIFI_BFLB AND NOT CONFIG_BUILD_ONLY_NO_BLOBS)
  # BL606P and BL808 are the same silicon; the WiFi MAC firmware and the
  # PHY/RF library ship as BL606P blobs.  --wrap=printf routes the blobs'
  # bare printf through the platform filter (soc/bflb/bl808/wifi).
  zephyr_link_libraries(
    ${BLOBS_DIR}/libbl606p_firmware_prebuild.a
    ${BLOBS_DIR}/libbl606p_phyrf.a
    -Wl,--wrap=printf
  )
endif()
