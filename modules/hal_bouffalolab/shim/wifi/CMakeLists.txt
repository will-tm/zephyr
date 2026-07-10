# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_BFLB_WL80211_SHIM wl80211_rtos_port.c wl80211_port.c)
zephyr_sources_ifdef(CONFIG_BFLB_BL602_OS_SHIM bl602_os_port.c)
if(CONFIG_BUILD_ONLY_NO_BLOBS)
  zephyr_sources_ifdef(CONFIG_BFLB_WL80211_SHIM wl80211_stubs.c)
  # The BL606P/BL808 firmware blob exposes the same host interface as the
  # BL602 one; the stub set is shared (types resolve per-SoC via the hal
  # include path).
  if(CONFIG_SOC_SERIES_BL60X OR CONFIG_SOC_SERIES_BL808)
    zephyr_sources(bl602_firmware_stubs.c)
  endif()
endif()
