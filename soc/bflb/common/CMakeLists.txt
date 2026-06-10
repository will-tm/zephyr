# Copyright (c) 2021-2025 ATL Electronics
#
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

if(CONFIG_SOC_SERIES_BL60X OR CONFIG_SOC_SERIES_BL70X OR CONFIG_SOC_SERIES_BL70XL)
zephyr_include_directories(e24)
zephyr_sources(
e24/soc_irq_privileged.c
e24/soc.c
)

if(CONFIG_PM)
  zephyr_code_relocate(FILES e24/soc_irq_privileged.c LOCATION ITCM NOKEEP)
endif()
endif()

zephyr_sources(
power.c
)

# Header is referenced by the per-SoC PDS driver (for HBN-RTC macros) even when
# the shared sources are not compiled, so keep the include path unconditional.
zephyr_include_directories(pds)

if(CONFIG_BFLB_PDS)
  zephyr_sources(
    pds/pds_bflb_time.c
  )
endif()
