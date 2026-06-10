# Copyright The Zephyr Project Contributors
#
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_sources(soc.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")

zephyr_code_relocate_ifdef(CONFIG_UART_BFLB FILES ${ZEPHYR_BASE}/drivers/serial/uart_bflb.c
  LOCATION ITCM NOKEEP)
zephyr_code_relocate_ifdef(CONFIG_RISCV_MACHINE_TIMER LIBRARY drivers__timer LOCATION ITCM NOKEEP)
zephyr_code_relocate_ifdef(CONFIG_PINCTRL_BFLB LIBRARY drivers__pinctrl LOCATION ITCM NOKEEP)
zephyr_code_relocate_ifdef(CONFIG_OTP_BFLB LIBRARY drivers__otp LOCATION ITCM NOKEEP)
zephyr_code_relocate_ifdef(CONFIG_CLOCK_CONTROL_BOUFFALOLAB_BL70X_L
  LIBRARY drivers__clock_control LOCATION ITCM NOKEEP)
zephyr_code_relocate_ifdef(CONFIG_CACHE_BFLB_L1C LIBRARY drivers__cache LOCATION ITCM NOKEEP)
zephyr_code_relocate_ifdef(CONFIG_SOC_FLASH_BFLB FILES ${ZEPHYR_BASE}/drivers/flash/flash_bflb.c
  LOCATION ITCM NOKEEP)

# PDS driver: bflb_pds_itcm.c and bflb_pds_flash.c run with XIP off,
# bflb_pds.c runs from flash. Timekeeping/RC32K policy and the gpio-keys wake
# shim live in soc/bflb/common/pds (shared, SoC-agnostic).
zephyr_sources_ifdef(CONFIG_PM bflb_pds.c bflb_pds_itcm.c bflb_pds_flash.c)
zephyr_code_relocate_ifdef(CONFIG_PM FILES bflb_pds_itcm.c bflb_pds_flash.c
  LOCATION ITCM NOKEEP)

# S2RAM runs inside bflb_pds_system_off / pds_restore, both with XIP off.
zephyr_code_relocate_ifdef(CONFIG_PM_S2RAM
  FILES ${ZEPHYR_BASE}/arch/riscv/core/pm_s2ram.c
        ${ZEPHYR_BASE}/arch/riscv/core/pm_s2ram.S
  LOCATION ITCM NOKEEP)

set(BL70XL_ROMDRIVER_LD ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/ld/bl70xl/romdriver.ld)
if(EXISTS ${BL70XL_ROMDRIVER_LD})
  zephyr_ld_options(-T${BL70XL_ROMDRIVER_LD})
endif()

if(CONFIG_PM)
  set(BL70XL_ROM_MAP_LD ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/ld/bl70xl/rom_map.ld)
  if(EXISTS ${BL70XL_ROM_MAP_LD})
    zephyr_ld_options(-T${BL70XL_ROM_MAP_LD})
  endif()
endif()

zephyr_linker_sources(NOINIT ${CMAKE_CURRENT_SOURCE_DIR}/bflb_noinit.ld)
zephyr_linker_sources(RWDATA ${CMAKE_CURRENT_SOURCE_DIR}/bflb_rwdata.ld)
zephyr_linker_sources(ITCM_SECTION ${CMAKE_CURRENT_SOURCE_DIR}/bflb_itcm.ld)

if(CONFIG_BT_BFLB_BL70XL)
  # The BLE blob's ble_rwdata.ld defines __global_pointer$ which causes the
  # linker to relax address loads into GP-relative instructions. Since the
  # Zephyr CRT does not initialize GP (CONFIG_RISCV_GP is not enabled),
  # these relaxed instructions would crash. Disable linker relaxation so
  # only the blob's own pre-compiled GP-relative code uses GP.
  zephyr_ld_options(-Wl,--no-relax)
  zephyr_linker_sources(SECTIONS bflb_hbn.ld)

  zephyr_sources(
    ble/btblecontroller_port_hw.c
    bflb_platform_shim.c
    bflb_rf.c
    rf_callbacks.c
  )

  if(CONFIG_BL70XL_PDS_S2RAM)
    zephyr_sources(ble/btble_pds.c)
  endif()
endif()
