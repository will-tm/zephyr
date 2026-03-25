# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BMV080)
  zephyr_include_directories(
    ${ZEPHYR_HAL_BOSCH_MODULE_DIR}/bmv080/include
  )

  set(BMV080_LIB_DIR ${ZEPHYR_HAL_BOSCH_MODULE_DIR}/bmv080/lib)

  # Select the correct library variant based on CPU architecture
  if(CONFIG_CPU_CORTEX_M0PLUS)
    set(BMV080_ARCH_DIR ${BMV080_LIB_DIR}/arm_cortex_m0plus)
  elseif(CONFIG_CPU_CORTEX_M4 AND CONFIG_FPU)
    set(BMV080_ARCH_DIR ${BMV080_LIB_DIR}/arm_cortex_m4f)
  elseif(CONFIG_CPU_CORTEX_M4)
    set(BMV080_ARCH_DIR ${BMV080_LIB_DIR}/arm_cortex_m4)
  elseif(CONFIG_CPU_CORTEX_M7 AND CONFIG_FPU)
    set(BMV080_ARCH_DIR ${BMV080_LIB_DIR}/arm_cortex_m7f)
  elseif(CONFIG_CPU_CORTEX_M33 AND CONFIG_FPU)
    set(BMV080_ARCH_DIR ${BMV080_LIB_DIR}/arm_cortex_m33f)
  elseif(CONFIG_CPU_CORTEX_M33)
    set(BMV080_ARCH_DIR ${BMV080_LIB_DIR}/arm_cortex_m33)
  elseif(CONFIG_SOC_SERIES_ESP32)
    set(BMV080_ARCH_DIR ${BMV080_LIB_DIR}/xtensa_esp32)
  elseif(CONFIG_SOC_SERIES_ESP32S2)
    set(BMV080_ARCH_DIR ${BMV080_LIB_DIR}/xtensa_esp32s2)
  elseif(CONFIG_SOC_SERIES_ESP32S3)
    set(BMV080_ARCH_DIR ${BMV080_LIB_DIR}/xtensa_esp32s3)
  elseif(CONFIG_RISCV AND CONFIG_CPU_HAS_FPU)
    set(BMV080_ARCH_DIR ${BMV080_LIB_DIR}/riscv_rv32imafc)
  elseif(CONFIG_RISCV)
    set(BMV080_ARCH_DIR ${BMV080_LIB_DIR}/riscv_rv32imc)
  else()
    message(FATAL_ERROR "BMV080: no pre-compiled library available for this architecture")
  endif()

  message(STATUS "BMV080 SDK library: ${BMV080_ARCH_DIR}")

  if(NOT CONFIG_BUILD_ONLY_NO_BLOBS)
    zephyr_link_libraries(
      ${BMV080_ARCH_DIR}/lib_bmv080.a
      ${BMV080_ARCH_DIR}/lib_postProcessor.a
    )
  endif()
endif()
