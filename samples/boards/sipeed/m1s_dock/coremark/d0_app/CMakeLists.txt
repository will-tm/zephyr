cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(coremark_d0)

set(COREMARK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src)

set(COREMARK_SOURCES
  ${COREMARK_DIR}/core_main.c
  ${COREMARK_DIR}/core_list_join.c
  ${COREMARK_DIR}/core_matrix.c
  ${COREMARK_DIR}/core_state.c
  ${COREMARK_DIR}/core_util.c
  ${COREMARK_DIR}/core_portme.c
)

target_sources(app PRIVATE src/main.c ${COREMARK_SOURCES})
target_include_directories(app PRIVATE ${COREMARK_DIR})

set_source_files_properties(${COREMARK_SOURCES} PROPERTIES
  COMPILE_DEFINITIONS "ITERATIONS=0;PERFORMANCE_RUN=1;main=coremark_run;CORE_NAME=\"[D0] \""
  COMPILE_OPTIONS "-O2"
)
