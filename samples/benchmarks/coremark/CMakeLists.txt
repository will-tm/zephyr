cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(coremark)

target_sources(app PRIVATE
  src/core_main.c
  src/core_list_join.c
  src/core_matrix.c
  src/core_state.c
  src/core_util.c
  src/core_portme.c
)

target_include_directories(app PRIVATE src)


target_compile_definitions(app PRIVATE
  ITERATIONS=0
  PERFORMANCE_RUN=1
)

# Optimise for speed
target_compile_options(app PRIVATE -O2)
