# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)

# Local WiFi credentials, kept out of the tree
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/credentials.conf)
  list(APPEND EXTRA_CONF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/credentials.conf)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(http_camera)

target_sources(app PRIVATE
  src/main.c
  src/camera.c
  src/http.c
)

set(gen_dir ${ZEPHYR_BINARY_DIR}/include/generated)
generate_inc_file_for_target(app src/index.html ${gen_dir}/index.html.gz.inc --gzip)
