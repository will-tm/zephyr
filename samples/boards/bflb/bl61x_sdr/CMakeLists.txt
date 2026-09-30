# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bl61x_sdr)

target_sources(app PRIVATE
  src/capture.c
  src/dump_ring.c
  src/iq_control.c
  src/iq_format.c
  src/iq_usb.c
  src/main.c
  src/resample.c
  src/rf.c
  src/stream_ring.c
)
