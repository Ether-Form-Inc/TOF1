# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ether Form Inc.
cmake_minimum_required(VERSION 3.15)
include(FetchContent)

# STM  builds the firmware for the STM32G0B1 (needs arm-none-eabi-gcc).
# HOST builds the unit tests for the PC.
set(ARCH "STM" CACHE STRING "Target architecture: STM or HOST")

if(${ARCH} STREQUAL "STM")
  set(CMAKE_TOOLCHAIN_FILE ../stm32-toolchain.cmake CACHE STRING "" FORCE)
  project(can_tof_firmware LANGUAGES C CXX ASM)
  add_subdirectory(stm_generated)
elseif(${ARCH} STREQUAL "HOST")
  project(can_tof_host_tests LANGUAGES C CXX)
  add_subdirectory(tests)
else()
  message(FATAL_ERROR "Specified architecture type not supported. Use \"-DARCH=<STM,HOST>\".")
endif()
