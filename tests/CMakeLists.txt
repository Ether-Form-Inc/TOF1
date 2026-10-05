# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ether Form Inc.
# Unit tests that run on the PC: the configuration rules and the DroneCAN message builder.
include(${CMAKE_SOURCE_DIR}/cmake/dronecan_c.cmake)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_Declare(
  gtest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.15.2
  SOURCE_DIR     ${CMAKE_BINARY_DIR}/gtest
  BINARY_DIR     ${CMAKE_BINARY_DIR}/gtest_build
)
FetchContent_MakeAvailable(gtest)

add_executable(tof_tests
  ${CMAKE_CURRENT_LIST_DIR}/tof_tests/tof_tests.cpp
  ${CANARD_SRC_DIR}/canard.c
  ${DSDLC_SOURCES})

target_compile_features(tof_tests PRIVATE cxx_std_17)

target_include_directories(tof_tests PRIVATE
  ${CMAKE_SOURCE_DIR}/board
  ${CANARD_SRC_DIR}
  ${DSDLC_GEN_DIR}/include)

target_link_libraries(tof_tests PRIVATE gtest gtest_main)
