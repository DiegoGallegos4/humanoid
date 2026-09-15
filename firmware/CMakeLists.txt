cmake_minimum_required(VERSION 3.22)
project(nori_firmware CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Two build targets share this tree:
#   * native (default)  -> hal/sim, unit tests run on the host/CI
#   * stm32g4 (cross)    -> hal/stm32, via a toolchain file (added later)
# NORI_TARGET selects the HAL implementation.
set(NORI_TARGET "native" CACHE STRING "Build target: native | stm32g4")
message(STATUS "Nori firmware target: ${NORI_TARGET}")

add_compile_options(-Wall -Wextra -Wpedantic)

# --- firmware core -----------------------------------------------------------
add_library(nori_core INTERFACE)
target_include_directories(nori_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/core/include)

# --- HAL interfaces (header-only contract) -----------------------------------
add_library(nori_hal INTERFACE)
target_include_directories(nori_hal INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/hal/include)

# --- control (header-only: PID + position/velocity controllers) --------------
add_library(nori_control INTERFACE)
target_include_directories(nori_control INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/control/include)

# --- protocol (header-only: CAN message set + codec) -------------------------
add_library(nori_protocol INTERFACE)
target_include_directories(nori_protocol INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/protocol/include)
target_link_libraries(nori_protocol INTERFACE nori_hal)

# --- HAL implementation, selected by target ----------------------------------
if(NORI_TARGET STREQUAL "native")
  # Sim HAL is header-only; expose it as an interface library.
  add_library(nori_hal_sim INTERFACE)
  target_include_directories(nori_hal_sim INTERFACE
      ${CMAKE_CURRENT_SOURCE_DIR}/hal/sim/include)
  target_link_libraries(nori_hal_sim INTERFACE nori_hal)
  add_library(nori_hal_impl ALIAS nori_hal_sim)
elseif(NORI_TARGET STREQUAL "stm32g4")
  add_library(nori_hal_stm32 STATIC hal/stm32/stm32_hal.cpp)
  target_include_directories(nori_hal_stm32 PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/hal/stm32/include)
  target_link_libraries(nori_hal_stm32 PUBLIC nori_hal)
  add_library(nori_hal_impl ALIAS nori_hal_stm32)
else()
  message(FATAL_ERROR "Unknown NORI_TARGET: ${NORI_TARGET}")
endif()

# --- tests (native only) -----------------------------------------------------
if(NORI_TARGET STREQUAL "native")
  enable_testing()
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()
  add_subdirectory(tests)
endif()
