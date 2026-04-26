# iOS CMake Toolchain
# Usage:
#   cmake -B build-ios -S . \
#     -DCMAKE_TOOLCHAIN_FILE=cmake/ios.toolchain.cmake \
#     -DPLATFORM=OS64
#
# PLATFORM options:
#   OS64          - iOS device (arm64)
#   SIMULATOR64   - iOS Simulator on x86_64
#   SIMULATORARM64 - iOS Simulator on Apple Silicon

cmake_minimum_required(VERSION 3.22)

set(CMAKE_SYSTEM_NAME iOS)

# ── Platform selection ────────────────────────────────────────────────────────
if(NOT DEFINED PLATFORM)
    set(PLATFORM "OS64")
endif()

if(PLATFORM STREQUAL "OS64")
    set(CMAKE_SYSTEM_PROCESSOR arm64)
    set(CMAKE_OSX_ARCHITECTURES arm64)
    set(CMAKE_OSX_SYSROOT iphoneos)
elseif(PLATFORM STREQUAL "SIMULATOR64")
    set(CMAKE_SYSTEM_PROCESSOR x86_64)
    set(CMAKE_OSX_ARCHITECTURES x86_64)
    set(CMAKE_OSX_SYSROOT iphonesimulator)
elseif(PLATFORM STREQUAL "SIMULATORARM64")
    set(CMAKE_SYSTEM_PROCESSOR arm64)
    set(CMAKE_OSX_ARCHITECTURES arm64)
    set(CMAKE_OSX_SYSROOT iphonesimulator)
else()
    message(FATAL_ERROR "Unknown PLATFORM: ${PLATFORM}. Use OS64, SIMULATOR64, or SIMULATORARM64.")
endif()

# ── Compiler ──────────────────────────────────────────────────────────────────
set(CMAKE_C_COMPILER   /usr/bin/clang)
set(CMAKE_CXX_COMPILER /usr/bin/clang++)

# ── Deployment target ─────────────────────────────────────────────────────────
if(NOT DEFINED CMAKE_OSX_DEPLOYMENT_TARGET)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "15.0" CACHE STRING "Minimum iOS version")
endif()

# ── Search paths ──────────────────────────────────────────────────────────────
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ── Skip executable run tests (cross-compiling) ───────────────────────────────
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

message(STATUS "iOS toolchain: PLATFORM=${PLATFORM}, SYSROOT=${CMAKE_OSX_SYSROOT}, ARCH=${CMAKE_OSX_ARCHITECTURES}")
