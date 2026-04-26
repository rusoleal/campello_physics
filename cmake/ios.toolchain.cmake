# iOS / Apple Platform CMake Toolchain
#
# Usage:
#   cmake -B build-ios -S . \
#     -DCMAKE_TOOLCHAIN_FILE=cmake/ios.toolchain.cmake \
#     -DPLATFORM=OS64
#
# PLATFORM options:
#   OS64             - iOS device (arm64)
#   SIMULATOR64      - iOS Simulator (x86_64)
#   SIMULATORARM64   - iOS Simulator (arm64, Apple Silicon host)
#   TVOS             - tvOS device (arm64)
#   TVOSSIMULATOR    - tvOS Simulator (arm64)
#   WATCHOS          - watchOS device (arm64_32 + arm64)
#   WATCHOSSIMULATOR - watchOS Simulator (arm64)
#   XROS             - visionOS device (arm64)
#   XROSSIMULATOR    - visionOS Simulator (arm64)

cmake_minimum_required(VERSION 3.22)

if(NOT DEFINED PLATFORM)
    set(PLATFORM "OS64")
endif()

# ── Per-platform sysroot, arch, and deployment target ────────────────────────

if(PLATFORM STREQUAL "OS64")
    set(CMAKE_SYSTEM_NAME          iOS)
    set(CMAKE_SYSTEM_PROCESSOR     arm64)
    set(CMAKE_OSX_ARCHITECTURES    arm64)
    set(CMAKE_OSX_SYSROOT          iphoneos)
    set(_DEFAULT_DEPLOY_TARGET     "15.0")

elseif(PLATFORM STREQUAL "SIMULATOR64")
    set(CMAKE_SYSTEM_NAME          iOS)
    set(CMAKE_SYSTEM_PROCESSOR     x86_64)
    set(CMAKE_OSX_ARCHITECTURES    x86_64)
    set(CMAKE_OSX_SYSROOT          iphonesimulator)
    set(_DEFAULT_DEPLOY_TARGET     "15.0")

elseif(PLATFORM STREQUAL "SIMULATORARM64")
    set(CMAKE_SYSTEM_NAME          iOS)
    set(CMAKE_SYSTEM_PROCESSOR     arm64)
    set(CMAKE_OSX_ARCHITECTURES    arm64)
    set(CMAKE_OSX_SYSROOT          iphonesimulator)
    set(_DEFAULT_DEPLOY_TARGET     "15.0")

elseif(PLATFORM STREQUAL "TVOS")
    set(CMAKE_SYSTEM_NAME          tvOS)
    set(CMAKE_SYSTEM_PROCESSOR     arm64)
    set(CMAKE_OSX_ARCHITECTURES    arm64)
    set(CMAKE_OSX_SYSROOT          appletvos)
    set(_DEFAULT_DEPLOY_TARGET     "16.0")

elseif(PLATFORM STREQUAL "TVOSSIMULATOR")
    set(CMAKE_SYSTEM_NAME          tvOS)
    set(CMAKE_SYSTEM_PROCESSOR     arm64)
    set(CMAKE_OSX_ARCHITECTURES    arm64)
    set(CMAKE_OSX_SYSROOT          appletvsimulator)
    set(_DEFAULT_DEPLOY_TARGET     "16.0")

elseif(PLATFORM STREQUAL "WATCHOS")
    set(CMAKE_SYSTEM_NAME          watchOS)
    set(CMAKE_SYSTEM_PROCESSOR     arm64)
    set(CMAKE_OSX_ARCHITECTURES    "arm64_32;arm64")
    set(CMAKE_OSX_SYSROOT          watchos)
    set(_DEFAULT_DEPLOY_TARGET     "8.0")

elseif(PLATFORM STREQUAL "WATCHOSSIMULATOR")
    set(CMAKE_SYSTEM_NAME          watchOS)
    set(CMAKE_SYSTEM_PROCESSOR     arm64)
    set(CMAKE_OSX_ARCHITECTURES    arm64)
    set(CMAKE_OSX_SYSROOT          watchsimulator)
    set(_DEFAULT_DEPLOY_TARGET     "8.0")

elseif(PLATFORM STREQUAL "XROS")
    set(CMAKE_SYSTEM_NAME          visionOS)
    set(CMAKE_SYSTEM_PROCESSOR     arm64)
    set(CMAKE_OSX_ARCHITECTURES    arm64)
    set(CMAKE_OSX_SYSROOT          xros)
    set(_DEFAULT_DEPLOY_TARGET     "1.0")

elseif(PLATFORM STREQUAL "XROSSIMULATOR")
    set(CMAKE_SYSTEM_NAME          visionOS)
    set(CMAKE_SYSTEM_PROCESSOR     arm64)
    set(CMAKE_OSX_ARCHITECTURES    arm64)
    set(CMAKE_OSX_SYSROOT          xrsimulator)
    set(_DEFAULT_DEPLOY_TARGET     "1.0")

else()
    message(FATAL_ERROR
        "Unknown PLATFORM '${PLATFORM}'. Valid values: "
        "OS64 SIMULATOR64 SIMULATORARM64 "
        "TVOS TVOSSIMULATOR "
        "WATCHOS WATCHOSSIMULATOR "
        "XROS XROSSIMULATOR")
endif()

# ── Deployment target ─────────────────────────────────────────────────────────

if(NOT DEFINED CMAKE_OSX_DEPLOYMENT_TARGET)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "${_DEFAULT_DEPLOY_TARGET}"
        CACHE STRING "Minimum OS deployment target")
endif()

# ── Compiler ──────────────────────────────────────────────────────────────────

set(CMAKE_C_COMPILER   /usr/bin/clang)
set(CMAKE_CXX_COMPILER /usr/bin/clang++)

# ── Search paths (cross-compiling) ────────────────────────────────────────────

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

message(STATUS
    "Apple toolchain: PLATFORM=${PLATFORM}  "
    "SYSROOT=${CMAKE_OSX_SYSROOT}  "
    "ARCH=${CMAKE_OSX_ARCHITECTURES}  "
    "DEPLOY=${CMAKE_OSX_DEPLOYMENT_TARGET}")
