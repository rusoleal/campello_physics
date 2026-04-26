cmake_minimum_required(VERSION 3.22)
project(campello_physics VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Options ──────────────────────────────────────────────────────────────────
option(CAMPELLO_PHYSICS_BUILD_TESTS      "Build unit tests"           OFF)
option(CAMPELLO_PHYSICS_BUILD_EXAMPLES   "Build examples"             OFF)
option(CAMPELLO_PHYSICS_BUILD_BENCHMARKS "Build performance benchmarks" OFF)
option(CAMPELLO_PHYSICS_GPU              "Enable GPU compute via campello_gpu" OFF)

# ── Dependencies ─────────────────────────────────────────────────────────────
include(FetchContent)

FetchContent_Declare(
    vector_math
    GIT_REPOSITORY https://github.com/rusoleal/vector_math.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(vector_math)

if(CAMPELLO_PHYSICS_BUILD_TESTS OR CAMPELLO_PHYSICS_BUILD_BENCHMARKS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.14.0
    )
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(BUILD_GMOCK   OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

if(CAMPELLO_PHYSICS_GPU)
    FetchContent_Declare(
        campello_gpu
        GIT_REPOSITORY https://github.com/rusoleal/campello_gpu.git
        GIT_TAG        v0.9.0
    )
    FetchContent_MakeAvailable(campello_gpu)
endif()

if(CAMPELLO_PHYSICS_BUILD_BENCHMARKS)
    FetchContent_Declare(
        googlebenchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG        v1.8.3
    )
    set(BENCHMARK_ENABLE_TESTING      OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_ENABLE_INSTALL      OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_DOWNLOAD_DEPENDENCIES ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googlebenchmark)
endif()

# ── Library target ────────────────────────────────────────────────────────────
add_library(campello_physics STATIC)

target_include_directories(campello_physics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src   # internal headers (e.g. broadphase/dynamic_bvh.h)
)

target_link_libraries(campello_physics PUBLIC vector_math)

# Suppress warnings from vector_math headers (external dependency) when compiling
# campello_physics sources — SYSTEM makes the compiler treat them as system headers.
target_include_directories(campello_physics SYSTEM PUBLIC
    $<TARGET_PROPERTY:vector_math,INTERFACE_INCLUDE_DIRECTORIES>
)

if(CAMPELLO_PHYSICS_GPU)
    target_link_libraries(campello_physics PRIVATE campello_gpu::campello_gpu)
    target_compile_definitions(campello_physics PRIVATE CAMPELLO_PHYSICS_GPU_ENABLED)
endif()

target_compile_features(campello_physics PUBLIC cxx_std_20)

# Platform detection definitions
if(WIN32)
    target_compile_definitions(campello_physics PUBLIC CAMPELLO_PLATFORM_WINDOWS)
elseif(APPLE)
    if(IOS)
        target_compile_definitions(campello_physics PUBLIC CAMPELLO_PLATFORM_IOS)
    else()
        target_compile_definitions(campello_physics PUBLIC CAMPELLO_PLATFORM_MACOS)
    endif()
elseif(ANDROID)
    target_compile_definitions(campello_physics PUBLIC CAMPELLO_PLATFORM_ANDROID)
elseif(UNIX)
    target_compile_definitions(campello_physics PUBLIC CAMPELLO_PLATFORM_LINUX)
endif()

# Compiler warnings
if(MSVC)
    target_compile_options(campello_physics PRIVATE /W4 /WX)
else()
    target_compile_options(campello_physics PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

# ── Subdirectories ────────────────────────────────────────────────────────────
add_subdirectory(src)

if(CAMPELLO_PHYSICS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(CAMPELLO_PHYSICS_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
