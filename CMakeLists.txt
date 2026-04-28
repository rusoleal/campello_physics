cmake_minimum_required(VERSION 3.22)
project(campello_physics VERSION 1.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Options ──────────────────────────────────────────────────────────────────
option(CAMPELLO_PHYSICS_BUILD_TESTS      "Build unit tests"           OFF)
option(CAMPELLO_PHYSICS_BUILD_EXAMPLES   "Build examples"             OFF)
option(CAMPELLO_PHYSICS_BUILD_BENCHMARKS "Build performance benchmarks" OFF)
option(CAMPELLO_PHYSICS_GPU              "Enable GPU compute via campello_gpu" OFF)

# Emscripten (WASM) toolchain setup — must be early so all targets inherit flags.
if(EMSCRIPTEN)
    add_compile_options(-msimd128)
    add_compile_options(-pthread)
    add_link_options(-pthread -sUSE_PTHREADS=1 -sPTHREAD_POOL_SIZE=4)
    add_link_options(-sALLOW_MEMORY_GROWTH=1)

    if(CAMPELLO_PHYSICS_BUILD_EXAMPLES)
        message(WARNING "CAMPELLO_PHYSICS_BUILD_EXAMPLES is not supported on WASM — forcing OFF")
        set(CAMPELLO_PHYSICS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    endif()
    if(CAMPELLO_PHYSICS_GPU)
        message(WARNING "CAMPELLO_PHYSICS_GPU is not supported on WASM — forcing OFF")
        set(CAMPELLO_PHYSICS_GPU OFF CACHE BOOL "" FORCE)
    endif()
endif()

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
        GIT_TAG        v0.13.0
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
if(EMSCRIPTEN)
    target_compile_definitions(campello_physics PUBLIC CAMPELLO_PLATFORM_WASM)
elseif(WIN32)
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

# Enable AVX2+FMA on x86_64 for SIMD performance
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64|AMD64|amd64)")
    if(MSVC)
        target_compile_options(campello_physics PRIVATE /arch:AVX2)
    else()
        target_compile_options(campello_physics PRIVATE -mavx2 -mfma)
    endif()
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

# ── Install / Packaging ───────────────────────────────────────────────────────
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS campello_physics
    EXPORT campello_physics-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(EXPORT campello_physics-targets
    FILE campello_physics-targets.cmake
    NAMESPACE campello_physics::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/campello_physics
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/campello_physics-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/campello_physics-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/campello_physics
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/campello_physics-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/campello_physics-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/campello_physics-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/campello_physics
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/campello_physics.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/campello_physics.pc
    @ONLY
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/campello_physics.pc
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)

if(CAMPELLO_PHYSICS_BUILD_BENCHMARKS)
    add_subdirectory(bench_cross_engine)
endif()
