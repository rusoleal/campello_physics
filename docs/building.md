# Building campello_physics

## Prerequisites

| Tool | Minimum version | Notes |
|---|---|---|
| CMake | 3.22 | Tested with 3.25+ |
| C++ compiler | C++20 | Clang 14+, GCC 12+, MSVC 2022 |
| Git | 2.x | For FetchContent dependency fetching |

All other dependencies (vector_math, GoogleTest, Google Benchmark, campello_gpu) are fetched automatically via CMake FetchContent and require an internet connection on the first configure.

---

## macOS

**Requirements:** Xcode 14+ (includes Clang, Metal, MetalKit)

```bash
# Configure
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release

# Build library only
cmake --build build

# Build + run tests
cmake -B build -S . -DCAMPELLO_PHYSICS_BUILD_TESTS=ON
cmake --build build
cd build && ctest --output-on-failure

# Build examples (requires campello_renderer — fetched automatically)
cmake -B build -S . -DCAMPELLO_PHYSICS_BUILD_EXAMPLES=ON
cmake --build build

# Debug build
cmake -B build -S . -DCMAKE_BUILD_TYPE=Debug -DCAMPELLO_PHYSICS_BUILD_TESTS=ON
```

**Apple Silicon (arm64):** default when building on M-series Macs.

**Universal binary (arm64 + x86_64):**
```bash
cmake -B build -S . -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64"
cmake --build build
```

---

## iOS

Cross-compile from a macOS host. The `cmake/ios.toolchain.cmake` toolchain is included in the repository.

```bash
cmake -B build-ios -S . \
  -DCMAKE_TOOLCHAIN_FILE=cmake/ios.toolchain.cmake \
  -DPLATFORM=OS64 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-ios
```

`PLATFORM` options:
- `OS64` — physical device (arm64)
- `SIMULATOR64` — x86_64 simulator
- `SIMULATORARM64` — arm64 simulator (Apple Silicon host)

Tests cannot run on the simulator from the command line; use Xcode for on-device testing.

---

## Windows

**Requirements:** Visual Studio 2022 (MSVC v143 or later, C++20 enabled)

```bat
cmake -B build -S . -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release

REM With tests
cmake -B build -S . -G "Visual Studio 17 2022" -A x64 -DCAMPELLO_PHYSICS_BUILD_TESTS=ON
cmake --build build --config Release
cd build && ctest -C Release --output-on-failure
```

**Clang on Windows (via LLVM):**
```bat
cmake -B build -S . -T ClangCL
cmake --build build
```

---

## Linux

**Requirements:** GCC 12+ or Clang 14+, cmake, build-essential

```bash
sudo apt install build-essential cmake git   # Ubuntu/Debian

cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# Tests
cmake -B build -S . -DCAMPELLO_PHYSICS_BUILD_TESTS=ON
cmake --build build -j$(nproc)
cd build && ctest --output-on-failure
```

---

## Android

Cross-compile from a macOS or Linux host using the Android NDK.

```bash
# NDK must be installed — set ANDROID_NDK to its path
cmake -B build-android -S . \
  -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake \
  -DANDROID_ABI=arm64-v8a \
  -DANDROID_PLATFORM=android-29 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-android
```

`ANDROID_ABI` options: `arm64-v8a` (recommended), `x86_64` (emulator), `armeabi-v7a`.

---

## CMake options reference

| Option | Default | Description |
|---|---|---|
| `CAMPELLO_PHYSICS_BUILD_TESTS` | `OFF` | Build GoogleTest unit tests |
| `CAMPELLO_PHYSICS_BUILD_EXAMPLES` | `OFF` | Build example apps (requires campello_renderer) |
| `CAMPELLO_PHYSICS_BUILD_BENCHMARKS` | `OFF` | Build Google Benchmark microbenchmarks |
| `CAMPELLO_PHYSICS_GPU` | `OFF` | Enable GPU compute backend via campello_gpu |
| `CMAKE_BUILD_TYPE` | — | `Debug`, `Release`, `RelWithDebInfo`, `MinSizeRel` |

---

## Installing the library

```bash
cmake -B build -S . -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build
cmake --install build
```

This installs:
- Headers to `<prefix>/include/campello_physics/`
- Static library to `<prefix>/lib/`
- CMake package config to `<prefix>/lib/cmake/campello_physics/`

Consumers can then use:
```cmake
find_package(campello_physics REQUIRED)
target_link_libraries(my_app PRIVATE campello_physics)
```

---

## Generating API documentation

Requires [Doxygen](https://www.doxygen.nl) and [Graphviz](https://graphviz.org) (for diagrams).

```bash
doxygen docs/Doxyfile
open docs/generated/html/index.html   # macOS
```
