#pragma once

// ── Platform ──────────────────────────────────────────────────────────────────
// CAMPELLO_PLATFORM_* is set by CMake. These guards provide a fallback for
// toolchains that define it via predefined macros instead.
#if !defined(CAMPELLO_PLATFORM_WINDOWS) && !defined(CAMPELLO_PLATFORM_MACOS) && \
    !defined(CAMPELLO_PLATFORM_LINUX)   && !defined(CAMPELLO_PLATFORM_IOS)   && \
    !defined(CAMPELLO_PLATFORM_ANDROID) && !defined(CAMPELLO_PLATFORM_WASM)
#  if defined(_WIN32) || defined(_WIN64)
#    define CAMPELLO_PLATFORM_WINDOWS
#  elif defined(__APPLE__)
#    include <TargetConditionals.h>
#    if TARGET_OS_IOS
#      define CAMPELLO_PLATFORM_IOS
#    else
#      define CAMPELLO_PLATFORM_MACOS
#    endif
#  elif defined(__ANDROID__)
#    define CAMPELLO_PLATFORM_ANDROID
#  elif defined(__EMSCRIPTEN__)
#    define CAMPELLO_PLATFORM_WASM
#  elif defined(__linux__)
#    define CAMPELLO_PLATFORM_LINUX
#  endif
#endif

// ── Compiler hints ────────────────────────────────────────────────────────────
#if defined(_MSC_VER)
#  define CAMPELLO_FORCE_INLINE __forceinline
#  define CAMPELLO_LIKELY(x)    (x)
#  define CAMPELLO_UNLIKELY(x)  (x)
#elif defined(__GNUC__) || defined(__clang__)
#  define CAMPELLO_FORCE_INLINE __attribute__((always_inline)) inline
#  define CAMPELLO_LIKELY(x)    __builtin_expect(!!(x), 1)
#  define CAMPELLO_UNLIKELY(x)  __builtin_expect(!!(x), 0)
#else
#  define CAMPELLO_FORCE_INLINE inline
#  define CAMPELLO_LIKELY(x)    (x)
#  define CAMPELLO_UNLIKELY(x)  (x)
#endif

#define CAMPELLO_NODISCARD [[nodiscard]]

// ── Math namespace alias ──────────────────────────────────────────────────────
#include <vector_math/vector_math.hpp>

namespace campello::physics {
    namespace vm = systems::leal::vector_math;
}
