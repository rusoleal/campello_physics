#!/usr/bin/env bash
# Convenience script to build campello_physics for WebAssembly (Emscripten).
# Requires the Emscripten SDK to be activated (emsdk_env.sh).

set -euo pipefail

BUILD_DIR="build_wasm"
BUILD_TYPE="Release"
BUILD_TESTS="ON"

while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --no-tests)
            BUILD_TESTS="OFF"
            shift
            ;;
        --build-dir)
            BUILD_DIR="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --debug         Build in Debug mode (default: Release)"
            echo "  --no-tests      Skip building tests"
            echo "  --build-dir DIR Build directory (default: build_wasm)"
            echo "  --help, -h      Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Run '$0 --help' for usage."
            exit 1
            ;;
    esac
done

echo "=== Configuring campello_physics for WASM ==="
echo "  Build dir:  ${BUILD_DIR}"
echo "  Build type: ${BUILD_TYPE}"
echo "  Tests:      ${BUILD_TESTS}"
echo ""

# Verify emcmake is available
if ! command -v emcmake &> /dev/null; then
    echo "Error: emcmake not found. Please activate the Emscripten SDK first:"
    echo "  source /path/to/emsdk/emsdk_env.sh"
    exit 1
fi

emcmake cmake -B "${BUILD_DIR}" -S . \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCAMPELLO_PHYSICS_BUILD_TESTS="${BUILD_TESTS}"

echo ""
echo "=== Building ==="
cmake --build "${BUILD_DIR}" --parallel

echo ""
echo "=== Build complete ==="
if [[ "${BUILD_TESTS}" == "ON" ]]; then
    echo "Run tests with:"
    echo "  cd ${BUILD_DIR} && ctest --output-on-failure"
fi
