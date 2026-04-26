#!/usr/bin/env bash
# Build all campello_physics examples (macOS only).
# Usage: ./build_examples.sh [--clean] [--release]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build_examples"
BUILD_TYPE="Debug"

for arg in "$@"; do
    case "$arg" in
        --clean)   rm -rf "$BUILD_DIR" ;;
        --release) BUILD_TYPE="Release" ;;
    esac
done

echo "==> Configuring ($BUILD_TYPE) …"
cmake -B "$BUILD_DIR" -S "$SCRIPT_DIR" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCAMPELLO_PHYSICS_BUILD_EXAMPLES=ON \
    -DCAMPELLO_PHYSICS_BUILD_TESTS=OFF

echo "==> Building …"
cmake --build "$BUILD_DIR" --parallel

echo ""
echo "Build complete. App bundles:"
find "$BUILD_DIR/examples" -name "*.app" -maxdepth 4 | sort | while read -r app; do
    echo "  $app"
done
echo ""
echo "Run an example with: ./run_example.sh <name>"
echo "  names: falling_sphere  character_controller  vehicle  ragdoll"
