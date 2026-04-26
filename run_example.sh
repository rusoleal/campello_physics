#!/usr/bin/env bash
# Launch a campello_physics example by name.
# Usage: ./run_example.sh <name>
#   name: falling_sphere | character_controller | vehicle | ragdoll

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build_examples"

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <name>"
    echo "  Available: falling_sphere  character_controller  vehicle  ragdoll"
    exit 1
fi

NAME="$1"

case "$NAME" in
    falling_sphere)       PRODUCT="example_falling_sphere" ;;
    character_controller) PRODUCT="example_character_controller" ;;
    vehicle)              PRODUCT="example_vehicle" ;;
    ragdoll)              PRODUCT="example_ragdoll" ;;
    *)
        echo "Unknown example: '$NAME'"
        echo "  Available: falling_sphere  character_controller  vehicle  ragdoll"
        exit 1
        ;;
esac

APP=$(find "$BUILD_DIR" -name "${PRODUCT}.app" -maxdepth 5 | head -n 1)

if [[ -z "$APP" ]]; then
    echo "App not found: ${PRODUCT}.app"
    echo "Run ./build_examples.sh first."
    exit 1
fi

echo "==> Launching $APP"
open "$APP"
