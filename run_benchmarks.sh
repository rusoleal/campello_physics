#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
BENCH_DIR="${BUILD_DIR}/tests"
CROSS_BENCH_DIR="${BUILD_DIR}/bench_cross_engine"

# Colors
BOLD='\033[1m'
RESET='\033[0m'
BLUE='\033[34m'
GREEN='\033[32m'

print_header() {
    echo ""
    echo -e "${BOLD}${BLUE}==>${RESET} ${BOLD}$1${RESET}"
    echo ""
}

run_benchmark() {
    local name="$1"
    local binary="$2"
    shift 2
    local args=("$@")

    if [[ ! -f "${binary}" ]]; then
        echo "Warning: ${binary} not found. Skipping ${name}."
        return 1
    fi

    print_header "Running: ${name}"
    "${binary}" "${args[@]}"
}

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS] [BENCHMARK_FILTER]

Options:
  -i, --internal     Run internal benchmarks only (build/tests/campello_physics_bench)
  -c, --cross        Run cross-engine benchmarks only (build/bench_cross_engine/campello_physics_cross_engine_bench)
  -a, --all          Run both benchmark suites (default)
  -h, --help         Show this help message

BENCHMARK_FILTER:
  Optional Google Benchmark filter (e.g., "ConstraintChains" or "BM_Pile/100")

Examples:
  ./run_benchmarks.sh
  ./run_benchmarks.sh --cross ConstraintChains
  ./run_benchmarks.sh --internal BM_FreeFall
EOF
}

# Parse arguments
MODE="all"
FILTER=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        -i|--internal)
            MODE="internal"
            shift
            ;;
        -c|--cross)
            MODE="cross"
            shift
            ;;
        -a|--all)
            MODE="all"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
        *)
            FILTER="$1"
            shift
            ;;
    esac
done

# Build benchmark args
BENCH_ARGS=()
if [[ -n "${FILTER}" ]]; then
    BENCH_ARGS+=("--benchmark_filter=${FILTER}")
fi
BENCH_ARGS+=("--benchmark_repetitions=3")
BENCH_ARGS+=("--benchmark_min_time=0.5s")

cd "${SCRIPT_DIR}"

# Run requested benchmarks
if [[ "${MODE}" == "all" || "${MODE}" == "internal" ]]; then
    run_benchmark \
        "Internal Benchmarks (${BENCH_ARGS[*]})" \
        "${BENCH_DIR}/campello_physics_bench" \
        "${BENCH_ARGS[@]}"
fi

if [[ "${MODE}" == "all" || "${MODE}" == "cross" ]]; then
    print_header "Running: Cross-Engine Benchmarks (${BENCH_ARGS[*]})"
    if [[ ! -f "${CROSS_BENCH_DIR}/campello_physics_cross_engine_bench" ]]; then
        echo "Warning: ${CROSS_BENCH_DIR}/campello_physics_cross_engine_bench not found. Skipping."
    else
        "${CROSS_BENCH_DIR}/campello_physics_cross_engine_bench" "${BENCH_ARGS[@]}" 2>&1 | tee /tmp/cross_bench_raw.txt
        echo ""
        if [[ -f "${SCRIPT_DIR}/scripts/parse_cross_engine_bench.py" ]]; then
            python3 "${SCRIPT_DIR}/scripts/parse_cross_engine_bench.py" < /tmp/cross_bench_raw.txt
        fi
    fi
fi

echo ""
echo -e "${BOLD}${GREEN}==>${RESET} ${BOLD}All benchmarks complete.${RESET}"
