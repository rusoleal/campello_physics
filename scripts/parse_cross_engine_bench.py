#!/usr/bin/env python3
"""
Parse cross-engine benchmark output and print a clean comparison table.
Groups results by benchmark scenario (e.g. ConstraintChains/100) and
shows Campello vs Bullet vs Jolt side by side.
"""

import sys
import re
from collections import defaultdict


def parse_benchmark_output(text):
    """Parse Google Benchmark output lines into structured data."""
    # Pattern: BM_Name/Engine_Nt/N/min_time:X.Y  [time]  [cpu]  [iters]  [counters...]
    pattern = re.compile(
        r'^(BM_[^/]+)/([^/]+)/(\d+)(?:/min_time:[^\s]+)?\s+'
        r'([\d.]+)\s+(\w+)\s+'      # Time value + unit
        r'([\d.]+)\s+(\w+)\s+'      # CPU value + unit
        r'(\d+)'                     # Iterations
    )

    results = defaultdict(lambda: defaultdict(dict))

    for line in text.splitlines():
        line = line.strip()
        # Skip Google Benchmark summary rows (mean, median, stddev, cv)
        if any(suffix in line for suffix in ['_mean', '_median', '_stddev', '_cv']):
            continue
        m = pattern.match(line)
        if not m:
            continue

        benchmark = m.group(1)
        engine_config = m.group(2)    # e.g. "Campello_1t"
        size = m.group(3)             # e.g. "100"
        time_val = float(m.group(4))
        time_unit = m.group(5)
        cpu_val = float(m.group(6))
        cpu_unit = m.group(7)
        iters = int(m.group(8))

        # Split engine and thread config
        parts = engine_config.rsplit('_', 1)
        if len(parts) == 2 and parts[1].endswith('t'):
            engine = parts[0]
            threads = parts[1]
        else:
            engine = engine_config
            threads = "1t"

        scenario = f"{benchmark}/{size}"
        key = (scenario, threads)

        results[key][engine] = {
            'time_val': time_val,
            'time_unit': time_unit,
            'cpu_val': cpu_val,
            'cpu_unit': cpu_unit,
            'iters': iters,
        }

    return results


def format_value(val, unit):
    """Format a time value with appropriate precision."""
    if unit == 'ms':
        return f"{val:7.3f} ms"
    elif unit == 'us':
        return f"{val:7.3f} us"
    elif unit == 'ns':
        return f"{val:7.3f} ns"
    else:
        return f"{val:7.3f} {unit}"


def compute_speedup(base_val, cmp_val):
    """Compute speedup factor."""
    if base_val == 0 or cmp_val == 0:
        return ""
    ratio = base_val / cmp_val
    if ratio >= 100:
        return f"{ratio:.0f}x"
    elif ratio >= 10:
        return f"{ratio:.1f}x"
    else:
        return f"{ratio:.2f}x"


def print_table(results):
    """Print a formatted comparison table."""
    if not results:
        print("No benchmark results found.", file=sys.stderr)
        return

    # Sort keys: by scenario name, then by thread count
    sorted_keys = sorted(results.keys(), key=lambda k: (k[0], k[1]))

    # Engines we expect to see
    engines = ['Campello', 'Bullet', 'Jolt']

    # Print header
    print()
    print("=" * 110)
    print(f"{'Scenario':<32} {'Threads':<8} {'Campello (Time)':<18} {'Bullet (Time)':<18} {'Jolt (Time)':<18} {'Campello vs Jolt':<12}")
    print("-" * 110)

    for key in sorted_keys:
        scenario, threads = key
        engines_data = results[key]

        campello = engines_data.get('Campello')
        bullet = engines_data.get('Bullet')
        jolt = engines_data.get('Jolt')

        c_str = format_value(campello['time_val'], campello['time_unit']) if campello else "N/A"
        b_str = format_value(bullet['time_val'], bullet['time_unit']) if bullet else "N/A"
        j_str = format_value(jolt['time_val'], jolt['time_unit']) if jolt else "N/A"

        speedup = ""
        if campello and jolt:
            speedup = compute_speedup(campello['time_val'], jolt['time_val'])

        print(f"{scenario:<32} {threads:<8} {c_str:<18} {b_str:<18} {j_str:<18} {speedup:<12}")

    print("=" * 110)

    # Second table: CPU times
    print()
    print("=" * 110)
    print(f"{'Scenario':<32} {'Threads':<8} {'Campello (CPU)':<18} {'Bullet (CPU)':<18} {'Jolt (CPU)':<18} {'Campello vs Bullet':<12}")
    print("-" * 110)

    for key in sorted_keys:
        scenario, threads = key
        engines_data = results[key]

        campello = engines_data.get('Campello')
        bullet = engines_data.get('Bullet')
        jolt = engines_data.get('Jolt')

        c_str = format_value(campello['cpu_val'], campello['cpu_unit']) if campello else "N/A"
        b_str = format_value(bullet['cpu_val'], bullet['cpu_unit']) if bullet else "N/A"
        j_str = format_value(jolt['cpu_val'], jolt['cpu_unit']) if jolt else "N/A"

        speedup = ""
        if campello and bullet:
            speedup = compute_speedup(campello['cpu_val'], bullet['cpu_val'])

        print(f"{scenario:<32} {threads:<8} {c_str:<18} {b_str:<18} {j_str:<18} {speedup:<12}")

    print("=" * 110)
    print()


def main():
    text = sys.stdin.read()
    results = parse_benchmark_output(text)
    print_table(results)


if __name__ == '__main__':
    main()
