#!/usr/bin/env bash
# Builds the Google Benchmark harness in Release mode and asserts the
# combined PID+Mixer / TFLM-Inference median time is under the 500 us
# real-time deadline for a 250 Hz attitude-control loop. Exits 1 (and
# prints the offending totals) if the deadline is exceeded.
set -euo pipefail
cd "$(dirname "$0")/.."

BUILD_DIR="build-benchmark"
NPROC=$(command -v nproc >/dev/null 2>&1 && nproc || sysctl -n hw.ncpu)

cmake -S . -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_BENCHMARK=ON
cmake --build "${BUILD_DIR}" --target eaq_benchmark -j"${NPROC}"

RESULTS_JSON=$(mktemp)
trap 'rm -f "${RESULTS_JSON}"' EXIT

"${BUILD_DIR}/eaq_benchmark" \
  --benchmark_format=json \
  --benchmark_min_time=10000x \
  --benchmark_repetitions=11 \
  --benchmark_report_aggregates_only=true \
  > "${RESULTS_JSON}"

python3 - "${RESULTS_JSON}" <<'PYEOF'
import json
import sys

with open(sys.argv[1]) as f:
    data = json.load(f)

TIME_UNIT_TO_US = {"ns": 1e-3, "us": 1.0, "ms": 1e3, "s": 1e6}
DEADLINE_US = 500.0

medians = {}
for b in data["benchmarks"]:
    if b.get("aggregate_name") != "median":
        continue
    scale = TIME_UNIT_TO_US[b["time_unit"]]
    medians[b["run_name"]] = b["real_time"] * scale

pid_mixer_us = medians["PID+Mixer"]
inference_us = medians["TFLM Inference"]
total_us = pid_mixer_us + inference_us

print(f"PID+Mixer:      {pid_mixer_us:.2f} us (median)")
print(f"TFLM Inference: {inference_us:.2f} us (median)")
print(f"Total:          {total_us:.2f} us  (deadline: {DEADLINE_US:.0f} us)")

if total_us > DEADLINE_US:
    print(f"FAIL: combined PID+Inference loop ({total_us:.2f} us) "
          f"exceeds the {DEADLINE_US:.0f} us real-time deadline")
    sys.exit(1)

print("OK: combined loop within the real-time deadline")
PYEOF
