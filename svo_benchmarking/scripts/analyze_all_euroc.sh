#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 2 ]]; then
  echo "Usage: $0 RESULT_ROOT REPORT_OUTPUT_DIRECTORY" >&2
  exit 2
fi
# Analyze all available estimate variants/trials, without an external evaluator.
ros2 run svo_benchmarking reference_report.py "$1" "$2" --batch
