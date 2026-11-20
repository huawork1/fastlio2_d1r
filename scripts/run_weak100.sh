#!/usr/bin/env bash
set -euo pipefail

: "${ROS_DISTRO:?Source ROS2 and this workspace install/setup.bash first.}"
command -v ros2 >/dev/null || { echo "Missing ros2 command." >&2; exit 2; }

output_root=${OUTPUT_ROOT:-"$PWD/weak100_runs"}
stamp=$(date +%Y%m%d_%H%M%S)
output_dir="$output_root/$stamp"
mkdir -p "$output_dir/ros_logs"

# weak100.launch.py supplies these when unset. Export them here so this
# run keeps normal1500, candidate retry, and weak100, and so the audit
# file stays inside this run directory.
export LIO_SAMPLE_MODE=normal
export LIO_SAMPLE_BUDGET=1500
export PGO_RETRY_CANDIDATES=1
export PGO_DIRECTION_MODE=weak100
export PGO_DIRECTION_AUDIT_PATH="$output_dir/direction_audit.jsonl"
export PGO_VIEW_AUDIT_PATH="$output_dir/view_gate_audit.jsonl"
export ROS_LOG_DIR="$output_dir/ros_logs"
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1

printf 'weak100 output: %s\n' "$output_dir"
exec ros2 launch pgo weak100.launch.py "rviz:=${RVIZ:-false}"
