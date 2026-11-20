#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: bash examples/run_example.sh /path/to/d1r_weak100_indoor_65s" >&2
  exit 2
fi
: "${ROS_DISTRO:?Source ROS2 and this workspace install/setup.bash first.}"
command -v ros2 >/dev/null || { echo "Missing ros2 command." >&2; exit 2; }

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
bag_path=$1
[[ -f "$bag_path/metadata.yaml" ]] || { echo "Missing $bag_path/metadata.yaml" >&2; exit 2; }

output_root=${OUTPUT_ROOT:-"$PWD/example_output"}
output_dir="$output_root/$(date +%Y%m%d_%H%M%S)"
[[ ! -e "$output_dir" ]] || { echo "Output already exists: $output_dir" >&2; exit 2; }
map_dir="$output_dir/map"
mkdir -p "$map_dir"

OUTPUT_ROOT="$output_dir/slam" RVIZ="${RVIZ:-false}" bash "$repo_dir/scripts/run_mapping.sh" \
  > "$output_dir/launch_console.log" 2>&1 &
launch_pid=$!
cleanup() {
  if kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

sleep "${STARTUP_DELAY:-5}"
# Delay first publication until discovery settles; reliable LiDAR plus final ACKs
# prevent silent frame loss at startup, during playback, and at end-of-bag.
ros2 bag play "$bag_path" --rate "${RATE:-1}" \
  --delay "${PLAY_DELAY:-2}" --wait-for-all-acked "${WAIT_FOR_ALL_ACKED_MS:-10000}" \
  --disable-keyboard-controls
sleep "${DRAIN_SECONDS:-8}"

ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: \"$map_dir\", save_patches: true}" | tee "$output_dir/save_service.log"

cleanup
for _ in $(seq 1 20); do
  if ! kill -0 "$launch_pid" 2>/dev/null; then break; fi
  sleep 0.25
done
if kill -0 "$launch_pid" 2>/dev/null; then kill -TERM "$launch_pid"; fi
wait "$launch_pid" || true
trap - EXIT

printf 'Example map: %s\n' "$map_dir/map.pcd"
printf 'Launch log: %s\n' "$output_dir/launch_console.log"
