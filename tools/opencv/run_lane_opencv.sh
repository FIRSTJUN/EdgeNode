#!/usr/bin/env bash
# Run the perception branch's camera detector in the existing ROS container.
set -eo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
show_preview=true
if [[ "${1:-}" == "--no-view" ]]; then
    show_preview=false
    shift
fi
if [[ "${1:-}" == "--help" ]]; then
    echo "Usage: bash tools/opencv/run_lane_opencv.sh [--no-view] [ROS arguments]"
    echo "Example: bash tools/opencv/run_lane_opencv.sh --ros-args -p camera_topic:=/image_jpeg/compressed"
    exit 0
fi
if "$show_preview" && [[ -z "${DISPLAY:-}" ]]; then
    echo "DISPLAY is unset. Use --no-view to run without the preview window." >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
cd "$project_dir/ros2_ws"
colcon build --symlink-install --packages-select lane_opencv_pkg
source install/local_setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-$project_dir/logs/lane_opencv}"
mkdir -p "$ROS_LOG_DIR"

if ! "$show_preview"; then
    exec python3 -m lane_opencv_pkg.lane_detector --ros-args -r __node:=lane_opencv_test "$@"
fi

process_ids=()
cleanup() {
    trap - EXIT INT TERM
    kill -INT "${process_ids[@]}" 2>/dev/null || true
    wait "${process_ids[@]}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

python3 -m lane_opencv_pkg.lane_detector --ros-args -r __node:=lane_opencv_test "$@" &
process_ids+=("$!")
python3 "$project_dir/tools/opencv/preview_lane.py" &
process_ids+=("$!")
wait -n "${process_ids[@]}"
