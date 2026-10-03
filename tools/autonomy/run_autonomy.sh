#!/usr/bin/env bash
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$project_root"
source /opt/ros/humble/setup.bash
source "$project_root/ros2_ws/install/setup.bash"
if [[ "${1:-}" == "--restart" ]]; then
    shift
    python3 "$project_root/tools/autonomy/stop_existing.py"
fi
# Refuse a second controller or perception publisher on the same MORAI topics.
python3 - <<'PY'
import time
import rclpy
rclpy.init()
node = rclpy.create_node('edgenode_launch_check')
end = time.monotonic()+2.0
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.2)
conflicts = [topic for topic in ('/ctrl_cmd', '/perception/lane_error')
             if node.count_publishers(topic)]
node.destroy_node()
rclpy.shutdown()
if conflicts:
    raise SystemExit('Already running: '+', '.join(conflicts)+'\n다시 실행하려면: bash /workspace/tools/autonomy/run_autonomy.sh --restart enable_drive:=true drive_duration_sec:=60.0')
PY
mkdir -p "$project_root/logs/autonomy"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$project_root/logs/autonomy}"
exec ros2 launch edgenode_bringup autonomy.launch.py "$@"
