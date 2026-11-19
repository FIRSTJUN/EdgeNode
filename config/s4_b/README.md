# Phase B opt-in profiles

These profiles are separate from `config/s4_v61` and `config/s4_a`.
`planning_10.yaml` and `planning_15.yaml` preserve the corresponding v61
qualified Planning values, adding the explicit B-only
`phase_b_normal_tracking_clearance_enabled` startup choice. `planning_20.yaml` changes only the avoidance
ceiling from the 15 profile to 20 km/h. Cruise/sharp targets remain 60/30.
The dedicated B node, not the original v61 planner, owns B integration.

```bash
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 launch edgenode_bringup s4_b_static20.launch.py enable_drive:=false \
  source_time_localization:=true \
  localization_profile_file:=/workspace/config/s4_b/localization_source_time.yaml \
  planning_profile_file:=/workspace/config/s4_b/planning_20.yaml
```

Launch only after the previous stack and its search workers are stopped and
`/ctrl_cmd` has zero publishers. This command starts one control publisher
and leaves drive disabled. Inspect `/planning/phase_b_runtime` and
`/planning/avoidance_debug`, sensor READY/source age, loaded files and actual
parameters before authorizing motion. The validated route is the saved
18-waypoint K-City mission in the YAML. It is not an arbitrary-map guarantee.

The avoidance ceiling is a request. A road Offset may select a lower speed
only after a complete unchanged map/body/PP proof. Runtime debug reports both
requested and effective values; return to NORMAL restores the ceiling.
Lower-speed safe passage is reported separately from 20 km/h performance.
`use_v61_baseline:=true` is for reproducing the preserved implementation,
not for claiming B integration qualification.

Map/route/native/SDK dependencies and hashes are retained under
`local_data/s4_final_completion_20261118/final-v61-package/`.
Actual qualification, failures, recovery and limits: `docs/s4_b_static20.md`.
Do not edit symlinked executing sources. Stop, build, restart and verify first.

The explicit B launch selects the independent source-time localizer by default.
The original localization_node.yaml remains unchanged. Final B uses
localization_source_time.yaml (GPS std0.05m, ROS vehicle speed std0.1m/s),
0.8s replay history and the original0.6s source/0.05s future limits.
NORMAL map-only C2 centering uses the same pinned measured road and full body
plus0.15m reserve. Neither original v61/A launch nor profile is overwritten.
