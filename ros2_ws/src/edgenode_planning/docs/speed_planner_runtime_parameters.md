# SpeedPlanner runtime parameter synchronization

`planning_node` now applies committed SpeedPlanner parameters at the start of
the next planning cycle. Previously these values were copied only during node
construction, so `ros2 param get` could report 30/15/8 km/h while the active
planner still recommended its startup values of 50/25/3.5 km/h.

## Configuration and transaction behavior

The following nine values form one configuration snapshot:

| ROS parameter | Valid values |
| --- | --- |
| `cruise_speed_kmh` | finite, >= 0 km/h |
| `curve_speed_kmh` | finite, >= 0 km/h |
| `sharp_curve_speed_kmh` | finite, >= 0 km/h |
| `speed_preview_distance_m` | finite, > 0 m |
| `curve_curvature_threshold` | finite, >= 0 and below sharp threshold |
| `sharp_curvature_threshold` | finite, above curve threshold |
| `speed_curvature_smoothing_window_m` | finite, > 0 m |
| `speed_curvature_persistence_m` | finite, > 0 m |
| `speed_curvature_hysteresis` | finite, >= 0 |

The on-set callback merges the proposed values with the current parameter
store and validates the complete combination. It does not change the active
planner: Humble calls this callback before committing parameters, and another
callback may still reject the request. An invalid atomic request changes no
parameters, including unrelated parameters bundled in that request.

The timer and parameter services use the node's default mutually exclusive
callback group. At the start of each cycle the node reads all nine committed
values together. If they changed, it constructs a new SpeedPlanner and clears
the previous Speed Mode. The next path calculation uses the new configuration.
If values did not change, it keeps the same object and hysteresis state; only
the nine-value comparison is added to the cycle. Route progress/cache and the
drive session are not reset by a speed configuration change.

The validation limits match the existing SpeedPlanner contract. Zero speed
and zero hysteresis remain valid; no new ordering constraint is imposed on
the three speed values. Invalid startup overrides retain the existing behavior:
speed planning returns INVALID and target speed remains zero until corrected.

Independent `ros2 param set` calls are independent transactions. A valid
intermediate configuration can be used between calls. Use
`set_parameters_atomically` when multiple values must change together, especially
when a threshold pair would otherwise pass through an invalid combination.

## Build and restart

Stop the existing launch with Ctrl+C before replacing its node. This change
does not update a Python process that is already running.

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
colcon build --symlink-install --packages-select edgenode_planning
source /workspace/ros2_ws/install/setup.bash
ros2 launch edgenode_bringup autonomy.launch.py
```

The launch above restarts localization/planning/control using their existing
configuration files. To restart only the planning node while keeping the
other existing nodes, stop the old planning process and instead run:

```bash
ros2 run edgenode_planning planning_node --ros-args \
  --params-file /workspace/ros2_ws/install/edgenode_planning/share/edgenode_planning/config/planning.yaml
```

Startup defaults/YAML files are unchanged by this fix. Runtime values do not
survive a restart; restore the route and drive configuration through the
existing procedure, and explicitly apply the desired speed profile again.

## MORAI verification without starting a drive

The commands below are a manual verification procedure, not commands executed
as part of the automated tests. Keep driving disabled while checking parameters
and recommendation topics.

```bash
ros2 param set /planning_node enable_drive false
ros2 topic info /planning/recommended_speed --verbose
ros2 param dump /planning_node
```

Check that the recommendation has one intended publisher. Apply all three
speeds and the independent 30 km/h drive cap as one transaction:

```bash
ros2 service call /planning_node/set_parameters_atomically \
  rcl_interfaces/srv/SetParametersAtomically \
  "{parameters: [
    {name: cruise_speed_kmh, value: {type: 3, double_value: 30.0}},
    {name: curve_speed_kmh, value: {type: 3, double_value: 15.0}},
    {name: sharp_curve_speed_kmh, value: {type: 3, double_value: 8.0}},
    {name: max_drive_speed_kmh, value: {type: 3, double_value: 30.0}}
  ]}"
ros2 param get /planning_node cruise_speed_kmh
ros2 param get /planning_node curve_speed_kmh
ros2 param get /planning_node sharp_curve_speed_kmh
```

Expect `successful: true` and 30.0/15.0/8.0. A subsequent single-value update
also works, for example `ros2 param set /planning_node curve_speed_kmh 15.0`.

Observe these topics in separate terminals, comparing timestamp-aligned or
stable-Mode samples rather than unrelated individual transition messages:

```bash
ros2 topic echo /planning/speed_mode
ros2 topic echo /planning/recommended_speed
ros2 topic echo /planning/target_speed
```

With a valid path, CRUISE/CURVE/SHARP_CURVE recommendations should use
30/15/8 km/h from the next cycle. This fix does not change when curvature is
classified as a curve. While `enable_drive=false`, target speed must remain 0
even if the recommendation is positive. With driving enabled in a separately
authorized test and the existing guards satisfied, target remains
`min(recommended_speed, max_drive_speed_kmh)`; no test here starts that drive.

To confirm rejection without changing the current valid settings:

```bash
ros2 param set /planning_node speed_preview_distance_m 0.0
ros2 param get /planning_node speed_preview_distance_m
```

Expect failure with a reason and the previous positive Preview value to remain.

## Regression coverage

`test_speed_parameters_ros.py` uses real Humble parameter APIs and an atomic
parameter service in a private Context/domain with unique topics. Planning
outputs are captured by mocks, so they cannot command MORAI. It covers:

- Three speed modes before/after a runtime profile change.
- All nine settings applied through one actual ROS atomic service request.
- Invalid bounds, nonfinite values, inconsistent thresholds, and whole-request rejection.
- Rejection by a later callback without premature planner/hysteresis mutation.
- Object reuse and hysteresis retention for no-op/unrelated changes.
- Hysteresis reset for a changed configuration, without resetting route/drive lease.
- Missing localization, invalid startup recovery, zero speed, and the drive gate.

The curvature algorithm, Pure Pursuit, PID, Steering Limit, global route files
and original rosbag are unchanged. Automated tests and package builds do not
constitute MORAI driving validation.
