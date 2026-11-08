# S4 Stage 4: distance admission and separate previews

This stage does not authorize real avoidance. `enable_drive=false` remains in
both normal and S4 configurations. No real DrivableArea, crossing authorization
or sensor range evidence is generated. Normal mode's Dijkstra, original 20 m
local extraction, Pure Pursuit, PID, and speed settings remain unchanged.

## Three different paths

1. NORMAL Control retains `local_path_length_m=20` and the original reference
   object/output. Global progress is updated only from that extraction.
2. S4 hazard preview uses a second bounded extraction, default 80 m, on the
   **same original global route**. It cannot extend past the real route. It is
   passed separately as `hazard_reference`, never published as the local path.
3. Lattice candidates still use the actual lane pair, capped by its support
   minus front-body/end margin and the existing 80 m prediction limit. Full
   entry/hold/return geometry and the real road envelope must pass. The S4
   avoidance Control window is independently `s4_distance_control_horizon_m`,
   default 20 m. Both production-PP rollout and local output use that value.

Increasing preview or prediction does not change LiDAR detection or authorize
unseen road. S4 rejects insufficient actual route stopping support. Candidate
entry plus delay reserve must fit verified visibility, the obstacle deadline,
real adjacent-lane support, and an explicitly approved observation curvature
scope. Unknown boundaries/permissions still stop avoidance. The existing
full-body clearance, return reservation, acquisition confirmations, steering
clamp/rate, lateral acceleration and tracking checks are unchanged.

## Stopping model

All distances use the rear axle reference. Front body reach is 3.790 m. Let
`v=max(measured_speed, S4_speed_cap)/3.6`. The S4 cap defaults to 3 km/h, even
before any obstacle. A lower target never substitutes for the actual speed.

```
T = source_age + perception_budget + planning_budget + Control_watchdog
    + Control_period + command_delay + reaction_time
D = front_reach + obstacle_margin + uncertainty_margin + v*T + v*v/(2*a)
```

Defaults: perception .2 s, whole Planning 1.5 s, watchdog .6 s, period 1/30 s,
command .1 s, reaction .3 s, `a=2 m/s²`, obstacle margin .25 m, uncertainty 1 m.
Age is the larger source-cloud/Planning-pose age, not a heartbeat receipt age.
Source age plus perception budget intentionally over-reserves elapsed latency.
These are **modelling assumptions requiring external measurements**, not proof
of MORAI braking/transport or a measured worst-case execution time. The 1.5 s
budget covers the previously observed .52–1.34 s cold samples but is not WCET.

|km/h|Stop, age 0 m|Stop, age .6 s m|20 m window rollout requirement m|
|---:|---:|---:|---:|
|3|7.49|7.99|6.43|
|10|14.56|16.23|14.21|
|20|27.94|31.27|25.32|
|50|91.23|99.56|62.41|

Numbers must be reproduced with `tools/s4_distance_report.py`; rounding in this
table does not set a threshold. The ROI x=20 plus mount=1.19 yields an optimistic
21.19 m **ceiling**. Clustering, masks, partial footprints, scan sparsity,
occlusion and road curvature can reduce effective range. READY_EMPTY means
valid processing, not that the stopping corridor is proven free.

Bare geometric entry needs `lookahead + v*command_delay + transition +
front_reach + obstacle_margin`. At 3 km/h, 12/16/20 m transitions need
18.623/22.623/26.623 m. The 16 m candidate already exceeds the ROI ceiling by
1.433 m. Stage 4 additionally reserves `1 m + v*(T-command_delay)` without
counting preparation's command delay twice. Even the 12 m candidate then needs
21.818 m at age zero (16 m: 25.818; 20 m: 29.818). No increased sensor range is
assumed. For the explicitly synthetic obstacle at 35–39 m, the existing
16 m return ends at 59.109 m from the initial rear axle (obstacle rear +
1.109 m rear overhang + .5 m return margin + 2.5 m lookahead + 16 m transition).
Supporting its front body and the existing 1 m map-end margin needs 63.899 m.
This is a full-maneuver map requirement, not a claim of observing that distance.
Short adjacent support is rejected; endpoints are never interpolated forward.
Braking after late detection cannot restore missing distance:
`S4_OBSTACLE_STOPPING_DISTANCE` latches before the DECELERATING branch.

## Externally reviewed observation evidence

The startup-only parameters `s4_observation_evidence_file` and
`s4_observation_evidence_approved_sha256` default empty. With no pinned report,
S4 rejects admission **even with fresh READY_EMPTY**, while the existing sensor
status still separately describes successful parsing. No production report is
shipped. The SHA pin records an operator's explicit review; it does not itself
prove the report's physical claims.

The JSON contract is `schema=edgenode.s4.observation.v1`,
`data_kind=measured_sensor_validation`, and nonempty `source`. Required fields:

- `forward_range_m`, `max_speed_kmh`, `max_curvature`, `route_link_ids` scope.
  All active route IDs must be within the reviewed scope. Range cannot exceed
  the actual launch YAML ROI+mount ceiling; exceeding that ceiling is rejected.
- Exact `lidar_config_sha256`, `control_config_sha256`, `mgeo_links_sha256`.
  Dedicated launch supplies the actual installed YAML file paths via read-only
  `s4_lidar_config_file` and `s4_control_config_file`. Direct node runs must
  supply those paths to validate a report. PP model/watchdogs must match.
- Positive `perception_latency_upper_s`, `planning_latency_upper_s`,
  `command_latency_upper_s`, `minimum_deceleration_mps2` within model bounds.
- Literal true `stopping_corridor_verified`, `occlusion_policy_verified`,
  `full_obstacle_extent_verified`, `braking_verified`, `timing_verified`.

Measurements must cover the full footprint and stopping corridor, side lane,
road curvature and operating scope; forward distance alone is insufficient.
Unknown/occluded parts cannot be called free. Verify the effective running
producer/Control parameters against these files before approving evidence;
this offline file binding does not attest another process's runtime overrides.
A sensor report never substitutes for separate real-map DrivableArea approval
or explicit lane crossing approval. Test reports are synthetic temporary
fixtures, not deployable map/sensor approval artifacts.

## Failure behavior and timing

S4 admission runs before route work, after preview/candidate work, and again
before the final speed command. The entire Planning timer uses steady elapsed
time, including map matching and route/path publication. An elapsed time above
1.5 s latches `S4_DISTANCE_PLANNING_BUDGET_EXCEEDED`. Sensor source-age checks
remain stricter (.6 s); a .52–1.34 s cold search can therefore fail freshness
before the compute budget. New searches still publish STOP before expensive
validation and require three fresh pose stamps before release. No stale cold
result authorizes motion. A blocked thread cannot run its watchdog; the stop
model reserves the Control watchdog interval and a Control tick as well.

Additional reasons include `OBSERVATION_EVIDENCE_*`, `OBSERVATION_SCOPE`,
`OBSERVATION_ROUTE_SCOPE`, `OBSERVATION_STOPPING_DISTANCE`,
`INSUFFICIENT_HAZARD_PREVIEW`, `MAP_STOPPING_SUPPORT`, `ENTRY_OBSERVATION_DISTANCE`,
`OBSERVATION_CURVATURE_SCOPE`, and `S4_OBSTACLE_STOPPING_DISTANCE` (distance-gate
integration reasons have the `S4_DISTANCE_` prefix). Failure preserves the first
BLOCKED cause, publishes an empty local path and speed 0. Healthy data, larger
preview or lower measured speed never automatically clears BLOCKED. Reset
requires drive disabled, healthy paired data/pose, and valid distance admission.

`/planning/s4_distance_state` is a separate `std_msgs/String` JSON diagnostic:
readiness/reason, source age and each delay, measured/checked speeds, reaction
and braking distances, front reach, stopping/entry/rollout requirements,
verified observation range, preview and actual control-window parameters,
and whole Planning elapsed time. It is emitted once fresh paired cloud/pose
data are available; before that, `/planning/s4_sensor_state` explains the sensor
failure. It is not a Control path or an actuator topic.

The existing PP rollout simulates a no-slip bicycle for 2 s, holding measured
curvature for the initial .1 s command delay and checking the unchanged .6 rad/s
wheel rate. It cannot reconstruct all pending real steering commands. The old
closed loop ends at BLOCKED and does not prove physical post-stop behavior.
New delayed-stop integration tests are analytic synthetic checks only; real
PID brake response, steering actuator delay, slip, clock/transport outages,
worst-case CPU and lateral visibility remain unvalidated. The unchanged lateral
ROI is ±4 m: even a synthetic 3.5 m side-lane offset plus .93 m body half-width
and .25 m obstacle margin exceeds that lateral coverage. This example is not
a measured K-City lane width and cannot authorize an ROI change.

## Reproduce without moving a vehicle

Use a separate ROS domain and localhost. `s4_validation.launch.py` starts only
Perception and Planning, not Control/localization/actuator nodes. Do not set
`enable_drive=true`. The no-sensor state should remain BLOCKED/empty/zero.

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select edgenode_planning edgenode_bringup --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1
ros2 launch edgenode_bringup s4_validation.launch.py use_sim_time:=false
```

Stop an older instance with Ctrl+C before restarting this launch (which
restarts both Planning and LiDAR). Choose `use_sim_time:=true` only with an
isolated recorded `/clock` and odometry/cloud stamps from the same time base.
The combined launch already starts LiDAR; do not also start the LiDAR launch.
For individual restarts in the same isolated domain:

```bash
ros2 launch edgenode_perception lidar_obstacle.launch.py
ros2 run edgenode_planning planning_node --ros-args \
  --params-file /workspace/ros2_ws/install/edgenode_planning/share/edgenode_planning/config/planning.yaml \
  --params-file /workspace/ros2_ws/install/edgenode_planning/share/edgenode_planning/config/s4_sensor_validation.yaml \
  -p enable_drive:=false
```

The individual Planning example intentionally has no evidence/config approval;
it stays fail-closed. Check installed outputs without any vehicle commands:

```bash
ros2 param get /planning_node enable_drive
ros2 param get /planning_node s4_distance_preview_m
ros2 topic echo --once /planning/s4_sensor_state
timeout 5s ros2 topic echo --once /planning/s4_distance_state
ros2 topic echo --once /planning/avoidance_debug
ros2 topic echo --once /planning/local_path
ros2 topic echo --once /planning/target_speed
```

From `/workspace`, with the sourced workspace, the offline reproduction and all
package tests start no vehicle/Control processes:

```bash
python3 tools/s4_distance_report.py --reproduce --out /tmp/s4_distances.json
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 -m pytest -q -p no:cacheprovider \
  ros2_ws/src/edgenode_perception/test ros2_ws/src/edgenode_planning/test \
  ros2_ws/src/edgenode_control/test
```
