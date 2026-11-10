# K-City 60/30/25 profile and bounded experiment preparation

The recommendation profile is CRUISE 60, CURVE 30 and SHARP_CURVE 25 km/h.
These are upper bounds, never minimum speeds. This configuration does not
establish that 60 km/h driving is safe on the stored K-City route.

## Configuration layers

| Layer | Independent ceiling | Drive enabled | Global speed planner |
| --- | --- | --- | --- |
| `config/planning.yaml` | 20 km/h low-speed default | false | false default |
| `config/kcity_speed_profile_60.yaml` startup overlay | 60 km/h | false | true |
| Historical `local_data/kcity_live_recovery_20261110/recovery_parameters.yaml` | 20 km/h historical reproduction | false | true |
| `live_test.py --prepare --max-speed-kmh N` | explicit N, >0 and <=60 | false | true |

Apply the startup overlay **after** the full `planning.yaml`; it is not a
replacement for the geometry, topics and map settings in that file. The
normal Bringup reads `planning.yaml`. Runtime experiment preparation applies
the profile and ceiling atomically without restarting it. Do not use separate
parameter writes to change a running vehicle's profile or ceiling.

`SpeedPlanner` and `LongitudinalSpeedPlanner` contain no fixed 20 km/h ceiling.
Their 20m geometry contexts and the controller's 20m Local Path are distances.
The historical recovery YAML remains an unchanged low-speed snapshot, not an
override that the new tool loads. Omitting the new CLI ceiling still selects 20.

## Speed composition and assumptions

Local classification selects a policy limit of 60, 30 or 25 km/h. The global
planner only lowers that recommendation. For each located curve span:

```text
curve_cap = min(cruise_policy, span_policy, 3.6 * sqrt(a_lateral / curvature))
D(v,u) = v*tau + (v*v-u*u)/(2*a_deceleration) + margin  # v,u in m/s, v>u
requested_preview = max(base_preview, D(max(actual_speed, cruise_policy), 0) + 12m)
target = min(Local recommendation, all relevant global envelopes, independent ceiling)
```

Motion additionally requires the existing Drive Enable, FSM and finite lease.
Missing/old speed, invalid geometry and an incomplete global cache still gate
motion. The existing target-rise limiter, urgent curve caps, terminal stop and
multiple-curve constraints are unchanged. A lower upstream Local recommendation
is preserved. Obstacle scenarios remain disconnected in the current branch;
this work adds no obstacle avoidance and does not validate obstacle braking.

Initial planning assumptions remain 1.5m/s² lateral acceleration, 1.5m/s²
deceleration, 0.5s reaction, 5m margin, 80m base global preview, 0.2s speed
freshness, 1m/s² base target rise and up to 2m/s² qualified recovery. The tool
preserves committed safety settings. It accepts more conservative settings
within its preparation checks and rejects less conservative ones; it does not
silently raise a stricter lateral limit. These are planning assumptions, not
measured vehicle acceleration/deceleration guarantees.

For radius 5.04m, the lateral formula gives **9.90km/h**, despite the SHARP
policy being 25. With a_lateral=1.5, curvature 0.05m⁻¹ gives 19.72km/h and
0.2m⁻¹ gives 9.86km/h. At 60km/h the nominal lateral curvature boundary is
0.0054m⁻¹. A locally straight window may still require braking for a future curve.

| Change | Required distance with the assumptions above |
| --- | ---: |
| 60 → 30km/h | 82.78m |
| 60 → 25km/h | 89.85m |
| 60 → 9.90km/h | 103.41m |

The 60km/h requested global horizon is about 117.93m, clipped to actual route
coverage. A 10m Speed Preview or 20m control path is insufficient for these
budgets. The original short control path, curvature algorithm, cache invalidation
and controller geometry remain unchanged.

## Startup: Dev Container and host

Stop old launch instances before restarting. A separately started Planning
process can survive stopping Bringup; inspect `ps` and the ROS graph first.
Require one Planning and one Control publisher, not two nodes with equal names.

In every Dev Container terminal:

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Build if needed, then source the updated install:

```bash
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select edgenode_planning edgenode_control edgenode_bringup
source /workspace/ros2_ws/install/setup.bash
```

Dev Container terminal A:

```bash
ros2 launch edgenode_bringup autonomy.launch.py
```

Dev Container terminal B, without container RViz:

```bash
ros2 launch edgenode_planning route_editor.launch.py use_rviz:=false
```

Dev Container terminal C, with MORAI stationary and external control available:

```bash
ros2 param set /planning_node enable_drive false
ros2 service call /route_editor/load std_srvs/srv/Trigger "{}"
ros2 service call /route_editor/activate std_srvs/srv/Trigger "{}"
ros2 param get /planning_node route_node_ids
ros2 topic echo /planning/map_match_status --once
ros2 topic echo /planning/drive_status --once
ros2 topic echo /planning/global_path --once --field header
```

`load` reads `kcity_demo_route` from the saved route directory. `activate`
applies it with drive disabled; it never starts driving. If the simulator was
teleported/reset, restart the old Planning session while drive is disabled so
monotonic progress does not retain the previous vehicle position.

Run RViz **on the Ubuntu host**, sourcing `/opt/ros/humble/setup.bash`, using
the same ROS environment above and the supplied
`ros2_ws/src/edgenode_planning/rviz/kcity_route_editor.rviz`. Copy that file from
the Dev Container if the host does not have the source checkout; no host
workspace overlay is needed for the standard Path/Marker messages.

## Preparation and explicit runs

These commands configure and verify while stationary. Neither enables driving:

```bash
python3 local_data/kcity_live_recovery_20261110/live_test.py --prepare --max-speed-kmh 20
python3 local_data/kcity_live_recovery_20261110/live_test.py --prepare --max-speed-kmh 60
```

`--prepare` first disables and confirms stopped speed, zero target and fresh
brake commands. It activates the editor draft, commits the profile/ceiling in
one transaction, checks the resulting safety settings, captures one valid
global route, waits for sustained READY, and leaves drive disabled. An empty
editor draft requires `load` first. A failed preparation exits nonzero and
still attempts to disable driving. Logs use date/time/microseconds in
Asia/Seoul and do not overwrite earlier experiment logs.

Only a separately invoked `--run` enables driving. Its ceiling must exactly
match the prepared ceiling; omitting it selects 20 and cannot accidentally use
a prepared 60. A low-speed bounded test, after the matching preparation:

```bash
python3 local_data/kcity_live_recovery_20261110/live_test.py --run --seconds 120 --max-speed-kmh 20
```

The same interface supports an explicit higher ceiling after separate review;
availability of the option is not approval or evidence for a high-speed run.
Duration is restricted to 1..120s and the independent lease remains bounded.
The monitor rejects stale streams/nonfinite data/target ceiling violations,
stops for sustained actual speed > selected ceiling +1.5km/h, sustained path
error >1m, immediate error >1.5m, lost map match or unavailable long planning.
These monitor thresholds are not guarantees of lane containment or speed.
Every completion/error attempts drive=false and confirms fresh zero target,
stopped speed and brake. `--run` never directly publishes `/ctrl_cmd`.

```bash
# Manual stop request; false disables motion, true enables it.
ros2 param set /planning_node enable_drive false
```

## Limits before increasing speed

- Start with repeated 20km/h full-route observations; the previous partial
  trial had 20.52km/h actual overshoot and approximately 0.8m curve error.
- Evaluate 30, then 40/50 only on reviewed straight segments, keeping all curve
  caps and monitoring actual braking distance and response delay. Higher-speed
  stage approval is separate from preparing a profile.
- Current Localization has `gps_velocity_max_mps=15` (54km/h) as its GPS-derived
  velocity measurement gate. Planning uses EgoStatus speed, so this is not a
  54km/h target ceiling; rejected GPS velocity updates above it still require
  separate localization validation before considering 60. This work does not
  modify Localization.
- Under a linear normalized-wheel-angle assumption, steering limit 0.7 and
  wheelbase 2.84m imply a radius near 6.23m, larger than the fitted 5.04m curve.
  MORAI steering calibration and observed tracking must be reviewed. Speed
  reduction alone is not proof of geometric tracking capability.
- The target-rise limit applies to recommendation history. Drive enable or a
  hard-cap change can expose an already higher recommendation; a smooth
  standstill-to-60 command and physical acceleration/jerk are not established.
  Make profile/cap changes only while disabled.
- A paused simulator can repeat a nonzero velocity with fresh ROS timestamps.
  Do not equate those timestamps with advancing simulation physics. Preparation
  refuses a nonzero reported speed even if the position appears frozen.

These limits are grounds to keep the source default disabled and to avoid
claiming high-speed driving readiness from configuration, tests or replay alone.
