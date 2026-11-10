# Global-route anticipatory speed planning

**Experimental; disabled by default.** Offline replay does not establish50km/h
MORAI safety. Deceleration, latency and lateral acceleration are assumptions.
Conservative extra slowing, low-speed release time and geometry quality still
need review before driving. Pure Pursuit/PID/Steering Limit are unchanged.

## Algorithm and precedence

The selected Global Path is copied/validated once and indexed by cumulative XY
arc. `RouteProfileBuilder` processes eight20m contexts per cycle, reusing Phase3
median/persistence, distributed heading and artifact checks. Original corners
remain; only long segments are subdivided to<=2m. Each context owns its central
1m on a0.25m measurement grid. The20m steering Local Path is unchanged.

Adjacent support is merged only while direction/mode agree and curvature varies
by at most10%. A distant sharper section cannot impose its maximum on an entire
gentle approach. Immutable spans record entry/exit, direction, curvature, heading
change and evidence flags (1 median/persistence,2 heading,3 both). Flags record
detector provenance, not calibrated confidence probabilities. Sustained gentle
curvature below the mode threshold also supplies lateral limits.

```text
curve_cap = min(cruise_speed, configured speed for the span's mode,
                3.6*sqrt(lateral_accel_limit/supported_curvature))
D_required = v*tau + (v*v-u*u)/(2*a) + margin  [v>u; SI units]
v_allow = max(u, -a*tau+sqrt((a*tau)^2+u*u+2*a*max(distance-margin,0)))
uncapped_recommendation = min(local recommendation, every relevant approach cap, cruise)
recommendation[n] = min(uncapped_recommendation,
                        recommendation[n-1] + 3.6*effective_release_rate*dt)
target_speed = existing enable/FSM/lease gate ? min(recommendation, drive cap) : 0
```

The preview is at least80m by default and expands to the braking distance from
`max(actual_speed,cruise)` to zero plus12m of geometry context. It is clipped to
available route length. All future constraints in that range are considered,
including a sharper second curve and the next curve after a short straight.
Below-threshold support uses cruise as its configured mode cap. Raw Max spikes
never directly set the lateral cap.

If measured speed already exceeds the **margin-free** braking budget, an urgent
cap latches until that curve exits. Lower measured speed cannot cancel it and
cause reacceleration. This cannot make inadequate brakes feasible. The known
route endpoint is a zero-speed boundary with zero advance margin, so the curve
margin does not move the mission endpoint5m back. Existing short-path/FSM gates
still apply; goal-reached FSM logic is not added.

The upward target limiter addresses release jumps and one-cycle local CURVE
dips without deleting their safety reductions. **All reductions apply immediately.**
The first timed query has no previous command; subsequent valid timed queries
are rise-limited. Invalid/backwards time, input loss or route replacement clears
history. Non-geometric runtime updates retain only release history, not urgent
state. A slower recovery is its explicit cost. It bounds recommendation changes,
not actual vehicle acceleration or jerk, and does not constrain the first command
after a reset. Long scheduler/service gaps cap the release allowance at the
configured speed timeout; they do not introduce a new stop if speed data is fresh.
Standalone untimed physics queries retain their previous API;
the ROS integration and vehicle replay always supply ROS time.

Existing `/planning/speed_mode`, curvature and hysteresis remain **local**. Global
modes never feed local previous-mode history. A local CRUISE mode with a lower
recommendation can reflect a distant curve or limited release; consult diagnostics.

## Phase 5: confirmed recovery without relaxing curve caps

The baseline target rise rate remains **1.0 m/s²**. A faster recovery rate is
eligible only when Local Mode is explicitly CRUISE, its recommendation is at
least cruise, and no forward geometric/terminal constraint is binding. These
conditions must be observed continuously for `speed_plan_recovery_clear_sec`
(default 1 s). A missing Mode, scheduler gap, route/input reset or binding curve
cannot establish that confidence. Non-geometric parameter changes keep target
history but require new clear-road observations.

After confirmation, the target rise rate can approach the recovery setting
(default 2 m/s²). Only the extra rate tapers near cruise:

```text
fast = max(baseline_rate, recovery_rate)
gap = max(0, uncapped_recommendation - previous_recommendation)
effective_release_rate = baseline_rate
  + (fast-baseline_rate) * min(1, gap / (3.6*fast*recovery_clear_sec))
baseline_rise = 3.6*baseline_rate*dt
allowed_rise = min(3.6*effective_release_rate*dt, max(baseline_rise, 0.5 km/h))
```

The extra recovery has a 0.5 km/h per-cycle step ceiling; it never reduces the
existing baseline allowance. At the nominal 20 Hz, a 2 m/s² target ramp adds
only 0.36 km/h per tick, so the extra ceiling does not bind. It prevents slower
cycles from turning faster recovery into larger command jumps. Diagnostics
report the allowed rate after this ceiling, not a measured acceleration.

The exact cruise cap remains reachable. Any new Local/forward safety reduction
still applies immediately. This changes target-rate planning, not actual vehicle
acceleration. It leaves curve detection, lateral caps, multi-curve minima, entry
braking and urgent exit tests unchanged. The new target never exceeds the same
underlying geometric/Local cap. Fixed-input steering equality does not imply
identical steering in a new closed-loop drive with different vehicle speeds.

A weak-signal deletion rule was not adopted: a short gentle S-curve can lack
discrete CURVE/Heading evidence while its sustained curvature still requires a
lateral cap. Preserve that constraint pending independent map/road evidence.

## Parameters and atomic updates

| Parameter | Default | Accepted range |
| --- | --- | --- |
| `speed_long_preview_enabled` | false | boolean |
| `speed_long_preview_distance_m` | 80.0 | .01..100000m |
| `speed_plan_deceleration_mps2` | 1.5 | .01..20m/s² |
| `speed_plan_reaction_time_sec` | .5 | 0..10s |
| `speed_plan_distance_margin_m` | 5.0 | 0..1000m |
| `speed_plan_lateral_accel_mps2` | 1.5 | .01..20m/s² |
| `speed_plan_speed_timeout_sec` | .2 | .01..5s |
| `speed_plan_acceleration_mps2` | 1.0 | .01..20m/s² target rise rate |
| `speed_plan_recovery_acceleration_mps2` | 2.0 | .01..20m/s² confirmed recovery target rate |
| `speed_plan_recovery_clear_sec` | 1.0 | .1..10s continuous clear observation |
| `ego_status_topic` | `/ego_vehicle_status` | startup topic |
| `long_speed_plan_topic` | `/planning/long_speed_plan` | startup topic |

Numeric values must be finite and not booleans. Bounds prevent numerical
degeneracy, not endorsement of extreme driving settings. Existing nine settings
keep their contract, including zero speeds and no new speed-order requirement.
All nineteen are read as one committed snapshot. Rejected atomic requests and
later callback rejections cannot mutate active planners.

Speed/budget/lateral/release changes reuse finished geometry and apply next
cycle, subject to the same downward caps/upward release bound. Threshold,
smoothing, persistence or enablement changes invalidate the profile. Rebuilding
gates speed rather than mixing configurations. No-op values retain objects/state.
Parameter updates never extend the drive lease/reset route progress.

## Readiness, freshness and resets

- Cache BUILDING, invalid path/configuration or missing/stale/nonfinite speed:
  no valid global recommendation; existing speed-ready/FSM gate gives target0.
  A failed build is retained until route/configuration changes.
- Ego speed is `hypot(vx,vy,vz)*3.6`, matching Control. Header and receipt ages
  must both be nonnegative and within timeout at the end of planning work.
  Missing speed is not substituted with zero speed.
- Match loss, missing speed or invalid/backwards query time clears urgent
  and release state while keeping valid geometry. Route replacement discards
  both builder/profile. Runtime geometry changes do likewise.
- Route Editor's atomic `enable_drive=false` + `route_node_ids` transaction is
  unchanged. Repeated XY on loops is distinguished by monotonic route arc.
- Clock rewind clears speed freshness, local history and query state. Existing
  latched drive-lease expiry is preserved; no automatic rearming occurs.

Routes are bounded to100,000 finite XYZ vertices/100km. Wait for READY with
driving disabled before arming a finite drive lease. The K-City route takes304
batches; elapsed warmup depends on cycle rate. Queries do not rebuild the4,911
points. Existing global-path publication/local extraction costs are unchanged.

## Diagnostics

`/planning/long_speed_plan` is JSON in `std_msgs/String`: readiness, measured speed
and ages, requested/available preview, controlling span ID/mode/distance/cap,
curvature/direction/heading/evidence, required distance/margin/feasibility and
urgent flag. Reasons include APPROACH, CURVE, URGENT_DECEL, TERMINAL_STOP and
LOCAL_OR_CRUISE. BUILDING reports completed/total windows. Missing values are null.
RELEASE_LIMIT reports an upward-limited target. `uncapped_target_speed_kmh`,
`release_limited`, `release_rise_limit_kmh` and `geometric_reason` distinguish
that release history from the current geometric constraint.
`recovery_active`, `recovery_clear_sec` (observed duration) and
`recovery_acceleration_mps2` (effective target rate) expose confirmed recovery.

## Build and manual MORAI review without starting a drive

Stop the old Planning process before replacing it. Do not run two motion-target
publishers. These commands were not executed against MORAI during development.

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
colcon build --packages-select edgenode_control edgenode_planning edgenode_bringup
source /workspace/ros2_ws/install/setup.bash
ros2 run edgenode_planning planning_node --ros-args \
  --params-file /workspace/ros2_ws/install/edgenode_planning/share/edgenode_planning/config/planning.yaml \
  --params-file /workspace/local_data/kcity_routes/planning_node.yaml \
  -p enable_drive:=false -p speed_long_preview_enabled:=true \
  -p cruise_speed_kmh:=30.0 -p curve_speed_kmh:=15.0 -p sharp_curve_speed_kmh:=8.0 \
  -p max_drive_speed_kmh:=30.0
```

Verify the saved route is the intended one; original route files are unchanged.
To inspect50/25/20 recommendations, keep driving disabled and apply one transaction:

```bash
ros2 service call /planning_node/set_parameters_atomically \
  rcl_interfaces/srv/SetParametersAtomically \
  "{parameters: [
    {name: enable_drive, value: {type: 1, bool_value: false}},
    {name: speed_long_preview_enabled, value: {type: 1, bool_value: true}},
    {name: cruise_speed_kmh, value: {type: 3, double_value: 50.0}},
    {name: curve_speed_kmh, value: {type: 3, double_value: 25.0}},
    {name: sharp_curve_speed_kmh, value: {type: 3, double_value: 20.0}},
    {name: speed_plan_deceleration_mps2, value: {type: 3, double_value: 1.5}},
    {name: speed_plan_reaction_time_sec, value: {type: 3, double_value: 0.5}},
    {name: speed_plan_distance_margin_m, value: {type: 3, double_value: 5.0}},
    {name: speed_plan_lateral_accel_mps2, value: {type: 3, double_value: 1.5}},
    {name: speed_plan_acceleration_mps2, value: {type: 3, double_value: 1.0}}
  ]}"
ros2 topic echo /planning/long_speed_plan
ros2 topic echo /planning/recommended_speed
ros2 topic echo /planning/target_speed
ros2 topic echo /planning/speed_mode
```

Target remains0. Confirm BUILDING→READY with fresh MORAI speed. At supported
curvature1/5.04m the1.5m/s² lateral cap is9.90km/h; actual MGeo638 supported mean
curvature gives about10.49km/h. These estimators differ; neither certifies grip.
Recorded30/15/8 limits, including sharp8, cannot be raised by the new planner.

## Current limits and driving verification

The535 original additional slow classifications include distance envelopes,
urgent braking budgets, conservative holds and a terminal stop. They are not an
independent road-ground-truth false-positive count. Release limiting removes
large immediate rises but prolongs low targets; keep the feature disabled by
default until that cost is reviewed. A lower target alone is not proof of safety.
Raw map quality, steering feasibility and actual braking remain uncertain.

Phase 5 retained every geometric constraint and reduced additional low-target
classifications from 910 to 862 in the same 30 km/h bag. In the conservative
Local-straight screen they fell from 241 to 207. The large adjacent reversal
metric stayed zero and the observed maximum one-cycle rise stayed 0.512 km/h.
An unbounded extra-recovery candidate reached 833/183 but increased the maximum
rise to 0.959 km/h; that trade-off was not selected. Actual acceleration is not
inferred from target rate. Low-target time, weak geometry and generalization
remain unresolved; the feature stays disabled by default.

Separate reviewed low-speed driving must measure entry speed, actual braking,
path error, steering saturation, release timing, unwanted slowing, CPU timing and
lease duration. These examples do not arm driving or establish high-speed safety.
