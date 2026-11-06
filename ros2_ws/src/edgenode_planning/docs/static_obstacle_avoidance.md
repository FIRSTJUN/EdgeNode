# Generic static obstacle avoidance

The only production path is `planning_node → StaticObstaclePlanner`. There is
no legacy mode selector. Existing MapMatcher, Dijkstra, LocalPathPlanner,
SpeedPlanner, readiness/drive-lease PlanningFSM, Localization, LiDAR producer,
Pure Pursuit and PID algorithms remain unchanged.

**Current K-City map has no verified road/lane boundary data.**
`avoidance_boundary_file: ""` is intentional. The planner can generate and display
MGeo-based alternatives, but rejects avoidance with missing road evidence,
publishes an empty control path and requests zero target speed. Normal obstacle-
free following retains the original local path, speed policy and drive lease.
Normal baseline following is not a new claim of surveyed road containment.

## Data and geometry

MGeo `link_set.json` supplies real centerline XYZ, directed adjacent destination
links and endpoint connectivity. Ordinary valid links, reciprocal adjacency,
same forward direction and connected successor pairs are required. Explicit
`can_move_*_lane: false` rejects both entry permission and reserved return. A
topological neighbor declaration is not surveyed lane-marking/road geometry.
No fixed lateral offsets or centerline-derived road-width polygons are used.

`DrivableArea` is a separate future interface for externally verified convex
road cells in the MGeo `map` frame with mandatory provenance. Its cell union
may be concave or contain holes. Missing/invalid data never selects a fallback.
Future JSON must contain `frame_id: map`, a nonempty `source` and `cells`
(lists of convex XY polygons from verified data). No example fake road is
provided. Synthetic test ground truth is injected only by `test/avoidance_world.py`.

Vehicle geometry is the user-approved MORAI Sonata rear-wheel axle midpoint:
front +3.790 m, rear -1.109 m, lateral +/-0.930 m; wheelbase 2.840 m.
The approved corners sum to 4.899 m versus nominal 4.900 m. The code records
the difference without modifying overhangs or numerical consistency tolerance.
The supplied MORAI width 4.86 m is rejected as a documentation typo; the user
approved Hyundai's 1.860 m width cross-check. Sources:

- https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/vehicle-specifications
- https://www.hyundai.com/contents/repn-car/catalog_en/sonata-catalog.pdf (PDF page 17)

The authoritative Control YAML is reused at launch and in offline tests for
wheelbase, wheel-angle scale 0.610865 rad, normalized clamp 0.70, sign -1,
30 Hz control rate and dynamic lookahead.
The command-geometry curvature limit remains 0.1604669205 /m.

LiDAR production code is preserved: incoming MORAI XYZ is sensor-local even
when PointCloud2 says `/map`; mount offset (1.19 m forward, 0 m lateral) and
odometry yaw/translation create the map-frame eight-point footprint. Output
keeps the cloud timestamp. Receipt age is used for freshness without assuming
that MORAI header time and ROS node clock share an epoch. Planning checks map
frame, complete groups, finite coordinates, ordering and receipt freshness.
Malformed/degenerate data stops; it never becomes apparent free space.
Obstacle-free startup preserves the existing baseline without requiring a
perception publisher. After the first received observation, a missing/stale
stream latches STOP. This compatibility policy does not certify an unobserved
road as obstacle-free; enabling baseline driving still requires its existing
readiness/lease and independent operational supervision.

Observed polygons are conservative convex hulls of the supplied footprint
samples. Overlapping static observations are merged to retain previously seen
extent and bound memory growth. Empty detections do not erase known obstacles.
New extents invalidate cached path checks and the return reservation. This is
not an object tracker or proof of unobserved/occluded obstacle extent.

## Fixed lattice construction

For each valid LEFT/RIGHT lane pair, seed transition lengths are 12/16/20 m.
Each proposal includes entry, adjacent hold, reserved return and original-lane
suffix over a common original-road prediction horizon (default up to 80 m,
bounded by actual reference support and the front overhang).

Before lateral entry, a **fixed preparation arc** carries measured turn curvature
for `production lookahead + speed * command_delay`. This prevents the first PP
target from demanding an instantaneous steering step. It consumes real entry
distance and never bypasses the obstacle deadline. The polynomial target comes
from the actual adjacent centerline, tangent and estimated terminal curvature.

For each world coordinate, quintic Hermite coefficients use:

```text
a0=p0; a1=L*t0; a2=0.5*L²*k0*n0
d0=p1-a0-a1-a2
d1=L*t1-a1-2*a2
d2=L²*k1*n1-2*a2
a3=10*d0-4*d1+0.5*d2
a4=-15*d0+7*d1-d2
a5=6*d0-3*d1+0.5*d2
```

The nominal return is reserved on the held lane and targets the original MGeo
centerline. It is approved again against the actual pose/turn before handoff.
Production local-path subdivision preserves native reference corners. Trimming
approved paths never refits their geometry. Rejected partial entry geometry is
kept only for red diagnostic display if a return cannot be constructed.

Multiple candidate transitions are generated during search. Counters distinguish
`generated_transition_count` / `candidate_lane_batches` from `entry_commits` /
`return_commits`. One selected entry object and one selected return object remain
fixed throughout a completed maneuver; errors do not generate continuations.

## Hard safety and controller model

Candidate cost never overrides a rejection. Validation requires valid directed
topology, forward progression, verified drivable area, complete solid-body and
obstacle-polygon separation including between path samples, curvature, speed-
dependent lateral acceleration, production-PP steering/clamp/rate response,
entry/braking distance and a complete return reservation.

Full-body containment subtracts verified convex road cells from the solid query
polygon. Checking vertices or only the perimeter would miss interior road holes.
Continuous inter-sample body motion is conservatively enclosed by the endpoint
body hull inflated by half-step travel plus body-radius times half-angle.
Road-boundary margin defaults to 0.0 m; solid-body obstacle margin remains
0.25 m. The prior 1.20 m reference clearance floor is retained as an ADDITIONAL
gate and cannot replace full-body validation. None of these minima are relaxed.
The measured current body is also checked in NORMAL before passed-obstacle
pruning, so tracking error cannot hide a collision beside a safe-looking
reference centerline.

Production `PurePursuit.compute()`, `calculate_lookahead_distance()` and
`to_morai_front_steer()` are directly called. A 2 s no-slip bicycle rollout at
the production 30 Hz control rate starts from measured pose/speed and angular-
rate/speed curvature. A causal command queue applies the 0.1 s assumed delay
to every steering update. The same production LocalPathPlanner window (default
20 m) is shared by rollout, ROS output and offline Control input; clipping that
window never regenerates the fixed maneuver. Short windows that cannot support
the rollout and PP lookahead are rejected. Planned path curvature and actual
turn are separate. Every predicted body, obstacle margin, tracking distance, clamp and
wheel-rate change is checked. The initial 0.6 rad/s wheel-rate bound is a
conservative model parameter, not a measured Sonata actuator guarantee.
Lateral acceleration 1.5 m/s², braking 2 m/s² and reaction 0.3 s are also explicit
kinematic assumptions, not tire/physical-braking certification.

Candidate validation can exceed one 50 ms planning period. While armed, the ROS
adapter publishes empty path + zero speed BEFORE searching a new maneuver or
validating changed obstacle geometry. A new/slow validation result cannot
immediately authorize motion on its old pose: state `VALIDATING` keeps STOP
until three distinct fresh odometry frames pass the fixed-path/current-controller
checks within the planning period. Input loss breaks confirmation. This is an
admission guard, not a second nominal avoidance FSM. Diagnostic geometry/messages
are cached separately and cannot become a control fallback.

## Cost and lifecycle

For common original-road support H, candidate arc coordinate l and reference
distance d, use distance-integrated normalized costs:

```text
J = 1.0*C_reference + 1.5*C_adjacent + 0.2*C_curvature
    + 0.2*C_lane_switch + 0.1*C_progress
C_reference = integral(d² dl) / (H * 1 m²)
C_adjacent = integral(nearest center is target lane dl) / H
C_curvature = integral(kappa² dl) / (H * (1/m)²)
C_lane_switch = number of switches / 2
C_progress = trajectory arc length / H - 1
```

Nearest-center association is only a soft occupancy cost, never a fabricated
road boundary. Safety is evaluated separately. The fixed direction suppresses
side oscillation; positive reference/occupancy costs prefer return. Explicit
physical acquisition guarantees completion rather than relying on cost alone.
Both directions use the same original-road prediction support. A shorter
adjacent chain is rejected with `INSUFFICIENT_COMMON_HORIZON`; it cannot appear
cheaper through a truncated score. This conservatively rejects some short-road
opportunities rather than fabricating unverified successor geometry.

`NORMAL → AVOID_ENTRY → AVOID_HOLD → RETURN → NORMAL` is the nominal lifecycle.
`BLOCKED` latches safety failure until explicit reset. ENTRY/RETURN acquisition
requires end-region progress, <=0.15 m target lateral error, <=8-degree heading
error, forward direction and three consecutive valid planning observations.
HOLD returns to the actual target centerline. Return is handed off only when
the rearmost actual transformed body corner has passed the remembered obstacle
rear plus 0.5 m. Unsafe return, new obstruction or tracking loss stops without
flipping side or repeatedly refitting a path.
Editing the route during an active/blocked maneuver latches STOP instead of
erasing known obstacles or bypassing the fixed-path contract. An explicit
disarmed reset is required. Normal route changes preserve existing behavior
and retain observed obstacle memory for revalidation on the new reference.

`PlanningFSM` remains the separate existing system readiness/drive-lease arbiter.
Its emergency stop outranks drive enablement; blocked/validating outputs have
empty `/planning/local_path` and zero `/planning/target_speed`. Normal speed
classification, hard drive cap and finite lease are preserved. Avoidance cap is
3 km/h by default, and a high initial measured speed must decelerate before entry.

## ROS and RViz

Core interfaces stay `/localization/odometry`, `/planning/obstacles`,
`/planning/global_path`, `/planning/local_path`, `/planning/target_speed` and
the existing normal diagnostics. New diagnostics are:

- `/planning/avoidance_state`, `/planning/avoidance_debug`
- `/planning/lattice_candidate_markers` (geometry, costs, rejection reasons, FSM)
- `/planning/original_lane_center`, `/planning/left_lane_center`, `/planning/right_lane_center`
- `/planning/avoidance_selected_path`, `/planning/avoidance_return_path`

RViz keeps Global/Local mirrors and adds these roles. Rejected paths stay red;
they are not sent to Control. Stable IDs and explicit stale deletion avoid flicker.
Legacy lane-change publishers/displays and 16 old YAML keys are removed from the
active path; 45 existing normal configuration values remain unchanged.

Build/restart manually (these commands were NOT used to run MORAI):

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
# Stop previously started instances before manually restarting the chosen launch.
ros2 launch edgenode_bringup autonomy.launch.py
ros2 launch edgenode_perception lidar_obstacle.launch.py
ros2 launch edgenode_planning route_editor.launch.py use_rviz:=true
```

Keep `enable_drive=false`. With no boundary data, use only stationary diagnostics:
verify the obstacle footprint, candidate rejections, empty selected control path,
zero target speed and emergency/validation state. There is no authorization for
lane-change driving on the current map. A future low-speed trial first needs
verified registered road boundaries and independent sensor/braking/latency checks.
While disarmed, an explicit reset is available via
`ros2 service call /planning/reset_avoidance std_srvs/srv/Trigger '{}'`.
Reset never bypasses the next input/safety validation; clock rewind requires
disarmed source/node restart to re-establish ordered sensor timestamps.

Current normal YAML still permits 50 km/h, as requested. With the model's
0.3 s reaction and 2 m/s² deceleration, that speed needs about 52.39 m before
front-overhang/margin allowance. The preserved LiDAR ROI is only 20 m sensor-
forward (+1.19 m mount offset). Late discovery therefore cannot prove a
collision-free physical stop. `BRAKING_DISTANCE` demands immediate STOP; it is
not a claim that impossible stopping distance has become safe.

## Verification and cleanup status

Tests distinguish pure geometry, production-PP offline kinematics and real ROS
transport on isolated/mock outputs. No future bag poses, ideal follower, MORAI
drive or physical performance claim is used. Raw S4 fixtures are retained and
unsafe recorded inputs remain rejected. All limits and model assumptions must
be checked before any later physical test.

The full 3/10 km/h capability scenarios use an EARLY known obstacle and 50 m
Planning preview (the Control/rollout window is still 20 m). They must not be
reported as default sensor/Planning configuration success. With the unchanged
20 m local preview and LiDAR 20 m range + 1.19 m mount offset, the synthetic
two-lane scene stops: 12 m fails solid-body road containment, while 16/20 m miss
the entry deadline. The 16 m candidate needs about 22.6--22.8 m front distance,
more than the first observation's approximately 21.2 m. Even an optimistic
complete footprint on first observation does not pass. No sensing range,
transition seed, boundary or steering threshold was changed to hide this.
With early obstacle knowledge but the default 20 m preview, 3 km/h completes;
10 km/h conservatively rejects ENTRY with STEERING_RATE. This is a rollout
delay-initialization limitation, not a measured actuator-limit violation:
Planning does not observe the pending command queue and holds measured turn
until the first delayed new command. A future command-history model requires
independent review; the gate remains active here. See the numerical report.

Following explicit approval of the 46-file manifest, 45 obsolete source/test/doc
files were removed. The original Autoware Apache-2.0 license/copyright notice is
retained conservatively for historical and recovery-source distribution; this
does not reactivate the retired constant-jerk implementation. The 71 normal-
following tests remain in `test_path_following_node.py`; original failure
fixtures and protected production subsystems are unchanged.

The pre-cleanup Git changes and all 46 original files were saved with verified
SHA256 hashes before deletion. Only the Planning package's generated build and
install trees were relocated to that checkpoint, then rebuilt from current
source. Shared geometry source outside the approved deletion list was retained.
The only active avoidance path remains `planning_node → StaticObstaclePlanner`.

The original pre-cleanup report and manifest remain historical records under
`/workspace/logs/static_avoidance_rebuild/`. Actual deletion, license review,
checkpoint recovery, package-artifact cleanup and post-cleanup full build/test
results are recorded in
`/workspace/logs/s4_cleanup_20261106T043805460188Z/REPORT.md`.
