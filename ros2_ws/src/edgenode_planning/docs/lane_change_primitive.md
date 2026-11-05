# Fixed Lane Change Primitive

This foundation generates a reference-relative shift once, retains that exact
path during ENTRY, and hands off to a fixed target centerline after static lane
acquisition. It has no ROS dependency or production caller. The existing
`AdjacentLanePlanner`, `PlanningNode`, Pure Pursuit and S4 remain as-is.

## Files and ownership

| File | Responsibility |
| --- | --- |
| `edgenode_planning/lane_change_types.py` | Frozen request/result/ENTRY path/target centerline, acquisition config and diagnostics, direction/state vocabulary. Target link is an opaque string; reason and return intent belong to the caller. |
| `edgenode_planning/lane_change_path.py` | Spatial constant-jerk shift, polyline sampling, finite/direction/curvature validation and target centerline snapshot preparation. |
| `edgenode_planning/lane_change_planner.py` | One generation attempt per request, separate static acquisition policy, consecutive confirmation and latched target centerline HOLD. |
| `test/test_lane_change_primitive.py` | LEFT/RIGHT, endpoints, derivatives, geometry failures, steering and repeated-call ownership tests. |
| `test/test_lane_change_acquisition.py` | Static acquisition, negative geometry cases, confirmation, one-way handoff, ENTRY/HOLD persistence and reset. |

The states are IDLE, ENTRY, TARGET_LANE_HOLD, RECOVERING, COMPLETE and BLOCKED.
RECOVERY is a compatibility alias for RECOVERING. `return_required` records
caller intent; only an explicit return request starts a return. There are no
obstacle front/rear concepts in the common planner.

## Autoware reference and provenance

Reference: **Autoware Universe**, Apache-2.0, local checkout
`/tmp/autoware_universe_ref`, commit
`1fc25fcec6219f39d6ae89c36b0720951c45173a`.
Upstream: <https://github.com/autowarefoundation/autoware_universe>.
Paths below are relative to its `planning/behavior_path_planner/` directory.

* `autoware_behavior_path_planner_common/src/utils/path_shifter/path_shifter.cpp`:
  `get_base_lengths_without_accel_limit()` gives longitudinal knots
  `[0, L/4, 3L/4, L]` and lateral knots `[0, D/12, 11D/12, D]`;
  `apply_spline_shifter()` applies the shift relative to reference arc length and
  retains the final offset after the shift end.
* `autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp`:
  documents constant-jerk cubic shifting and separates `ShiftLine` intent from
  the generated `ShiftedPath` and per-point shift lengths.
* `autoware_behavior_path_lane_change_module/src/scene.cpp`:
  `NormalLaneChange::hasFinishedLaneChange()` separates its fixed shift-line
  endpoint from lane acquisition checks. Its branches inspect signed progress
  and target polygon membership, or lateral/yaw deviation. This informed the
  separation of geometry from future completion policy; no completion logic is
  ported here.
* `autoware_behavior_path_static_obstacle_avoidance_module/src/scene.cpp`:
  `updatePathShifter()`, `addNewShiftLines()` and `isValidShiftLine()` separate
  candidate generation/validation from registered shift lines.
  `src/shift_line_generator.cpp`, `updateRegisteredRawShiftLines()` and
  `setRawRegisteredShiftLine()`, maintain registered shift intent as the vehicle
  moves. `PathShifter::removeBehindShiftLineAndSetBaseOffset()` preserves the
  terminal offset after removing passed lines.

Only the basic constant-speed, unlimited-lateral-acceleration **spatial shape**
is mathematically adapted. The adapted Python file carries Apache-2.0 attribution;
the license text and source notice are in
[`autoware_path_shifter_LICENSE.md`](autoware_path_shifter_LICENSE.md).
No acceleration-limited seven-point profile, C++ class translation, Lanelet2,
RTC, Autoware messages or Planner Manager is included. Autoware modules can
update their paths/shift lines; an entirely fixed maneuver path is this
primitive's explicit ownership rule, not a claim about all Autoware behavior.

## Geometry and checks

Let `s` be the original XY reference's arc length, `L = end_s - start_s`,
`u = (s - start_s)/L`, and `D` the signed terminal offset. LEFT is positive
along the forward tangent's left normal, RIGHT negative. The normalized profile
for `0 <= u <= 1` is

```
g(u) = (16/3) [u^3 - 2 max(u-1/4, 0)^3 + 2 max(u-3/4, 0)^3]
d(s) = D g(u)
P(s) = reference(s) + d(s) normal(s)
```

Outside the transition, `d=0` before and `d=D` after it. The implementation uses
reflection about `u=1/2` to reduce numerical cancellation near the endpoint.
The normalized spatial jerk `g'''` is `+32, -32, +32` across the quarter/half/
quarter intervals. `g`, `g'`, `g''` are continuous; both derivatives vanish at
the endpoints. At constant reference-station rate `v`, temporal jerk magnitude
would be `32 |D| v^3/L^3`. This generator has no speed or comfort contract.

XY/XYZ lists or tuples are copied to immutable XY tuples. Exact consecutive
duplicates are removed. Original vertices and shift boundaries/knots are
retained and sampled at no more than 0.25 m. Connected miter normals interpolate
between polyline corners. A sharp/reversed reference or a folded offset can be
rejected; the reference is never smoothed or extended. The C2 claim applies to
the lateral profile: arbitrary native map corners are not made C2 in XY.

Checks reject nonfinite inputs/output, invalid station/offset ranges, excessive
sampling size, reference reversals and nonforward steps. A conservative local
derivative bound additionally rejects possible between-sample folds. On a
straight reference, `|kappa| <= max|d''| = 8 |D|/L^2`; this continuous bound
rejects too-short shifts even for sparse two-point inputs. Curved references
also receive the existing `LatticePlanner.maximum_curvature()` sampled XY check
(only that stateless geometry helper is reused). These basic geometry checks
do not prove continuous vehicle footprint, corridor or controller safety on
arbitrary curved references.

Default Sonata command geometry is unchanged: wheelbase 2.84 m, wheel-angle
scale 0.610865 rad, normalized clamp 0.70. The limit is
`tan(0.610865 * 0.70)/2.84 = 0.1604669205 /m`. A caller may provide another
positive curvature limit; no automatic transition length search is performed.
The conservative straight-reference minimum length is about 13.21 m for a
3.5 m shift and 14.12 m for 4 m. Shorter requests return an error/BLOCKED.

## API and fixed lifetime

```python
from edgenode_planning.lane_change_planner import LaneChangePlanner
from edgenode_planning.lane_change_types import (
    LaneAcquisitionConfig, LaneChangeDirection, LaneChangeRequest, LaneChangeState,
)

planner = LaneChangePlanner(acquisition_config=LaneAcquisitionConfig())
request = LaneChangeRequest('adjacent_link', LaneChangeDirection.RIGHT,
                            return_required=True, reason='caller policy')
result = planner.start(request, [(x, 0.0) for x in range(41)],
                       start_s_m=5.0, end_s_m=25.0, lateral_offset_m=-3.5,
                       target_reference_points=[(x, -3.5) for x in range(41)])
assert planner.update(current_position=(6.0, -0.02), current_yaw=0.0) is result
for x in (23.5, 23.75, 24.0):
    hold = planner.update(current_position=(x, -3.5), current_yaw=0.0)
assert hold.state is LaneChangeState.TARGET_LANE_HOLD
assert planner.entry_path is result.path
assert hold.path.points == tuple((float(x), -3.5) for x in range(41))
```

Use `current_position=(x,y)` instead of `start_s_m` to project the longitudinal
start onto the reference. Projection only chooses a station: the shift starts
on the reference with zero offset, without fitting an off-center pose or yaw.
The ENTRY path includes the unshifted prefix and the terminal-offset suffix.
Its generator still accepts a reference plus constant terminal offset, without
inferring a correspondence to an arbitrary adjacent centerline. The caller
supplies the actual target centerline once through `target_reference_points`;
the target link ID performs no map lookup. The target is copied to immutable XY
tuples and checked using the same finite/reference/steering geometry helpers,
without shifting, smoothing or resampling it. Invalid target geometry blocks
the start attempt. Omitting the target preserves first-stage geometry-only
usage, but then acquisition cannot occur (`TARGET_REFERENCE_REQUIRED`).

The first `start()` consumes a snapshot of both geometries. During ENTRY,
`update()` returns the same result/path objects until acquisition is confirmed.
Then one new result selects the saved target centerline; the original path
remains available as `entry_path`. During HOLD, every update returns that exact
HOLD result/path object, without reevaluating ENTRY or returning to it on pose
jitter. Repeated `start()` with the same request never consumes changed geometry
or generates a path. A different request requires `reset()`; BLOCKED is also
cached. Reset clears both paths, confirmation and diagnostics. There is no
automatic path slicing, boundary re-fitting or recovery. This differs from the
legacy `_candidate()`/`_boundary()` per-cycle quintic continuation and rebasing.

## Static acquisition semantics

`LaneAcquisitionPolicy` uses the existing `lane_geometry.PolylineIndex`,
`sample()` and `cross()` helpers. Both indexes are built once. Each evaluation
projects vehicle position onto the fixed ENTRY and target centerlines, obtains
their local forward tangents, and computes:

* ENTRY progress `p = projected_entry_arc - transition_start_arc`, remaining
  distance `r = transition_end_arc - projected_entry_arc`, and fraction `p/L`.
  These are **shifted XY path arc lengths**, not nominal original-reference
  stations and not a caller-supplied or monotonically carried progress anchor.
* Signed target lateral distance `cross(target_tangent, position - projection)`.
  Positive means left of the target's forward direction, for either maneuver side.
* Heading error `atan2(cross(target_tangent, vehicle_forward),
  dot(target_tangent, vehicle_forward))`, wrapped to [-pi, pi].

All eligibility conditions must pass together:

| Configuration | Default | Meaning |
| --- | --- | --- |
| `entry_end_region_m` | 2.0 m | Remaining fixed transition arc must be <= this value. |
| `min_entry_progress_fraction` | 0.80 | At least this fraction of fixed transition arc must be reached. |
| `max_lateral_distance_m` | 0.15 m | Absolute signed target lateral error limit. |
| `max_heading_error_rad` | 8 degrees (0.13962634 rad) | Absolute target heading error limit. |
| `confirmation_updates` | 3 | Consecutive complete eligible update calls; configurable integer >= 2. |

In addition, the vehicle, target tangent and fixed ENTRY tangent must agree in
forward direction (all pairwise dot products positive). Reversing the target
polyline cannot authorize a reverse maneuver. Projection beyond actual path
support is rejected rather than extending a finite lane through its endpoints.
The minimum fraction prevents the 2 m end-region allowance covering the start
of a very short shift. It and the end-region allowance are validated dataclass
parameters, as are lateral, heading and confirmation thresholds.

The nominal end need not be crossed: on a 5..25 m right shift, positions
`(23.5,-3.5)`, `(23.75,-3.5)`, `(24,-3.5)` with yaw 0 confirm HOLD before station
25. Conversely, reaching station 25 while remaining on the original centerline
does not acquire the target. Acquisition after the transition end is also
allowed if the supplied paths still cover the position and all conditions pass.

Confirmation rejects a single transient center pass and two eligible samples
interrupted by a bad observation. Every failed condition, missing position/yaw,
invalid projection or missing target resets the count. Repeated `start()` does
not count as an observation. `planner.acquisition` exposes the last ENTRY
evaluation's reason, signed lateral/heading errors, progress and confirmation
count. These diagnostics remain the last confirmed ENTRY observation in HOLD.

Confirmation counts calls, without timestamps, elapsed dwell time, speed,
motion trend or yaw-rate history. Callers must supply fresh observations. This
is a static geometry acquisition contract: it does not predict future outward
motion or prove controller tracking stability. HOLD is a latched path handoff,
not a post-acquisition tracking safety monitor. No controller rollout is added.

## Validation and next boundary

The unit suite covers 3/3.5/4 m shifts in both directions, zero prefix/start,
persistent terminal offset, finite monotonic samples, a rotated reference,
gentle curved reference, C2 joins, jerk signs, short requests, sharp/reversed
references, folds, immutable storage and Sonata steering feasibility. A spy
verifies exactly one generation across 100 pose updates and repeated starts.
Legacy Planning tests and first-stage primitive tests are run unchanged.
Second-stage tests add large heading/lateral errors, end reached without
acquisition, early center proximity, transient passes, reverse direction,
invalid/missing observations, endpoint range, configurable thresholds, both
sides and a new maneuver after reset. Spies verify one ENTRY generation and one
target snapshot preparation across 100 ENTRY and 100 HOLD updates, including
mutation of the caller's input arrays. Execution results are recorded in
`/workspace/logs/s4_lanechange_acquisition/` and reported with this change.

For a straight 40 m reference with shift from station 5 to 25 m, both signs
produce 161 samples and symmetric feasibility results:

| Absolute shift | Sampled max curvature | Continuous straight-reference bound | Required max wheel angle |
| --- | --- | --- | --- |
| 3 m | 0.05706042 /m | 0.060 /m | 0.16065500 rad |
| 3.5 m | 0.06578372 /m | 0.070 /m | 0.18469654 rad |
| 4 m | 0.07416576 /m | 0.080 /m | 0.20759624 rad |

All remain below the command wheel-angle limit of 0.4276055 rad. This is
geometric steering feasibility only, without a Pure Pursuit tracking claim.

## Explicit return and COMPLETE

`request_return(request, reference_points, *, target_reference_points,
start_s_m=None, current_position=None, end_s_m, lateral_offset_m)` accepts a
common `LaneChangeRequest` only in TARGET_LANE_HOLD. The reference is the held
lane; the target reference is the caller's return destination. It calls the
same `generate_shift_path()` as ENTRY exactly once, then enters RECOVERING.
The common planner never interprets request reason or chooses a return trigger.

Repeated equal return requests return the cached result, including after
COMPLETE or a failed generation attempt. Changing a return request requires
reset; pose changes cannot re-fit the recovery path. `recovery_path` retains
that exact path object. Invalid/too-short return geometry enters BLOCKED.

The same `LaneAcquisitionPolicy` and confirmation thresholds evaluate recovery
against the original lane. Its historical `entry_*` diagnostics now describe
the active transition; `transition_progress_m` and `transition_remaining_m`
are generic aliases. Upon confirmation, COMPLETE selects the saved original
centerline and stays latched. Reset clears both requests, both transition paths,
centerlines, policy and confirmation before an independent maneuver.

Phase 3 tests cover both full direction combinations, two total shift-generation
calls (one per phase), 100 recovery updates, repeated return requests, negative
heading/lateral/direction/transient observations, broken confirmation, short
return BLOCKED and an independent second maneuver after reset.

Corridor/obstacle validation, dynamic safety, Pure Pursuit closed-loop validation
and production integration remain future work.
