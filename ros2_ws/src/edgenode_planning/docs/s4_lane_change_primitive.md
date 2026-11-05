# S4 fixed Lane Change Primitive mode

Select the opt-in startup parameter `obstacle_avoidance_mode: lane_change_primitive`.
The existing default `adjacent_lane` and `legacy_offset` modes are retained.
`AdjacentLanePlanner` remains unchanged as a legacy/reference implementation.

`StaticObstacleLaneChangeBehavior` accepts the same world-frame obstacle points
and ordered groups as the existing PlanningNode path. Perception's ROI, ground
removal, clustering and footprint production are unchanged. The behavior reuses
existing footprint extent association and remembers occupied points/rear after
missing frames. Repeated frames do not generate trajectories or move progress.

The behavior owns only obstacle blockage, valid directed adjacent lane selection,
candidate safety validation, and the return trigger. It creates
`LaneChangeRequest(target_link_id, direction, return_required=True,
reason='STATIC_OBSTACLE')`. Shape and acquisition are owned by the common
`LaneChangePlanner`. ENTRY and RECOVERY each call its one-shot generator once
for the approved maneuver, without fitting tracking errors each cycle.

`lane_change_primitive_length_m` defaults to 20 m. Transition length is the
greater of that distance and measured/requested speed times the existing
`lane_change_min_duration_sec` (3 s). The common primitive's unchanged curvature
gate still rejects infeasible requests. ENTRY must finish before obstacle front
minus the existing `lattice_transition_before_m` (5 m). No length search or
controller/speed tuning is performed.

HOLD follows the actual saved adjacent MGeo centerline. On actual position
projection along the original lane exceeding the remembered obstacle rear plus
`lattice_obstacle_rear_margin_m` (0.5 m), the behavior explicitly requests a
return. A supplied global progress number cannot authorize it. Recovery must
fit the available adjacent reference. A changed obstacle that invalidates the
return gate or clearance causes STOP instead of re-fitting a trajectory.

Geometry validation retains existing 1.2 m reference-center-to-footprint-sample
clearance, Sonata curvature/steering limits, forward direction and the exact
center-to-center corridor union with its existing numerical tolerance. Raw map
points are validated before lane-pair assembly can omit invalid points. Both
fixed geometry and actual pose/connectors are checked. The corridor is the
existing reference-center strip, not verified full vehicle/lane-boundary geometry.
Controller tracking viability is not implied by these static checks.

The entire immutable path is supplied to the existing Control interface. Safety
checks use its future suffix, so previously passed obstacles cannot invalidate
the already-passed prefix. COMPLETE publishes the validated original centerline;
the next cycle explicitly closes the lifecycle and resumes the normal local
path if clear. No vehicle pose is moved/snapped. Invalid geometry or any safety
failure returns an empty path and latches BLOCKED until explicit reset, preserving
PlanningNode's existing emergency-stop and zero-speed behavior.

The integration tests exercise NORMAL -> ENTRY -> TARGET_LANE_HOLD -> RECOVERING
-> COMPLETE for LEFT/RIGHT with synthetic observations, exactly one shift per
phase, missing/updated/duplicate detections, insufficient transition distance,
invalid/absent/reversed adjacency, unsafe target lane, actual corridor escape,
constructor mode selection, and PlanningNode empty-path/emergency-stop output.
These observation tests are a separate gate from production PP closed-loop tests.
