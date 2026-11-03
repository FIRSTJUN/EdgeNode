# S4 avoidance feasibility and low-speed baseline

Entry/recovery lengths are now adaptive; see
[adaptive transition design and stationary validation](s4_adaptive_transition.md).

This change follows the real wide-road collision bag
`kcity_s4_static_obstacle_5kph_wide_01`. That recording commanded 50 km/h and
reached 24.333 km/h. Its clearance-safe selected paths exceeded the existing
Control steering geometry; none of the checks here certify a new MORAI pass.

## Candidate approval

Every AVOIDING candidate and every RECOVERING path must satisfy both:

```text
sample_clearance >= max(legacy_floor, ego_half_width + safety_margin) = 1.20 m
max_abs_XY_curvature <= usable_curvature_limit
usable_curvature_limit = curvature_safety_factor
                       * tan(max_front_steer_normalized * max_wheel_angle_rad)
                       / wheelbase_m
                       = 1.0 * tan(0.70 * 0.610865) / 2.84
                       = 0.1604669205 1/m
```

Curvature is the absolute circumcircle curvature of each consecutive distinct XY
triple: `2*abs(cross(b-a,c-b))/(|b-a|*|c-b|*|c-a|)`. Duplicate consecutive XY
samples are skipped. Nonfinite/degenerate geometry and collinear reversals are
infeasible. Factor range is `(0,1]`; default 1.0 uses the existing command limit
without inventing an additional steering allowance. Normal-drive reference paths
retain the existing SpeedPlanner behavior; this is not a general routing change.

`test/fixtures/s4_wide_failed_paths.json` contains three **recorded** Local Paths
and their reconstructed input snapshots, with source bag SHA256. Their curvature
is 0.232200, 0.507899 and 1.219993 1/m. Each recorded path remains rejected. With adaptive generation, the 9.477 s
snapshot can produce a different feasible candidate; the two later snapshots
remain BLOCKED. Fixtures do not depend
on an ignored bag/cache being present during tests.

`autonomy.launch.py` reads the three geometry values directly from the same
`control.yaml` it passes to unchanged Control. Standalone Planning has matching
defaults/config values. Planner geometry/safety parameters are startup read-only;
restart Planning and Control together after changing geometry. Do not override
only one node's geometry. The current Control clamp, Pure Pursuit, Dynamic
Lookahead, PID, Localization and Perception self-mask are unchanged.

## Direction continuity and BLOCKED release

The last approved offset is maneuver state. If it remains collision-safe and
steer-feasible, prefer it even if the opposite side has a better instantaneous
score. Otherwise try another safe, feasible offset on the same side.

If only the opposite side is available, the current cycle is BLOCKED. An
opposite-side proposal is only eligible on subsequent BLOCKED cycles when actual
lateral displacement is within the existing 0.15 m recovery threshold of the
reference center. It must pass the same generated-path clearance/curvature
checks and release confirmation. A laterally committed maneuver remains BLOCKED;
there is no automatic sign flip while Ego remains displaced to that side.
This is deliberately conservative and can require stopping/repositioning or a
new verified maneuver; no maneuver-history reset is exposed as a driving bypass.

Default `lattice_blocked_release_cycles=3` requires the same safe behavior and
offset sign on three consecutive valid Planning cycles. One unsafe cycle,
missing local path, or map-match loss resets confirmation. A changing proposed
behavior/sign also resets it. `RELEASE_PENDING` still requests target=0 through
the existing emergency-stop FSM priority. This counts Planning cycles, not
independent LiDAR frames or proof that physical vehicle speed is zero.
Route replacement clears maneuver state; match fluctuations do not.

## Recovery before and after

Previously every candidate embedded a return starting at the first relevant
sample's nearest route vertex plus 1 m. For the wide bag that was before the
observed NPC far end:

| Recorded path time | NPC near..far (m) | Old return start (m) |
|---|---|---|
| 9.578 s | 6.53..10.60 | 7.341 |
| 11.177 s | 4.03..8.39 | 4.833 |

Now an AVOIDING path blends to the avoidance offset and **holds it through the
entire remaining horizon**, including future points beyond the obstacle. It has
no embedded return. Longitudinal front/rear extents use segment projection, with
signed tangent extrapolation at local-path endpoints. Front means the near end
encountered along route; rear means the far end that must be passed.

World-space front/rear anchors retain the most conservative observed extent.
Because the unchanged anonymous PoseArray has no IDs, extents conservatively
union all received samples after a relevant obstacle activates avoidance; they
do not infer an undocumented 8-point grouping. Multiple objects may therefore
prolong the hold. A shrinking/missing detection cannot erase the remembered far
end. If detections disappear before passage, return BLOCKED with
`OBSTACLE_NOT_PASSED`, not RECOVERING.

Only **actual Ego reference progress > remembered rear +
`lattice_obstacle_rear_margin_m`** permits RECOVERING. Default margin is 0.5 m,
an explicit reference-point buffer, not any vehicle half-length/overhang.
At equality, recovery is still prohibited. Recovery uses the existing smoothstep
with a preferred 10 m length, extended as necessary for the unchanged curvature
limit and bounded by the available reference horizon.
After actual lateral displacement reaches the existing 0.15 m threshold, CLEAR
releases maneuver history. `hold_half_length_m=1.0` is retained only for legacy constructor compatibility;
it no longer sets entry or return distance and is not an Ego half-length.

TODO: replace the reference-only rear buffer with verified Ego rear swept
clearance when MORAI geometry, collision mesh and localization reference point
are measured. No Sonata half-length, front/rear overhang or GPS-to-rear-axle
translation is guessed here. The segment-to-obstacle-point collision model and
its general 8-point approximation limitations remain. Sampled curvature and
near-center sign-switch restrictions do not prove initial-yaw continuity,
closed-loop tracking, lane-boundary legality, stopping distance, or full-body
collision clearance. Validate those separately before authorizing a real pass.

## S4 final-command cap and diagnostics

`obstacle_avoid_speed_cap_kmh=3.0` is applied in Planning **after** SpeedPlanner
classification and FSM arbitration whenever FSM=OBSTACLE_AVOID, including
RECOVERING. Final command is the minimum of recommendation, existing drive cap
and S4 cap. It is read every cycle; invalid/nonpositive/nonfinite cap fails closed
in avoidance instead of falling back to cruise. NORMAL_DRIVE retains the existing
cruise and drive cap. The existing drive enablement, lease and FSM stop priority
remain in force.

Diagnostics published each cycle:

| Topic under `/planning/` | Meaning |
|---|---|
| `obstacle_avoid_speed_cap_kmh` | Live validated configured cap (0 if invalid) |
| `applied_speed_cap_kmh` | Effective motion cap, 0 in non-motion states |
| `lattice_entry_length`, `lattice_full_offset_start_s` | Solved entry length and full-offset endpoint, m |
| `lattice_recovery_length` | Solved recovery length, m |
| `lattice_curvature_limit` | Actual planner usable limit, 1/m |
| `lattice_max_curvature` | Selected/diagnostic candidate maximum, 1/m |
| `lattice_obstacle_front_s`, `lattice_obstacle_rear_s` | Signed remembered extent relative to current local reference start, m |
| `lattice_blocked_reason` | `NO_SAFE_FEASIBLE_CANDIDATE`, `DIRECTION_CHANGE_REQUIRES_STOP`, `RELEASE_PENDING`, `OBSTACLE_NOT_PASSED`, `RECOVERY_UNSAFE`, or unavailable-reference reason |

The existing lattice status/offset/clearance, target_speed and FSM topics remain.
BLOCKED clearance is still the best candidate sample clearance, not clearance of
the reference path published while stopping. A pending-release diagnostic can
show sufficient clearance while target remains zero. Missing-detection hold uses
clearance=0 (unknown is not reported as free space). NaN extent/curvature means
unavailable, not a safe measurement.

Preflight readback examples; these commands do not arm driving:

```bash
ros2 param get /planning_node obstacle_avoid_speed_cap_kmh
ros2 param get /planning_node lattice_wheelbase_m
ros2 param get /planning_node lattice_max_wheel_angle_rad
ros2 param get /planning_node lattice_max_front_steer_normalized
ros2 param get /planning_node lattice_curvature_safety_factor
ros2 param get /planning_node lattice_obstacle_rear_margin_m
ros2 param get /planning_node lattice_blocked_release_cycles
ros2 param dump /planning_node
ros2 param dump /control_node
ros2 topic echo /planning/applied_speed_cap_kmh
ros2 topic echo /planning/lattice_curvature_limit
ros2 topic echo /planning/lattice_blocked_reason
```

## Next MORAI check, operated separately

1. Keep `enable_drive=false`, rebuild/restart, confirm readbacks and actual Control
   geometry/clamp. Verify this is the intended `k-city` workspace/node namespace.
2. For the **whole 3 km/h trial**, temporarily set the existing
   `max_drive_speed_kmh=3.0` too. The S4 cap only applies during avoidance; the
   temporary whole-trial cap prevents normal approach/exit accelerating to the
   unchanged 50 km/h cruise. Save both parameter dumps. No permanent cruise edit
   is required.
3. Verify static obstacle presence and observed extent through distance/yaw
   changes, and confirm geometry/reference points. Record collision events and
   NPC ground truth when available, raw LiDAR/GPS/IMU/TF, all existing planning and
   control topics, and the new diagnostics/parameter events.
4. Reject infeasible paths before motion. Verify BLOCKED remains target=0 and
   cannot clear on one safe cycle. Only a separately operated test should arm
   motion; this implementation never issues a MORAI drive command.
5. During the later trial confirm OBSTACLE_AVOID target<=3.0, actual speed within
   the measured trial tolerance, no unsafe sign flip, no return before actual
   rear+margin passage, feasible recovery, full-body observed clearance, and
   collision-free completion. A lower speed alone is not a safety certificate.
