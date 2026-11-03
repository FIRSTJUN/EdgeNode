# S4 footprint collision baseline

This records the historical pre-feasibility baseline at `81a2add`. For the
current steering gate, direction continuity, actual-rear recovery and S4 cap,
see [S4 safe avoidance](s4_safe_avoidance.md). The legacy approach/return and
immediate recovery descriptions below are superseded by that change.

The first MORAI run detected the NPC but collided after selecting a path that was
clear of a single cluster centroid. Centroid separation is not vehicle clearance.
This revision keeps `/planning/obstacles` as `geometry_msgs/msg/PoseArray` and
changes its producer to emit observed footprint samples.

## Geometry and parameters

Perception computes the filtered 3D connected component's XYZ centroid and XY
min/max. Each cluster supplies four AABB corners and four edge midpoints in sensor
coordinates, then applies the existing mount translation and odometry yaw to
every point. The bounding box is computed **before** world rotation, so a rotated
sensor rectangle is not replaced by a larger axis-aligned world rectangle.

`/perception/lidar_obstacle_count` remains the number of accepted clusters.
`/perception/nearest_obstacle_distance` remains the minimum sensor-origin XY
centroid range. `/perception/lidar_obstacle_centers` is a separate map-frame
PoseArray for debugging, with one center per cluster. It is not the Planning input.

The Planning startup parameters in `config/planning.yaml` are:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `ego_half_width_m` | 0.95 m | Provisional MORAI Sonata trial value |
| `obstacle_safety_margin_m` | 0.25 m | Additional centerline-to-footprint margin |
| `lattice_clearance_m` | 0.55 m | Legacy minimum floor, not the effective threshold |
| `lattice_candidate_offsets_m` | 0, ±0.6, ±0.8, ±1.2, ±1.5, ±1.8, ±2.1, ±2.4 m | Signed left/right candidates |
| `lattice_transition_after_m` | 10.0 m | Length of the existing return smoothstep |

The effective threshold is:

```text
required_clearance_m = max(lattice_clearance_m,
                          ego_half_width_m + obstacle_safety_margin_m)
                     = max(0.55, 0.95 + 0.25) = 1.20 m
```

No existing physical width parameter was found in the project. The 0.95 m
half-width is a trial baseline, not a verified manufacturer specification. Width
and margin are not added again to the already represented obstacle width.
`LatticePlanner.obstacle_clearance_m` remains the effective value for old callers;
the optional constructor argument of that name is now a minimum floor. Historical
point-model replay can explicitly use zero width/margin and the old candidate set.
Old configs specifying 0.55 cannot silently disable the new 1.20 m inflation.

The score remains `abs(target_offset-current_offset)`; candidate order breaks ties.
Candidates are checked against **all** received samples, including adjacent-side
obstacles outside the center path's relevance corridor. The relevance corridor
still determines whether avoidance is needed. Distance is measured from each
obstacle sample to complete candidate **segments**, avoiding gaps between coarse
candidate vertices. Local Path resampling and progress code are unchanged.

With a centered 1.70 m-wide car, the nominal centerline gap needs approximately
`0.85+0.95+0.25=2.05 m`, so the 2.1 m candidate is the first symmetric option.
For the recorded, laterally asymmetric NPC, the static replay chooses -1.8 m.
This choice is scene-dependent; -1.8 m is not hard-coded.

## Transition profile and offline evidence

The quintic smoothstep equations, current-offset continuity, hold behavior, scoring,
RECOVERING/CLEAR logic and FSM priority remain. The return length alone changes
from 6 to 10 m and is exposed as `lattice_transition_after_m`. This follows a dense
0.02 m geometric inspection on a straight reference with obstacle progress 15.283 m:

| Offset | Return 6 m peak curvature | Return 8 m | Return 10 m |
| --- | --- | --- | --- |
| 1.8 m | 0.268 1/m | 0.155 1/m | 0.101 1/m |
| 2.4 m | 0.343 1/m | 0.201 1/m | 0.132 1/m |

The unchanged 2.84 m wheelbase, 0.610865 rad wheel angle and 0.70 command clamp imply
`tan(0.70*0.610865)/2.84 ≈ 0.1605 1/m`. The 10 m return is below this geometric
limit for those straight baseline cases; 8 m is insufficient for the largest
candidate. At 10 km/h, the 2.4 m/10 m illustrative lateral acceleration is about
1.02 m/s², compared with 2.64 m/s² for the old 6 m return. These are offline
geometry calculations, not a vehicle-dynamics or tracking validation.

The existing `transition_before_m=5.0` was stored but **not used** by the approach
profile. The actual approach blends from current offset over the available
`max(0.5, obstacle_s-hold_half_length_m)` distance. That equation is preserved; the
code does not claim that changing `transition_before_m` lengthens the approach.
Late detection/short approach and underlying route curvature can still exceed the
steering limit. No Control or SpeedPlanner changes were made.

Reproducible failed-run and curvature artifacts:
`/workspace/logs/kcity_s4_footprint_analysis/REPORT.md`, `replay_and_curvature.py`,
`curvature_sweep.csv`, and the diagnostic PNGs beside them.

## Limits exposed by the failed run

The run's first cluster is exactly the observed baseline: 64 returns, centroid
about `(14.093, 0.246)`, bounds x=13.790..14.941, y=-0.597..1.100 m. Recorded
AVOIDING clearance is 0.564..1.313 m (mean 0.926), measured against centers.
In a stationary snapshot the old model selects -0.6 m with clearance 0.979 m;
the new footprint model selects -1.8 m with sample clearance 1.303 m.

The failed run used an oversized `x<5 && abs(y)<1.5` self mask that removed
near-front returns. At bag time 10.414 s, 907 above-ground ROI returns become just
2 after the mask; obstacle_count becomes 0 at 10.417 s and Lattice changes to
RECOVERING at 10.493 s. At the strongest impact indication (11.074 s), reported
clearance is infinity because no obstacle is available. Infinity is not proof of
physical free space. A subsequent NPC-free MORAI measurement found 834 Ego
returns within x=0.301..1.934 m, abs(y)<=0.690 m, z=-0.493..-0.024 m. The self-mask
baseline is now `x<2.0 && abs(y)<0.8`; near-origin radius stays 0.3 m. This is a
self-mask-only follow-up: footprint inflation, candidates, required clearance,
profile, tracking and FSM logic are unchanged. New stationary validation remains
pending.

Eight points approximate an observed LiDAR box; they do not encode a continuous
polygon, cluster IDs or hidden surfaces in the unchanged interface. Inflation
uses a lateral half-width radius, not a swept oriented rectangle with front/rear
overhang. Lane boundaries, adjacent traffic and drivable-area constraints are not
part of this Lattice algorithm. A geometrically safe sample clearance therefore
does not authorize driving or certify that the original collision cannot recur.

## Next MORAI checks

Start with `enable_drive=false`, rebuild/restart Planning and the detector, and
read back the width, margin, candidate list and return-length parameters. For one
nondegenerate cluster, verify count=1, 8 Planning poses, 1 debug center, and the
same centroid range (initially about 14.095 m). Check world rotation with ego yaw
0/90 degrees and NPC left/right positions. Confirm the previous ±0.6/±0.8 choices
are rejected, selected sample clearance is at least 1.20 m when AVOIDING, and
unarmed target speed stays 0. All-candidates-blocked must retain BLOCKED and its
existing emergency-stop arbitration.

Keep Ego stationary and move one NPC through **14, 10, 7, 5, then 3 m**. After
restarting the detector, read back self_mask_max_x=2.0,
self_mask_half_width_y=0.8 and near_origin_radius_m=0.3. At every distance require
obstacle_count=1, 8 Planning poses, 1 debug center and a finite centroid range.
With the NPC removed, verify Ego returns do not create an obstacle cluster. These
checks validate the measured smaller mask; this implementation task does not run
MORAI or claim that the new static test has passed. Also check visible footprint
coverage, vehicle reference point, available lateral road space and path curvature.

After these stationary checks pass, a later separately operated run should begin
at a 3–5 km/h cap on a clear, controlled route, log the same topics plus debug
centers and collision events if available, and verify pass/return clearance before
repeating at 10 km/h. Check repeated ± direction switches, saturation, BLOCKED,
perception loss, RECOVERING and final CLEAR. This implementation task runs no
MORAI driving and changes no drive enablement or speed parameters.
