# S4 Stage 3: map and crossing validation

No K-City runtime DrivableArea was released. `avoidance_boundary_file` and
`avoidance_boundary_approved_sha256` stay empty; `enable_drive=false` remains.
Stage 1/2 sensor state, steady watchdog and latched BLOCKED behavior are retained.

## Runtime guards

The existing startup-only `s4_sensor_validation_enabled` mode also selects
`LaneNetwork(require_explicit_permission=True)`. The baseline mode keeps its
previous adjacency behavior and normal global/local following.

S4 requires literal boolean `True` for outward lane change and the adjacent
lane's reserved return direction. Missing, null, numeric/string substitutes,
explicit denial or unknown target fail closed. Both lanes must be ordinary
travel lanes (`link_type=6`) within this verification profile. Intersection,
bus-only, reversible and unknown lane types need separate crossing evidence and
are not admitted. This conservative profile is not a claim that every special
lane change is legally prohibited. Existing reciprocal adjacency, forward
direction, valid endpoints/speed and exact successor connectivity still apply
to every pair section. Nothing is inherited across missing successor metadata.

Positive flags are necessary metadata, not a surveyed road/crossing approval.
They never override body containment, obstacle margins, entry/return distance,
controller feasibility or a missing boundary. The actual K-City data has no
`can_move_*_lane` fields, so no adjacent lane is admitted in this mode.

The DrivableArea cell-union contract remains convex finite XY cells, map frame
and source provenance. S4 additionally requires an **operator-reviewed exact
artifact SHA256** in `avoidance_boundary_approved_sha256`, `data_kind=real_map`
and an exact `mgeo_sha256` dictionary for `global_info.json`, `node_set.json`,
`link_set.json`. The pin is read-only at runtime. An empty pin, changed artifact,
synthetic artifact or different map revision fails initialization without a
fallback. Empty boundary input still returns no area and preserves normal
obstacle-free following / blocked avoidance.

Hash pinning establishes identity and explicit operator admission; it does not
perform or prove the external review. No approved hash, boundary file or
real-map artifact is supplied here. The legacy loader contract remains available
outside S4 mode. Unit-test artifacts only exercise validation contracts and do
not establish real K-City geometry.

## Official code definitions

Source: National Geographic Information Institute,
[정밀도로지도 구축 매뉴얼, 2019.12](https://www.ngii.go.kr/kor/contents/view.do?sq=1195&board_code=contents_data),
[official PDF](https://www.ngii.go.kr/other/file_down.do?sq=76349).
PDF SHA256: `93958d4079dcdde311368753049f0df95238122ab6bde0ea241fed89499a6541`.

- pp.87-89, tables 54-57: B2 Type describes color, multiplicity and line pattern;
  Kind describes regulatory role. R/L link IDs refer to the right/left links
  relative to the marking. `212/503` is a white single dashed lane divider;
  this is only a marking candidate and never unconditional crossing permission.
- `214` is named in the table but its detailed definition is a dash. The two
  observed occurrences are treated as undefined for admission, not extrapolated
  from other codes. Other/unknown and directional mixed markings never receive
  automatic permission from this tool.
- p.57, table 24: A2 LinkType `1` is an intersection path and `6` is ordinary
  travel. Adjacency fields (p.58) identify neighbors, not a complete crossing
  authorization. The tool checks SHP/MGeo LinkType agreement.
- p.64, tables 31-32: A3 polygons describe limited section types; they are not
  a complete ordinary-road surface mask. Kind `7` is an autonomous-driving
  exclusion. Absence of such a polygon is not proof that all other exclusions
  have been represented.
- pp.103-104, tables 67-68: C3 is facility geometry with upper/lower attributes,
  not a continuous drivable-area polygon. The delivered `Ref_ID` spelling differs
  from the manual's `Ref_FID`; delivery-version applicability needs confirmation.

Manual interpretation is not proof of dataset completeness or MORAI scene
registration. The delivered records say Version 2019 with older update dates;
the actual delivery schema and scene revision still need confirmation.

## Reproducible offline audit

```bash
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
python3 -m edgenode_planning.hdmap_audit \
  --mgeo /workspace/local_data/kcity_mgeo \
  --hdmap /workspace/local_data/kcity_hdmap \
  --output /tmp/kcity_map_diagnostic.json
```

The reader checks SHP/DBF record lengths/order, IDs, finite XYZ and multipart
structure. It uses the existing pyproj dependency to check all PRJs are UTM52N
(EPSG:32652), then subtracts the exact MGeo origin. Source file hashes are in
the report. Existing linear segments may be sampled, but missing parts are
never joined, extrapolated or interpolated across. Boundaries that intersect
themselves or each other are rejected. Original corners plus a 1m grid are
used for normal-intersection diagnostics; a 2.36m..12m preliminary width screen
is explicitly not a swept-body or road-surface certificate. Sampled support
spans are observations, not approved continuous corridors or polygons.

The report is `artifact_type=diagnostic_map_audit`, always
`approved_for_runtime=false`, contains no `cells`, exports no runtime artifact
or approval pin, and cannot be loaded by DrivableArea. Output into either source
map directory is rejected. Counts from different preliminary algorithms (such
as the earlier 94-link screen) are not interchangeable with this stricter
whole-link support test.

Current data confirms 352 MGeo records (254 ordinary, 98 lazy), 809 SHP A2,
253 shared IDs, 1574 B2, 131 shared-ID links with associations on both sides.
One ordinary MGeo link `L1` has no SHP counterpart. Maximum sampled XY deviation
between shared centerlines is about 0.173m; sampled Z deviation after origin
subtraction is about 0.027m. These are vertex-to-segment measurements, not a
continuous Hausdorff bound or verified vertical datum.

`075/096` share `B219BS010377` (`212/503`), with outer observed markings
`B219BS010415` and `B219BS010396`. Each centerline is only about 21.7m; sampled
two-sided support is approximately s=0.5..21.5m, with missing end support.
`624/625` are intersection paths (`LinkType=1`) with no shared B2 marking.
These facts must not be bridged into a longer approved maneuver corridor.

Only one complete whole-link normal-support screen passes (`398`, about 25.2m),
and 75 links have some sampled support. Neither is road approval. No reviewed
surface/exclusion mask, explicit crossing permission or valid full-maneuver
release exists. C3 has 690 observed features (all LowHigh=2), only seven closed
observed parts; closing/filling their gaps or choosing inside/outside would
invent geometry. A3 has six small section polygons. No DrivableArea is produced.

## Stage 4 and restart

Stage 4 must reconcile actual sensor range, hazard preview, entry preparation,
deceleration/stop distance and return support. Default 20m sensor ROI plus
1.19m offset cannot satisfy the demonstrated approximately 22.6m minimum front
distance for a 16m transition at 3km/h. An 80m lattice prediction does not extend
sensing or make an unsupported lane successor available. At 50km/h the existing
model needs about 56.43m from the rear-axle reference including front/margin;
it is not a measured braking guarantee. No ranges/speeds were changed here.

Only Planning needs rebuilding for Stage 3:

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select edgenode_planning --symlink-install
source install/setup.bash
# Stop previous Planning/LiDAR instances in their own terminals first.
export ROS_DOMAIN_ID=182
export ROS_LOCALHOST_ONLY=1
ros2 launch edgenode_bringup s4_validation.launch.py use_sim_time:=false
```

This is the existing disarmed Stage 2 launch (no Control/Localization). Inspect
the gate and empty boundary/pin; missing input is expected to STOP:

```bash
ros2 param get /planning_node enable_drive
ros2 param get /planning_node s4_sensor_validation_enabled
ros2 param get /planning_node avoidance_boundary_file
ros2 param get /planning_node avoidance_boundary_approved_sha256
ros2 topic echo /planning/s4_sensor_state --once
```
