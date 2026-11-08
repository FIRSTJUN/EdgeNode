# LiDAR obstacle perception for S4

For the opt-in S4 sensor gate, `/perception/lidar_status` distinguishes successful
empty detection from invalid, missing, stale, duplicate and reordered input.
The legacy PoseArray and ROI remain compatible. See the
[sensor contract, timing requirements and disarmed validation launch](../edgenode_planning/docs/s4_sensor_validation.md).

`lidar_obstacle_node` filters local MORAI LiDAR, clusters returns, and publishes
**8 observed footprint samples per cluster** to the existing world-frame Planning
PoseArray. It also keeps cluster counts, centroid range, and separate debug centers.
The camera `perception_node` remains a separate executable.

## Start

```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch edgenode_perception lidar_obstacle.launch.py
```

This launch starts only the detector. It does not start Planning/Control or change
`enable_drive`. `params_file:=/absolute/path/lidar_obstacle.yaml` selects an alternate
config. Parameters are startup-only/read-only: edit the file and restart to change
these baseline values. Rebuild and restart Planning too for new Lattice settings.

## Topics

| Direction | Default topic | Type |
| --- | --- | --- |
| Subscribe | `/lidar/points` | `sensor_msgs/msg/PointCloud2` |
| Subscribe | `/localization/odometry` | `nav_msgs/msg/Odometry` |
| Publish | `/planning/obstacles` | `geometry_msgs/msg/PoseArray`: 8 footprint poses per cluster |
| Publish | `/perception/lidar_obstacle_centers` | `geometry_msgs/msg/PoseArray`: 1 debug centroid per cluster |
| Publish | `/perception/lidar_obstacle_count` | `std_msgs/msg/Int32`: actual cluster count |
| Publish | `/perception/nearest_obstacle_distance` | `std_msgs/msg/Float32`: sensor-origin centroid XY range |

Topic parameters are `lidar_topic`, `localization_topic`, `obstacles_topic`,
`cluster_centers_topic`, `obstacle_count_topic`, and `nearest_distance_topic`.
LiDAR QoS is **BEST_EFFORT / VOLATILE / KEEP_LAST depth 5**; odometry and outputs
use RELIABLE / VOLATILE / KEEP_LAST depth 10.

Both PoseArrays use `frame_id=map`, the cloud's acquisition stamp, `z=0` and identity
orientation. A cluster's sensor-local XY AABB contributes four corners and four
edge midpoints in perimeter order. Degenerate bounds retain 8 possibly duplicate
samples. The arithmetic centroid is published separately for diagnostics; Planning
receives the footprint samples. No cluster grouping metadata is added to PoseArray.

Count is the number of accepted connected components, **not** the number of poses.
Nearest distance remains `min(hypot(center_x,center_y))` before mount translation,
so the observed `(14.093,0.246)` centroid is about 14.095 m. It is not bumper or
world-origin clearance. Empty/unavailable detections publish empty arrays/count 0/
range NaN. Clustering does not prove object class or that an obstacle is stationary.

## Initial MORAI Sonata filter parameters

| Parameter | Default |
| --- | --- |
| `roi_min_x`, `roi_max_x` | 1.0, 20.0 m, inclusive |
| `roi_min_y`, `roi_max_y` | -4.0, 4.0 m, inclusive |
| `ground_z_threshold` | -1.0 m; remove z <= threshold |
| `near_origin_radius_m` | 0.3 m; remove hypot(x,y) < radius |
| `self_mask_max_x`, `self_mask_half_width_y` | 2.0, 0.8 m; remove x<2 and abs(y)<0.8 |
| `cluster_distance_m`, `min_cluster_points` | 0.8 m 3D neighbour radius, minimum 5 points |
| `lidar_offset_x_m`, `lidar_offset_y_m` | 1.19, 0.0 m |
| `localization_timeout_sec` | 0.6 s, node receipt-time age |

Pipeline: finite -> near-origin -> ROI -> ground -> self mask -> KD-tree Euclidean
connected components -> centroid and bounds -> 8 footprint points -> world poses.
Neighbour chains belong to one component; complete components below the minimum
size are discarded. PointCloud2 parsing respects named FLOAT32/FLOAT64 XYZ fields,
offsets, extra fields, byte order and row padding without mutating the buffer.

## Coordinates and unavailable inputs

The current MORAI cloud says `/map`, but its XYZ is observed to be sensor-local.
The header does not bypass or select the transform. For **each** footprint point:

```text
vehicle_x = lidar_x + lidar_offset_x_m
vehicle_y = lidar_y + lidar_offset_y_m
world_x = ego_x + cos(yaw)*vehicle_x - sin(yaw)*vehicle_y
world_y = ego_y + sin(yaw)*vehicle_x + cos(yaw)*vehicle_y
```

The same transform is applied separately to the debug centroid. Yaw comes only
from normalized odometry quaternion, with the existing ROS counter-clockwise +yaw
convention. There is no direct MORAI heading use or new heading correction. Mount
z=1.11 m does not affect the local ground threshold or planar output. No sensor
mount rotation or time interpolation/motion compensation is introduced.

No valid map pose, invalid pose/quaternion or stale odometry produces empty output
and a warning throttled to 5 seconds on incoming clouds. Empty clouds clear prior
output. Missing LiDAR messages do not fabricate a free-space result: the node stays
alive and Planning keeps its last message. Count 0 does not distinguish unavailable
input from clear space, so monitor warnings and topic rates.

**Measured self-mask baseline:** NPC-free MORAI measurement contained 834 Ego
returns at x=0.301..1.934 m, y=-0.690..0.690 m and z=-0.493..-0.024 m. There were
653 returns at x=0.3..1.0 m, none at 1.0..1.5 m, 181 at 1.5..2.0 m, and none beyond
2.0 m. The new `x<2.0 && abs(y)<0.8` mask covers this measured envelope without
masking front obstacles beyond 2.0 m. `near_origin_radius_m=0.3` and the other
filtering/clustering/footprint settings are unchanged. Default ROI x>=1.0 already
removes the earliest returns; the self-mask test separately includes the full
measured x envelope so ROI cannot conceal a mask error.

The previous `x<5 && abs(y)<1.5` mask removed close NPC returns and caused count 0
and premature RECOVERING in the failed run. The measured smaller mask replaces
that oversized baseline. Stationary MORAI validation of the new values is still
pending; this change does not introduce tracking or prove collision-free driving.

## Stationary validation and later driving

Keep the existing Planning node unarmed, then start the detector:

```bash
ros2 param set /planning_node enable_drive false
ros2 param get /planning_node enable_drive
ros2 topic info /lidar/points --verbose
ros2 topic echo /perception/lidar_obstacle_count
ros2 topic echo /perception/nearest_obstacle_distance
ros2 topic echo /planning/obstacles --once
ros2 topic echo /perception/lidar_obstacle_centers --once
```

After rebuilding, restart the detector: parameters are read-only at runtime.
Confirm the running values before moving the NPC:

```bash
ros2 param get /lidar_obstacle_node self_mask_max_x
ros2 param get /lidar_obstacle_node self_mask_half_width_y
ros2 param get /lidar_obstacle_node near_origin_radius_m
```

Expect `2.0`, `0.8`, `0.3`. Keep Ego stationary with `enable_drive=false`; move the
single NPC through **14 -> 10 -> 7 -> 5 -> 3 m**, pausing at each location. At every
location verify obstacle_count=1, 8 Planning poses, 1 debug center, finite centroid
range matching the observed sensor distance, and no premature RECOVERING due to
lost detections. The initial centroid range may be about 14.095 m; staged NPC
placement distances and LiDAR centroid distances need not be numerically identical.
With the NPC removed, verify Ego returns do not produce an obstacle cluster.

Use actual odometry to check footprint world positions. In RViz use Fixed Frame
`map` and separate PoseArray displays for footprint and center topics. This task
performs no MORAI motion or stationary measurement itself.

Unarmed Planning must retain target speed 0 and normally READY; Lattice can still
report AVOIDING or BLOCKED. All candidates blocked keeps the existing emergency
arbitration. Check lateral road space and that perception does not drop an unpassed
NPC. Only after passing stationary checks should a separate, controlled 3–5 km/h
run be considered, with 10 km/h reserved for a later validated repeat.

The parameter rationale, failed-run evidence, curvature analysis, and detailed
stationary/low-speed checklist are in
`../edgenode_planning/docs/s4_obstacle_footprint.md`.

## Tests

```bash
cd /workspace/ros2_ws
python3 -m pytest -q src/edgenode_perception/test src/edgenode_planning/test \
  src/edgenode_control/test src/edgenode_localization/test
colcon build --symlink-install
git -C /workspace diff --check
```

Coverage includes filtering/clustering regressions, footprint bounds, yaw=0/90
world transforms, cluster counts vs footprint counts, preserved range semantics,
PointCloud2 layouts, real isolated BEST_EFFORT transport, Planning callback/Lattice
integration, candidate clearance, recovery, and unchanged FSM motion gating.
Tests do not start a live driving stack or perform MORAI validation.
