# S4 sensor validation

The normal baseline keeps `s4_sensor_validation_enabled: false`. Its existing
obstacle PoseArray, normal path/speed policy, Dijkstra, Pure Pursuit and PID are
unchanged. The S4 overlay `config/s4_sensor_validation.yaml` enables a startup-only
sensor gate and explicitly keeps `enable_drive: false`. Runtime disabling of the
gate or changing its safety timing parameters is rejected by ROS parameters.

## Wire contract

`/planning/obstacles` remains `geometry_msgs/PoseArray`, frame `map`, cloud stamp,
eight footprint points per cluster. Existing subscribers remain compatible.
On input/processing failure the legacy producer still emits an empty array;
an S4 consumer must never use that array without its matching success status.

`/perception/lidar_status` is `std_msgs/String`, reliable/volatile/depth 10, with
a JSON object (`schema_version: 1`). It is published after each cloud and at
0.1-second steady-clock heartbeat intervals, even before the first cloud.

|Field|Meaning|
|---|---|
|clock|`ros`; source, localization and status timestamps must share this clock|
|session_id|Random 32-hex ID per detector process; changes on restart|
|status_seq|Strictly increasing per published snapshot, including heartbeat|
|frame_seq|Increments per cloud callback, including failed or duplicate input|
|fault_seq|Cumulative failure events; persists after recovery so a missed error snapshot cannot hide a failure|
|state|Processing result or watchdog reason below|
|source_stamp_ns|Unmodified cloud acquisition stamp as integer nanoseconds|
|frame_received_ns|ROS receipt time of that cloud, never renewed by heartbeat|
|published_ns|ROS time of this snapshot|
|cluster_count|Successful cluster count; does not alone indicate validity|
|obstacle_digest|SHA256 of compact JSON `[frame_id, stamp_ns, [[float(x), float(y)], ...]]`; present for successful frames|

Success states are `READY_EMPTY` and `READY_OBSTACLES`. Failure/readiness states
are `WAITING_FOR_INPUT`, `INVALID_INPUT`, `PROCESSING_ERROR`,
`LOCALIZATION_UNAVAILABLE`, `INPUT_TIMEOUT`, `STALE_INPUT`, `INVALID_STAMP`,
`FUTURE_STAMP`, `DUPLICATE_FRAME`, `OUT_OF_ORDER`, `CLOCK_REGRESSION`, and
`PROCESSING` (not ready). Normal empty input must still be a correctly laid-out
PointCloud2 with XYZ fields and a valid acquisition stamp. Missing fields,
inconsistent empty dimensions, truncated data and entirely nonfinite returns
are invalid; legitimate filtered/no-cluster data is a successful empty result.

Temporal validation has deterministic precedence: clock regression, missing
stamp, old source, future source, duplicate/reordered stamp, then processing
outcome. Consequently one input with multiple defects reports the first gate.

## Timing and pairing

Default input/status/paired-frame timeout and maximum source age are 0.6 s;
future tolerance is 0.05 s. These are explicit initial verification limits, not
measured MORAI performance claims. ROI and clustering thresholds are unchanged.
Source age is checked against ROS time independently of receipt freshness.
Localization used for health must also have a fresh ordered source stamp.
Planning independently checks its own map pose stamp, ordering and unit
quaternion in this mode, so a fresh detector stream cannot bless an older pose.

There is **no automatic clock offset estimation**. A MORAI clock with a different
epoch, a zero stamp or mismatched `use_sim_time` fails closed. Verify the clock
contract before interpreting a stream as ready. The dedicated launch applies
the same `use_sim_time` argument to both nodes; external odometry must agree.
ROS clock rewind requires restart of the affected source/Planning node, not a
reset that erases timestamp ordering. Steady-clock heartbeat and Planning's
50-ms sensor watchdog also detect loss while ROS time is paused. Actual braking
still depends on the existing Control process/timer/transport and is not proven
by these checks.

Planning pairs success status with the exact PoseArray stamp, digest and point
count. Either DDS arrival order is supported. It retains at most 16 pending
arrays and can keep a still-fresh previous complete pair while the next success
pair is in transit; it never uses an unpaired new array. A heartbeat or replayed
array cannot renew source/frame age. Failure status overrides the previous good
pair immediately. Unknown/malformed protocol, stale status, replay, retired
session, pairing failure or a changed frame identity cannot authorize driving.

## STOP and recovery

The S4 gate is checked before route work, before local path publication and
again before the final motion gate. The status callback and independent steady
watchdog publish an empty local path and target speed zero on failure and latch
the existing StaticObstaclePlanner BLOCKED state. They do not invoke Control or
alter its PP/PID. `/planning/s4_sensor_state` (`std_msgs/String`) reports enabled,
ready, reason, session ID and last paired source stamp. Existing avoidance/FSM
topics retain the latched safety reason and emergency arbitration.

Startup without paired data also latches STOP. A newly healthy stream restores
sensor readiness only. It never clears BLOCKED, obstacle memory or a committed
maneuver. Detector restart invalidates the old pair and requests STOP; retired
sessions cannot regain authority. After fresh data is paired, an explicit reset
is accepted only while `enable_drive=false`. Existing non-sensor BLOCKED reasons
are preserved, and the next normal obstacle/boundary validation still applies.
An increasing fault counter in an otherwise successful snapshot also requests
STOP, covering failures that recover before an intermediate status is delivered.

## Launch and restart

`autonomy.launch.py` remains the normal baseline. The dedicated validation
launch starts only LiDAR and Planning; it starts neither Localization nor
Control. Supply map odometry separately. Stop prior instances of these two
nodes in their own terminals before restarting; do not run duplicate publishers.

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select edgenode_perception edgenode_planning edgenode_bringup --symlink-install
source install/setup.bash
# Isolated stationary validation; no MORAI/Control in this domain.
export ROS_DOMAIN_ID=182
export ROS_LOCALHOST_ONLY=1
ros2 launch edgenode_bringup s4_validation.launch.py use_sim_time:=false
```

In another terminal with the same environment, inspect status and explicitly
reset only after `ready: true` and verifying drive is disabled:

```bash
ros2 param get /planning_node enable_drive
ros2 param get /planning_node s4_sensor_validation_enabled
ros2 topic echo /perception/lidar_status --once
ros2 topic echo /planning/s4_sensor_state --once
ros2 service call /planning/reset_avoidance std_srvs/srv/Trigger '{}'
```

The isolated domain needs its own synthetic/recorded, clock-compatible inputs.
This launch is not MORAI driving approval. Verified road boundaries, usable
sensing/entry distances, full obstacle extent, control latency and braking
remain separate requirements.
