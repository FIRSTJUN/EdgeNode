# Phase C source-time LiDAR tracking

Explicit opt-in: `ros2 launch edgenode_bringup s4_c_tracking.launch.py enable_drive:=false`.
Qualified source/profile, actual MORAI trials, limits and reproduction are in
[the Phase C report](../../../docs/s4_c_dynamic_tracking.md).
The node consumes LiDAR frames and source-time odometry, never Actor truth.
It publishes tracks only; the launch selects preserved A NORMAL-only safety.
Passenger dimensions are a declared observation prior, never verified collision dimensions.
