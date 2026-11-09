# MORAI school-zone YOLO

`school_zone_yolo_test.py` and `school_zone_yolo_bridge.py` are unchanged copies
of the working scripts in `/workspace/local_data/`. The originals are preserved.
The test publishes visualization only; the archived bridge uses 2FPS and a
3-of-5 condition **without** the 0.80 confidence requirement.

Use the new `school_zone_yolo_node` for the requested entry condition:
5 inference frames, at least 3 detections (confidence >= 0.50), and at least one
confidence >= 0.80. Inference starts are limited to 5FPS using monotonic time.
It publishes one event and rearms after 10 consecutive missing inference frames.
The weights must be the custom, single-class school-zone YOLO26 Nano model;
architecture/training provenance cannot be established from the scripts alone.

## Run

Use the existing ROS 2 Humble + CUDA environment. Python must be able to import
`rclpy`, `sensor_msgs`, `std_msgs`, `cv2`, `numpy`, `ultralytics`, and CUDA-capable
PyTorch. Ultralytics/PyTorch are optional YOLO dependencies, not dependencies of
the existing perception nodes. The local environment is
`/workspace/local_data/yolo_venv`; keep its installed versions intact.
Verified local versions: Ultralytics 8.4.208, PyTorch 2.9.1+cu128, NumPy 1.26.4,
OpenCV 4.11.0.86.
Weights remain local at `/workspace/local_data/models/school_zone/best.pt`.

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source /workspace/local_data/yolo_venv/bin/activate
```

For direct execution without rebuilding or changing the running installation:

```bash
python /workspace/ros2_ws/src/edgenode_perception/edgenode_perception/school_zone_yolo_node.py --ros-args -p model_path:=/workspace/local_data/models/school_zone/best.pt -p device:=0
```

On the next normal package build (`cd /workspace/ros2_ws && colcon build
--packages-select edgenode_perception`), source `install/setup.bash`; the new
entry point is `ros2 run edgenode_perception school_zone_yolo_node`.
Use `-p device:=cpu` for CPU inference.

The input is `/camera/image/compressed` (`sensor_msgs/CompressedImage`, sensor
QoS). Outputs are `/perception/school_zone/sign_detected` (`std_msgs/Bool`, True
event) and `/perception/school_zone/yolo_debug` (`sensor_msgs/Image`, bgr8,
sensor QoS). View debug images with a sensor-QoS-compatible image viewer.
Run only one YOLO publisher at a time; the archive bridge and new node publish
the same topics. Do not run the visualization script through pytest.

To run the archived scripts, use the same Python environment:
`python /workspace/tools/school_zone_yolo/school_zone_yolo_test.py` or
`python /workspace/tools/school_zone_yolo/school_zone_yolo_bridge.py`.
Both retain their original absolute weights path and GPU device 0.

Existing road classifier, transition detector, manager, Planning, Control and
Route Editor interfaces are unchanged. The manager's existing
`sim_auto_confirm_on_gray:=true` enables MORAI automatic exit confirmation.
The speed adapter already supports a 30km/h cap and verified baseline restore
with `allow_confirmed_restore:=true`; moving simulation restore additionally
requires `sim_allow_moving_restore:=true` and obeys `max_sim_restore_kmh`
(default 60). These existing options remain opt-in.
