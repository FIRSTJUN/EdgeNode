import math

import numpy as np
import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG_DIR = "/workspace/rosbags/c_track_full"


def normalize_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def quaternion_to_yaw_deg(x, y, z, w):
    # Standard ROS quaternion -> yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.degrees(
        math.atan2(siny_cosp, cosy_cosp)
    )


def circular_error_deg(a, b):
    return normalize_deg(a - b)


def main():
    storage_options = rosbag2_py.StorageOptions(
        uri=BAG_DIR,
        storage_id="sqlite3",
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(
        storage_options,
        converter_options,
    )

    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }

    imu_type = get_message(topic_types["/Imu"])
    ego_type = get_message(
        topic_types["/ego_vehicle_status"]
    )

    imu_samples = []
    ego_samples = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        t = timestamp / 1e9

        if topic == "/Imu":
            msg = deserialize_message(
                data,
                imu_type,
            )

            q = msg.orientation

            yaw = quaternion_to_yaw_deg(
                float(q.x),
                float(q.y),
                float(q.z),
                float(q.w),
            )

            imu_samples.append({
                "t": t,
                "yaw": yaw,
                "gyro_z": float(
                    msg.angular_velocity.z
                ),
                "accel_x": float(
                    msg.linear_acceleration.x
                ),
            })

        elif topic == "/ego_vehicle_status":
            msg = deserialize_message(
                data,
                ego_type,
            )

            ego_samples.append({
                "t": t,
                "heading": float(msg.heading),
            })

    print(f"IMU samples : {len(imu_samples)}")
    print(f"Ego samples : {len(ego_samples)}")

    ego_t = np.array(
        [s["t"] for s in ego_samples],
        dtype=np.float64,
    )

    # heading에는 ±180 wrap이 있으므로
    # 그냥 np.interp하지 않고 nearest timestamp를 사용한다.
    comparisons = []

    for imu in imu_samples:
        t = imu["t"]

        idx = int(
            np.searchsorted(ego_t, t)
        )

        candidates = []

        if idx < len(ego_samples):
            candidates.append(idx)

        if idx > 0:
            candidates.append(idx - 1)

        if not candidates:
            continue

        best_idx = min(
            candidates,
            key=lambda i: abs(
                ego_samples[i]["t"] - t
            ),
        )

        ego = ego_samples[best_idx]

        comparisons.append({
            "t": t,
            "imu_yaw": imu["yaw"],
            "ego_heading": ego["heading"],
            "gyro_z": imu["gyro_z"],
        })

    # 여러 좌표계 관계를 모두 시험한다.
    transforms = {
        "imu_yaw": lambda y: y,
        "-imu_yaw": lambda y: -y,
        "imu_yaw + 90": lambda y: normalize_deg(y + 90.0),
        "imu_yaw - 90": lambda y: normalize_deg(y - 90.0),
        "-imu_yaw + 90": lambda y: normalize_deg(-y + 90.0),
        "-imu_yaw - 90": lambda y: normalize_deg(-y - 90.0),
        "imu_yaw + 180": lambda y: normalize_deg(y + 180.0),
    }

    print()
    print("===== CANDIDATE RELATIONSHIPS =====")

    results = []

    for name, fn in transforms.items():
        errors = []

        for c in comparisons:
            converted = fn(
                c["imu_yaw"]
            )

            error = abs(
                circular_error_deg(
                    converted,
                    c["ego_heading"],
                )
            )

            errors.append(error)

        errors = np.array(errors)

        result = (
            name,
            float(np.median(errors)),
            float(np.percentile(errors, 95)),
            float(np.mean(errors)),
        )

        results.append(result)

    results.sort(key=lambda x: x[1])

    for name, median, p95, mean in results:
        print(
            f"{name:18s} "
            f"median={median:7.3f} deg  "
            f"p95={p95:7.3f} deg  "
            f"mean={mean:7.3f} deg"
        )

    best_name = results[0][0]
    best_fn = transforms[best_name]

    print()
    print(
        f"Best candidate: {best_name}"
    )

    print()
    print(
        f"{'idx':>6} "
        f"{'imu_yaw':>10} "
        f"{'converted':>10} "
        f"{'ego_head':>10} "
        f"{'error':>9} "
        f"{'gyro_z':>9}"
    )
    print("-" * 62)

    for i in range(
        0,
        len(comparisons),
        max(1, len(comparisons) // 10),
    ):
        c = comparisons[i]

        converted = best_fn(
            c["imu_yaw"]
        )

        error = circular_error_deg(
            converted,
            c["ego_heading"],
        )

        print(
            f"{i:6d} "
            f"{c['imu_yaw']:10.3f} "
            f"{converted:10.3f} "
            f"{c['ego_heading']:10.3f} "
            f"{error:9.3f} "
            f"{c['gyro_z']:9.4f}"
        )


if __name__ == "__main__":
    main()
