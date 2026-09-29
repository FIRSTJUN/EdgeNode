import math

import numpy as np
import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG_DIR = "/workspace/rosbags/c_track_full"


def quaternion_to_yaw_rad(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(
        siny_cosp,
        cosy_cosp,
    )


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

    imu_type = get_message(
        topic_types["/Imu"]
    )

    times = []
    yaws = []
    gyro_zs = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic != "/Imu":
            continue

        msg = deserialize_message(
            data,
            imu_type,
        )

        q = msg.orientation

        yaw = quaternion_to_yaw_rad(
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )

        times.append(timestamp / 1e9)
        yaws.append(yaw)
        gyro_zs.append(
            float(msg.angular_velocity.z)
        )

    times = np.array(
        times,
        dtype=np.float64,
    )

    yaws = np.array(
        yaws,
        dtype=np.float64,
    )

    gyro_zs = np.array(
        gyro_zs,
        dtype=np.float64,
    )

    print(f"IMU samples: {len(times)}")

    if len(times) < 3:
        raise RuntimeError(
            "Not enough IMU samples"
        )

    # ±pi 경계 때문에 yaw를 연속적인 각도로 변환
    yaw_unwrapped = np.unwrap(yaws)

    # 각 timestamp 기준 중앙차분에 가까운 numerical derivative
    yaw_rate = np.gradient(
        yaw_unwrapped,
        times,
    )

    # 비정상 timestamp 간격 제거
    dt = np.diff(times)

    print()
    print("Timestamp interval:")
    print(
        f"  median dt = "
        f"{np.median(dt):.6f} s"
    )
    print(
        f"  median Hz = "
        f"{1.0 / np.median(dt):.2f} Hz"
    )

    valid = (
        np.isfinite(yaw_rate)
        & np.isfinite(gyro_zs)
    )

    rate = yaw_rate[valid]
    gyro = gyro_zs[valid]

    error = gyro - rate
    abs_error = np.abs(error)

    if len(rate) > 1:
        correlation = np.corrcoef(
            gyro,
            rate,
        )[0, 1]
    else:
        correlation = float("nan")

    same_sign_mask = (
        (np.abs(rate) > 0.05)
        & (np.abs(gyro) > 0.05)
    )

    if np.any(same_sign_mask):
        same_sign_ratio = np.mean(
            np.sign(rate[same_sign_mask])
            ==
            np.sign(gyro[same_sign_mask])
        )
    else:
        same_sign_ratio = float("nan")

    print()
    print("========== SUMMARY ==========")
    print(
        f"Compared samples : {len(rate)}"
    )
    print(
        f"Correlation      : "
        f"{correlation:.6f}"
    )
    print(
        f"Mean error       : "
        f"{np.mean(error):.6f} rad/s"
    )
    print(
        f"Median abs error : "
        f"{np.median(abs_error):.6f} rad/s"
    )
    print(
        f"95% abs error    : "
        f"{np.percentile(abs_error, 95):.6f} rad/s"
    )
    print(
        f"Max abs error    : "
        f"{np.max(abs_error):.6f} rad/s"
    )

    if np.isfinite(same_sign_ratio):
        print(
            f"Same sign ratio  : "
            f"{same_sign_ratio * 100.0:.2f}%"
        )

    print()
    print(
        f"{'idx':>6} "
        f"{'yaw[deg]':>10} "
        f"{'dyaw/dt':>11} "
        f"{'gyro_z':>11} "
        f"{'error':>11}"
    )
    print("-" * 58)

    step = max(
        1,
        len(times) // 12,
    )

    for i in range(
        0,
        len(times),
        step,
    ):
        print(
            f"{i:6d} "
            f"{math.degrees(yaws[i]):10.3f} "
            f"{yaw_rate[i]:11.4f} "
            f"{gyro_zs[i]:11.4f} "
            f"{gyro_zs[i] - yaw_rate[i]:11.4f}"
        )


if __name__ == "__main__":
    main()
