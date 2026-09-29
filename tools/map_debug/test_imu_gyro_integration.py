import math

import numpy as np
import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG_DIR = "/workspace/rosbags/c_track_full"

# 약 50 Hz IMU 기준 10 sample ≈ 0.2 sec
WINDOW = 10


def quaternion_to_yaw(x, y, z, w):
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
        t.name: t.type
        for t in reader.get_all_topics_and_types()
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

        yaw = quaternion_to_yaw(
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

    times = np.asarray(times)
    yaws = np.unwrap(
        np.asarray(yaws)
    )
    gyro_zs = np.asarray(
        gyro_zs
    )

    actual_deltas = []
    predicted_deltas = []

    for i in range(
        0,
        len(times) - WINDOW,
    ):
        j = i + WINDOW

        window_t = times[i:j + 1]
        window_gyro = gyro_zs[i:j + 1]

        # timestamp 이상치가 있는 구간은 제외
        dt = np.diff(window_t)

        if np.any(dt <= 0.0):
            continue

        if np.any(dt > 0.1):
            continue

        actual_delta = (
            yaws[j] - yaws[i]
        )

        predicted_delta = np.trapz(
            window_gyro,
            window_t,
        )

        if not (
            np.isfinite(actual_delta)
            and
            np.isfinite(predicted_delta)
        ):
            continue

        actual_deltas.append(
            actual_delta
        )

        predicted_deltas.append(
            predicted_delta
        )

    actual = np.asarray(
        actual_deltas
    )

    predicted = np.asarray(
        predicted_deltas
    )

    error = predicted - actual

    abs_error_deg = np.degrees(
        np.abs(error)
    )

    correlation = np.corrcoef(
        actual,
        predicted,
    )[0, 1]

    # 거의 회전하지 않는 구간은 부호 비교에서 제외
    moving = (
        (np.abs(actual) > math.radians(1.0))
        |
        (np.abs(predicted) > math.radians(1.0))
    )

    if np.any(moving):
        same_sign = np.mean(
            np.sign(actual[moving])
            ==
            np.sign(predicted[moving])
        )
    else:
        same_sign = float("nan")

    print(
        f"IMU samples      : {len(times)}"
    )
    print(
        f"Compared windows : {len(actual)}"
    )

    print()
    print(
        "========== SUMMARY =========="
    )

    print(
        f"Correlation      : "
        f"{correlation:.6f}"
    )

    print(
        f"Mean error       : "
        f"{np.degrees(np.mean(error)):.4f} deg"
    )

    print(
        f"Median abs error : "
        f"{np.median(abs_error_deg):.4f} deg"
    )

    print(
        f"95% abs error    : "
        f"{np.percentile(abs_error_deg, 95):.4f} deg"
    )

    print(
        f"Max abs error    : "
        f"{np.max(abs_error_deg):.4f} deg"
    )

    if np.isfinite(same_sign):
        print(
            f"Same sign ratio  : "
            f"{same_sign * 100.0:.2f}%"
        )

    print()
    print(
        f"{'idx':>6} "
        f"{'actual[deg]':>13} "
        f"{'gyro[deg]':>13} "
        f"{'error[deg]':>13}"
    )

    print("-" * 50)

    step = max(
        1,
        len(actual) // 10,
    )

    for i in range(
        0,
        len(actual),
        step,
    ):
        print(
            f"{i:6d} "
            f"{math.degrees(actual[i]):13.3f} "
            f"{math.degrees(predicted[i]):13.3f} "
            f"{math.degrees(error[i]):13.3f}"
        )


if __name__ == "__main__":
    main()
