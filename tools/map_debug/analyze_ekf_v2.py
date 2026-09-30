import csv
import math
import os

import numpy as np
import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


BAG_DIR = "/workspace/local_data/ekf_validation_v2"
OUTPUT_DIR = "/workspace/local_data/ekf_analysis_v2"


def get_time_sec(msg, bag_timestamp_ns):
    """Prefer the original ROS header timestamp when available."""
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        stamp = msg.header.stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if t > 0.0:
            return t

    return float(bag_timestamp_ns) * 1e-9


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(
        siny_cosp,
        cosy_cosp,
    )


def wrap_angle(angle):
    return (
        angle + math.pi
    ) % (
        2.0 * math.pi
    ) - math.pi


def unique_sorted(t, *arrays):
    order = np.argsort(t)

    t = np.asarray(t, dtype=np.float64)[order]
    arrays = [
        np.asarray(a, dtype=np.float64)[order]
        for a in arrays
    ]

    unique_t, unique_idx = np.unique(
        t,
        return_index=True,
    )

    unique_arrays = [
        a[unique_idx]
        for a in arrays
    ]

    return (unique_t, *unique_arrays)


def print_metrics(name, values, unit):
    values = np.asarray(values)

    print(name)
    print(
        f"  Mean          : "
        f"{np.mean(values):.6f} {unit}"
    )
    print(
        f"  Median        : "
        f"{np.median(values):.6f} {unit}"
    )
    print(
        f"  95%           : "
        f"{np.percentile(values, 95):.6f} {unit}"
    )
    print(
        f"  Max           : "
        f"{np.max(values):.6f} {unit}"
    )


def main():
    os.makedirs(
        OUTPUT_DIR,
        exist_ok=True,
    )

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

    required = [
        "/localization/odometry",
        "/ego_vehicle_status",
    ]

    for topic in required:
        if topic not in topic_types:
            raise RuntimeError(
                f"Required topic missing: {topic}"
            )

    odom_type = get_message(
        topic_types["/localization/odometry"]
    )

    ego_type = get_message(
        topic_types["/ego_vehicle_status"]
    )

    ekf_t = []
    ekf_x = []
    ekf_y = []
    ekf_yaw = []
    ekf_v = []

    ego_t = []
    ego_x = []
    ego_y = []
    ego_yaw = []

    while reader.has_next():
        topic, data, bag_timestamp = reader.read_next()

        if topic == "/localization/odometry":
            msg = deserialize_message(
                data,
                odom_type,
            )

            t = get_time_sec(
                msg,
                bag_timestamp,
            )

            p = msg.pose.pose.position
            q = msg.pose.pose.orientation

            yaw = quaternion_to_yaw(
                float(q.x),
                float(q.y),
                float(q.z),
                float(q.w),
            )

            ekf_t.append(t)
            ekf_x.append(float(p.x))
            ekf_y.append(float(p.y))
            ekf_yaw.append(yaw)
            ekf_v.append(
                float(msg.twist.twist.linear.x)
            )

        elif topic == "/ego_vehicle_status":
            msg = deserialize_message(
                data,
                ego_type,
            )

            t = get_time_sec(
                msg,
                bag_timestamp,
            )

            ego_t.append(t)
            ego_x.append(
                float(msg.position.x)
            )
            ego_y.append(
                float(msg.position.y)
            )

            # MORAI heading was already verified against IMU yaw.
            ego_yaw.append(
                math.radians(
                    float(msg.heading)
                )
            )

    (
        ekf_t,
        ekf_x,
        ekf_y,
        ekf_yaw,
        ekf_v,
    ) = unique_sorted(
        ekf_t,
        ekf_x,
        ekf_y,
        ekf_yaw,
        ekf_v,
    )

    (
        ego_t,
        ego_x,
        ego_y,
        ego_yaw,
    ) = unique_sorted(
        ego_t,
        ego_x,
        ego_y,
        ego_yaw,
    )

    print(
        f"EKF samples : {len(ekf_t)}"
    )
    print(
        f"Ego samples : {len(ego_t)}"
    )

    if len(ekf_t) < 2 or len(ego_t) < 3:
        raise RuntimeError(
            "Not enough samples for analysis"
        )

    # ------------------------------------------------------------
    # Only compare the time interval where both datasets exist.
    # ------------------------------------------------------------

    start_t = max(
        ekf_t[0],
        ego_t[0],
    )

    end_t = min(
        ekf_t[-1],
        ego_t[-1],
    )

    valid = (
        (ekf_t >= start_t)
        &
        (ekf_t <= end_t)
    )

    ekf_t = ekf_t[valid]
    ekf_x = ekf_x[valid]
    ekf_y = ekf_y[valid]
    ekf_yaw = ekf_yaw[valid]
    ekf_v = ekf_v[valid]

    # ------------------------------------------------------------
    # Ego ground truth interpolation
    # ------------------------------------------------------------

    ego_x_i = np.interp(
        ekf_t,
        ego_t,
        ego_x,
    )

    ego_y_i = np.interp(
        ekf_t,
        ego_t,
        ego_y,
    )

    # ±pi discontinuity must be removed before interpolation.
    ego_yaw_unwrapped = np.unwrap(
        ego_yaw
    )

    ego_yaw_i = np.interp(
        ekf_t,
        ego_t,
        ego_yaw_unwrapped,
    )

    # ------------------------------------------------------------
    # Ground-truth speed from Ego x/y derivative.
    #
    # This deliberately avoids assuming the unit of
    # EgoVehicleStatus.velocity.
    # ------------------------------------------------------------

    ego_vx = np.gradient(
        ego_x,
        ego_t,
    )

    ego_vy = np.gradient(
        ego_y,
        ego_t,
    )

    ego_speed = np.hypot(
        ego_vx,
        ego_vy,
    )

    ego_speed_i = np.interp(
        ekf_t,
        ego_t,
        ego_speed,
    )

    # ------------------------------------------------------------
    # Errors
    # ------------------------------------------------------------

    dx = ekf_x - ego_x_i
    dy = ekf_y - ego_y_i

    position_error = np.hypot(
        dx,
        dy,
    )

    yaw_error = np.array(
        [
            wrap_angle(
                float(a - b)
            )
            for a, b in zip(
                ekf_yaw,
                ego_yaw_i,
            )
        ]
    )

    yaw_abs_error_deg = np.degrees(
        np.abs(yaw_error)
    )

    velocity_error = (
        ekf_v - ego_speed_i
    )

    velocity_abs_error = np.abs(
        velocity_error
    )

    elapsed = (
        ekf_t - ekf_t[0]
    )

    print()
    print(
        "========== EKF VALIDATION V2 =========="
    )

    print(
        f"Compared samples : {len(ekf_t)}"
    )

    print(
        f"Duration         : "
        f"{ekf_t[-1] - ekf_t[0]:.3f} s"
    )

    print()
    print_metrics(
        "POSITION ERROR",
        position_error,
        "m",
    )

    print()
    print_metrics(
        "YAW ABS ERROR",
        yaw_abs_error_deg,
        "deg",
    )

    print()
    print_metrics(
        "VELOCITY ABS ERROR",
        velocity_abs_error,
        "m/s",
    )

    print()
    print(
        "VELOCITY RANGE"
    )
    print(
        f"  EKF min/max    : "
        f"{np.min(ekf_v):.3f} / "
        f"{np.max(ekf_v):.3f} m/s"
    )
    print(
        f"  Ego min/max    : "
        f"{np.min(ego_speed_i):.3f} / "
        f"{np.max(ego_speed_i):.3f} m/s"
    )

    # ------------------------------------------------------------
    # Save aligned numeric data
    # ------------------------------------------------------------

    csv_path = os.path.join(
        OUTPUT_DIR,
        "aligned_metrics.csv",
    )

    with open(
        csv_path,
        "w",
        newline="",
    ) as f:
        writer = csv.writer(f)

        writer.writerow(
            [
                "time_sec",
                "ekf_x",
                "ekf_y",
                "ego_x",
                "ego_y",
                "position_error_m",
                "ekf_yaw_deg",
                "ego_yaw_deg",
                "yaw_error_deg",
                "ekf_v_mps",
                "ego_v_mps",
                "velocity_error_mps",
            ]
        )

        for i in range(len(ekf_t)):
            writer.writerow(
                [
                    elapsed[i],
                    ekf_x[i],
                    ekf_y[i],
                    ego_x_i[i],
                    ego_y_i[i],
                    position_error[i],
                    math.degrees(
                        ekf_yaw[i]
                    ),
                    math.degrees(
                        wrap_angle(
                            ego_yaw_i[i]
                        )
                    ),
                    math.degrees(
                        yaw_error[i]
                    ),
                    ekf_v[i],
                    ego_speed_i[i],
                    velocity_error[i],
                ]
            )

    # ------------------------------------------------------------
    # Save text summary
    # ------------------------------------------------------------

    summary_path = os.path.join(
        OUTPUT_DIR,
        "summary.txt",
    )

    with open(
        summary_path,
        "w",
    ) as f:
        f.write(
            "EKF VALIDATION V2\n"
        )

        f.write(
            f"Compared samples: {len(ekf_t)}\n"
        )

        f.write(
            f"Duration: "
            f"{ekf_t[-1] - ekf_t[0]:.3f} s\n\n"
        )

        f.write(
            "POSITION ERROR [m]\n"
        )
        f.write(
            f"mean={np.mean(position_error):.6f}\n"
        )
        f.write(
            f"median={np.median(position_error):.6f}\n"
        )
        f.write(
            f"p95={np.percentile(position_error, 95):.6f}\n"
        )
        f.write(
            f"max={np.max(position_error):.6f}\n\n"
        )

        f.write(
            "YAW ABS ERROR [deg]\n"
        )
        f.write(
            f"mean={np.mean(yaw_abs_error_deg):.6f}\n"
        )
        f.write(
            f"median={np.median(yaw_abs_error_deg):.6f}\n"
        )
        f.write(
            f"p95={np.percentile(yaw_abs_error_deg, 95):.6f}\n"
        )
        f.write(
            f"max={np.max(yaw_abs_error_deg):.6f}\n\n"
        )

        f.write(
            "VELOCITY ABS ERROR [m/s]\n"
        )
        f.write(
            f"mean={np.mean(velocity_abs_error):.6f}\n"
        )
        f.write(
            f"median={np.median(velocity_abs_error):.6f}\n"
        )
        f.write(
            f"p95={np.percentile(velocity_abs_error, 95):.6f}\n"
        )
        f.write(
            f"max={np.max(velocity_abs_error):.6f}\n"
        )

    # ------------------------------------------------------------
    # Graphs
    # ------------------------------------------------------------

    if plt is not None:
        plt.figure()
        plt.plot(
            ego_x_i,
            ego_y_i,
            label="Ego ground truth",
        )
        plt.plot(
            ekf_x,
            ekf_y,
            label="EKF",
        )
        plt.xlabel("MGeo X [m]")
        plt.ylabel("MGeo Y [m]")
        plt.title("EKF vs Ego trajectory")
        plt.axis("equal")
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            os.path.join(
                OUTPUT_DIR,
                "01_xy_trajectory.png",
            ),
            dpi=150,
        )
        plt.close()

        plt.figure()
        plt.plot(
            elapsed,
            position_error,
        )
        plt.xlabel("Time [s]")
        plt.ylabel("Position error [m]")
        plt.title("EKF position error")
        plt.tight_layout()
        plt.savefig(
            os.path.join(
                OUTPUT_DIR,
                "02_position_error.png",
            ),
            dpi=150,
        )
        plt.close()

        plt.figure()
        plt.plot(
            elapsed,
            np.degrees(
                np.unwrap(ekf_yaw)
            ),
            label="EKF",
        )
        plt.plot(
            elapsed,
            np.degrees(
                ego_yaw_i
            ),
            label="Ego ground truth",
        )
        plt.xlabel("Time [s]")
        plt.ylabel("Yaw [deg, unwrapped]")
        plt.title("EKF yaw vs Ego heading")
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            os.path.join(
                OUTPUT_DIR,
                "03_yaw_comparison.png",
            ),
            dpi=150,
        )
        plt.close()

        plt.figure()
        plt.plot(
            elapsed,
            ekf_v,
            label="EKF v",
        )
        plt.plot(
            elapsed,
            ego_speed_i,
            label="Ego position-derived speed",
        )
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [m/s]")
        plt.title("EKF velocity vs Ego ground truth")
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            os.path.join(
                OUTPUT_DIR,
                "04_velocity_comparison.png",
            ),
            dpi=150,
        )
        plt.close()

    print()
    print(
        f"Saved analysis to:"
    )
    print(
        f"  {OUTPUT_DIR}"
    )

    print(
        f"  {csv_path}"
    )

    print(
        f"  {summary_path}"
    )

    if plt is None:
        print()
        print(
            "matplotlib is not installed, "
            "so graphs were skipped."
        )


if __name__ == "__main__":
    main()
