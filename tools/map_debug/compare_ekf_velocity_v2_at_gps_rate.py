import math
from collections import defaultdict

import numpy as np
import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG_DIR = "/workspace/local_data/ekf_validation_v2"


def header_time(msg, bag_stamp):
    if hasattr(msg, "header"):
        t = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        if t > 0.0:
            return t

    return float(bag_stamp) * 1e-9


reader = rosbag2_py.SequentialReader()

reader.open(
    rosbag2_py.StorageOptions(
        uri=BAG_DIR,
        storage_id="sqlite3",
    ),
    rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    ),
)

types = {
    t.name: t.type
    for t in reader.get_all_topics_and_types()
}

ekf_type = get_message(
    types["/localization/odometry"]
)

gps_type = get_message(
    types["/localization/gps_odometry"]
)

ego_type = get_message(
    types["/ego_vehicle_status"]
)

# EKF duplicate header timestamps:
# keep the last-published state for each sensor timestamp.
ekf_by_stamp = {}

gps_t = []
gps_x = []
gps_y = []

ego_t = []
ego_x = []
ego_y = []

while reader.has_next():
    topic, data, bag_stamp = reader.read_next()

    if topic == "/localization/odometry":
        msg = deserialize_message(
            data,
            ekf_type,
        )

        t = header_time(
            msg,
            bag_stamp,
        )

        ekf_by_stamp[t] = (
            float(msg.twist.twist.linear.x),
            int(bag_stamp),
        )

    elif topic == "/localization/gps_odometry":
        msg = deserialize_message(
            data,
            gps_type,
        )

        gps_t.append(
            header_time(msg, bag_stamp)
        )
        gps_x.append(
            float(msg.pose.pose.position.x)
        )
        gps_y.append(
            float(msg.pose.pose.position.y)
        )

    elif topic == "/ego_vehicle_status":
        msg = deserialize_message(
            data,
            ego_type,
        )

        ego_t.append(
            header_time(msg, bag_stamp)
        )
        ego_x.append(
            float(msg.position.x)
        )
        ego_y.append(
            float(msg.position.y)
        )


# Sort EKF
ekf_t = np.asarray(
    sorted(ekf_by_stamp.keys()),
    dtype=np.float64,
)

ekf_v = np.asarray(
    [
        ekf_by_stamp[t][0]
        for t in ekf_t
    ],
    dtype=np.float64,
)

# Sort GPS
gps_order = np.argsort(gps_t)

gps_t = np.asarray(
    gps_t,
    dtype=np.float64,
)[gps_order]

gps_x = np.asarray(
    gps_x,
    dtype=np.float64,
)[gps_order]

gps_y = np.asarray(
    gps_y,
    dtype=np.float64,
)[gps_order]

# Sort Ego
ego_order = np.argsort(ego_t)

ego_t = np.asarray(
    ego_t,
    dtype=np.float64,
)[ego_order]

ego_x = np.asarray(
    ego_x,
    dtype=np.float64,
)[ego_order]

ego_y = np.asarray(
    ego_y,
    dtype=np.float64,
)[ego_order]


gps_speed = []
ego_speed = []
ekf_speed = []
dts = []

for k in range(1, len(gps_t)):
    t0 = gps_t[k - 1]
    t1 = gps_t[k]

    dt = t1 - t0

    if dt <= 0.05 or dt > 1.0:
        continue

    # GPS distance speed
    gps_dx = (
        gps_x[k] - gps_x[k - 1]
    )

    gps_dy = (
        gps_y[k] - gps_y[k - 1]
    )

    v_gps = (
        math.hypot(gps_dx, gps_dy)
        / dt
    )

    # Ego displacement over exactly same interval
    ex0 = np.interp(
        t0,
        ego_t,
        ego_x,
    )

    ey0 = np.interp(
        t0,
        ego_t,
        ego_y,
    )

    ex1 = np.interp(
        t1,
        ego_t,
        ego_x,
    )

    ey1 = np.interp(
        t1,
        ego_t,
        ego_y,
    )

    v_ego = (
        math.hypot(
            ex1 - ex0,
            ey1 - ey0,
        )
        / dt
    )

    # EKF velocity at current GPS timestamp
    v_ekf = np.interp(
        t1,
        ekf_t,
        ekf_v,
    )

    gps_speed.append(v_gps)
    ego_speed.append(v_ego)
    ekf_speed.append(v_ekf)
    dts.append(dt)


gps_speed = np.asarray(gps_speed)
ego_speed = np.asarray(ego_speed)
ekf_speed = np.asarray(ekf_speed)

gps_error = np.abs(
    gps_speed - ego_speed
)

ekf_error = np.abs(
    ekf_speed - ego_speed
)


def report(name, speed, error):
    print()
    print(name)

    print(
        f"  Correlation      : "
        f"{np.corrcoef(speed, ego_speed)[0,1]:.6f}"
    )

    print(
        f"  Mean abs error   : "
        f"{np.mean(error):.6f} m/s"
    )

    print(
        f"  Median abs error : "
        f"{np.median(error):.6f} m/s"
    )

    print(
        f"  95% abs error    : "
        f"{np.percentile(error,95):.6f} m/s"
    )

    print(
        f"  Max abs error    : "
        f"{np.max(error):.6f} m/s"
    )

    print(
        f"  Min / max        : "
        f"{np.min(speed):.3f} / "
        f"{np.max(speed):.3f} m/s"
    )


print(
    "===== FAIR VELOCITY COMPARISON ====="
)

print(
    f"Compared intervals : "
    f"{len(ego_speed)}"
)

print(
    f"Median interval dt  : "
    f"{np.median(dts):.4f} s"
)

report(
    "CURRENT EKF V",
    ekf_speed,
    ekf_error,
)

report(
    "GPS DISTANCE V",
    gps_speed,
    gps_error,
)

print()
print("EGO INTERVAL GROUND TRUTH")

print(
    f"  Min / max        : "
    f"{np.min(ego_speed):.3f} / "
    f"{np.max(ego_speed):.3f} m/s"
)
