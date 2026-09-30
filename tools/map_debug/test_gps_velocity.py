import math

import numpy as np
import rosbag2_py

from pyproj import Transformer
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG_DIR = "/workspace/rosbags/c_track_full"

TRANSFORMER = Transformer.from_crs(
    "EPSG:4326",
    "EPSG:32652",
    always_xy=True,
)


def msg_time(msg, bag_stamp):
    if hasattr(msg, "header"):
        sec = int(msg.header.stamp.sec)
        nsec = int(msg.header.stamp.nanosec)

        t = sec + nsec * 1e-9

        if t > 0:
            return t

    return bag_stamp * 1e-9


def quat_to_yaw(q):
    siny_cosp = 2.0 * (
        q.w * q.z + q.x * q.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        q.y * q.y + q.z * q.z
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp,
    )


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

gps_type = get_message(types["/gps"])
imu_type = get_message(types["/Imu"])
ego_type = get_message(types["/ego_vehicle_status"])

gps_t = []
gps_x = []
gps_y = []

imu_t = []
imu_yaw = []

ego_t = []
ego_x = []
ego_y = []

while reader.has_next():
    topic, data, bag_stamp = reader.read_next()

    if topic == "/gps":
        msg = deserialize_message(
            data,
            gps_type,
        )

        easting, northing = TRANSFORMER.transform(
            float(msg.longitude),
            float(msg.latitude),
        )

        x = (
            easting
            - float(msg.east_offset)
        )

        y = (
            northing
            - float(msg.north_offset)
        )

        gps_t.append(
            msg_time(msg, bag_stamp)
        )
        gps_x.append(x)
        gps_y.append(y)

    elif topic == "/Imu":
        msg = deserialize_message(
            data,
            imu_type,
        )

        imu_t.append(
            msg_time(msg, bag_stamp)
        )

        imu_yaw.append(
            quat_to_yaw(msg.orientation)
        )

    elif topic == "/ego_vehicle_status":
        msg = deserialize_message(
            data,
            ego_type,
        )

        ego_t.append(
            msg_time(msg, bag_stamp)
        )

        ego_x.append(
            float(msg.position.x)
        )

        ego_y.append(
            float(msg.position.y)
        )


gps_t = np.asarray(gps_t)
gps_x = np.asarray(gps_x)
gps_y = np.asarray(gps_y)

imu_t = np.asarray(imu_t)
imu_yaw = np.unwrap(
    np.asarray(imu_yaw)
)

ego_t = np.asarray(ego_t)
ego_x = np.asarray(ego_x)
ego_y = np.asarray(ego_y)


raw_speed = []
forward_speed = []
truth_speed = []
sample_dt = []

for k in range(1, len(gps_t)):
    t0 = gps_t[k - 1]
    t1 = gps_t[k]

    dt = t1 - t0

    if (
        dt <= 0.05
        or dt > 1.0
    ):
        continue

    dx = gps_x[k] - gps_x[k - 1]
    dy = gps_y[k] - gps_y[k - 1]

    v_mag = math.hypot(
        dx,
        dy,
    ) / dt

    # Latest IMU yaw at or before current GPS time.
    idx = np.searchsorted(
        imu_t,
        t1,
        side="right",
    ) - 1

    if idx < 0:
        continue

    yaw = imu_yaw[idx]

    v_forward = (
        dx * math.cos(yaw)
        +
        dy * math.sin(yaw)
    ) / dt

    # Ground truth displacement over exactly
    # the same GPS interval.
    ego_x0 = np.interp(
        t0,
        ego_t,
        ego_x,
    )
    ego_y0 = np.interp(
        t0,
        ego_t,
        ego_y,
    )

    ego_x1 = np.interp(
        t1,
        ego_t,
        ego_x,
    )
    ego_y1 = np.interp(
        t1,
        ego_t,
        ego_y,
    )

    v_truth = math.hypot(
        ego_x1 - ego_x0,
        ego_y1 - ego_y0,
    ) / dt

    raw_speed.append(v_mag)
    forward_speed.append(v_forward)
    truth_speed.append(v_truth)
    sample_dt.append(dt)


raw_speed = np.asarray(raw_speed)
forward_speed = np.asarray(forward_speed)
truth_speed = np.asarray(truth_speed)
sample_dt = np.asarray(sample_dt)

raw_error = np.abs(
    raw_speed - truth_speed
)

forward_error = np.abs(
    forward_speed - truth_speed
)


def report(name, speed, error):
    corr = np.corrcoef(
        speed,
        truth_speed,
    )[0, 1]

    print()
    print(name)
    print(
        f"  Correlation     : {corr:.6f}"
    )
    print(
        f"  Mean abs error  : "
        f"{np.mean(error):.6f} m/s"
    )
    print(
        f"  Median abs error: "
        f"{np.median(error):.6f} m/s"
    )
    print(
        f"  95% abs error   : "
        f"{np.percentile(error, 95):.6f} m/s"
    )
    print(
        f"  Max abs error   : "
        f"{np.max(error):.6f} m/s"
    )
    print(
        f"  Min / max speed : "
        f"{np.min(speed):.3f} / "
        f"{np.max(speed):.3f} m/s"
    )


print("===== GPS VELOCITY VALIDATION =====")
print(
    f"Compared intervals : {len(truth_speed)}"
)
print(
    f"Median GPS dt       : "
    f"{np.median(sample_dt):.4f} s"
)
print(
    f"Median GPS rate     : "
    f"{1.0 / np.median(sample_dt):.2f} Hz"
)

report(
    "GPS DISTANCE SPEED",
    raw_speed,
    raw_error,
)

report(
    "GPS FORWARD SPEED",
    forward_speed,
    forward_error,
)

print()
print("GROUND TRUTH")
print(
    f"  Min / max speed : "
    f"{np.min(truth_speed):.3f} / "
    f"{np.max(truth_speed):.3f} m/s"
)

print()
print(
    "Negative forward-speed samples : "
    f"{np.sum(forward_speed < -0.05)}"
)
