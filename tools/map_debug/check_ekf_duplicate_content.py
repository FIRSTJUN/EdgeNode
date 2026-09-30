import math
from collections import defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG_DIR = "/workspace/local_data/ekf_validation_v1"
TOPIC = "/localization/odometry"


def yaw_from_q(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


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
    x.name: x.type
    for x in reader.get_all_topics_and_types()
}

msg_type = get_message(types[TOPIC])

groups = defaultdict(list)

while reader.has_next():
    topic, data, bag_stamp = reader.read_next()

    if topic != TOPIC:
        continue

    msg = deserialize_message(data, msg_type)

    stamp = (
        int(msg.header.stamp.sec) * 1_000_000_000
        + int(msg.header.stamp.nanosec)
    )

    groups[stamp].append(
        (
            int(bag_stamp),
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_q(msg.pose.pose.orientation),
            float(msg.twist.twist.linear.x),
        )
    )


duplicate_groups = [
    values
    for values in groups.values()
    if len(values) > 1
]

dxs = []
dys = []
dposs = []
dyaws = []
dvs = []
dbag_ms = []

for values in duplicate_groups:
    a = values[0]
    b = values[1]

    dx = b[1] - a[1]
    dy = b[2] - a[2]

    dxs.append(abs(dx))
    dys.append(abs(dy))
    dposs.append(math.hypot(dx, dy))
    dyaws.append(abs(math.degrees(wrap(b[3] - a[3]))))
    dvs.append(abs(b[4] - a[4]))
    dbag_ms.append((b[0] - a[0]) / 1e6)


def stats(name, data, unit):
    data = sorted(data)

    if not data:
        return

    n = len(data)

    print(
        f"{name:<22}"
        f"median={data[n//2]:.8f} {unit}   "
        f"max={data[-1]:.8f} {unit}"
    )


print("===== DUPLICATE CONTENT CHECK =====")
print(f"Duplicate groups : {len(duplicate_groups)}")
print()

stats("Position delta", dposs, "m")
stats("Yaw delta", dyaws, "deg")
stats("Velocity delta", dvs, "m/s")
stats("Bag-time gap", dbag_ms, "ms")

print()
print("First 10 duplicate pairs:")
print(
    f"{'bag gap[ms]':>12} "
    f"{'pos Δ[m]':>12} "
    f"{'yaw Δ[deg]':>12} "
    f"{'v Δ[m/s]':>12}"
)
print("-" * 52)

for values in duplicate_groups[:10]:
    a = values[0]
    b = values[1]

    dp = math.hypot(
        b[1] - a[1],
        b[2] - a[2],
    )

    dyaw = abs(
        math.degrees(
            wrap(b[3] - a[3])
        )
    )

    dv = abs(
        b[4] - a[4]
    )

    gap = (
        b[0] - a[0]
    ) / 1e6

    print(
        f"{gap:12.3f} "
        f"{dp:12.6f} "
        f"{dyaw:12.6f} "
        f"{dv:12.6f}"
    )
