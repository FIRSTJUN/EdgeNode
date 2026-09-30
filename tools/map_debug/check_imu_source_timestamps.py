from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG_DIR = "/workspace/rosbags/c_track_full"
TOPIC = "/Imu"

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

msg_type = get_message(types[TOPIC])

stamps = []

while reader.has_next():
    topic, data, _ = reader.read_next()

    if topic != TOPIC:
        continue

    msg = deserialize_message(data, msg_type)

    stamp_ns = (
        int(msg.header.stamp.sec) * 1_000_000_000
        + int(msg.header.stamp.nanosec)
    )

    stamps.append(stamp_ns)

counter = Counter(stamps)

duplicate_groups = [
    count
    for count in counter.values()
    if count > 1
]

duplicate_extra = sum(
    count - 1
    for count in duplicate_groups
)

print("===== SOURCE IMU TIMESTAMP CHECK =====")
print(f"Total IMU messages       : {len(stamps)}")
print(f"Unique header timestamps : {len(set(stamps))}")
print(f"Duplicate extra messages : {duplicate_extra}")
print(f"Duplicate groups         : {len(duplicate_groups)}")
