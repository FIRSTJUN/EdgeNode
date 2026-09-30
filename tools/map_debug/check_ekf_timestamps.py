import collections

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG_DIR = "/workspace/local_data/ekf_validation_v1"
TOPIC = "/localization/odometry"


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

topic_types = {
    t.name: t.type
    for t in reader.get_all_topics_and_types()
}

msg_type = get_message(topic_types[TOPIC])

header_stamps = []
bag_stamps = []

while reader.has_next():
    topic, data, bag_timestamp = reader.read_next()

    if topic != TOPIC:
        continue

    msg = deserialize_message(data, msg_type)

    stamp_ns = (
        int(msg.header.stamp.sec) * 1_000_000_000
        + int(msg.header.stamp.nanosec)
    )

    header_stamps.append(stamp_ns)
    bag_stamps.append(int(bag_timestamp))


counter = collections.Counter(header_stamps)

duplicate_groups = [
    (stamp, count)
    for stamp, count in counter.items()
    if count > 1
]

duplicate_messages = sum(
    count - 1
    for _, count in duplicate_groups
)

print("===== EKF TIMESTAMP CHECK =====")
print(f"Total messages            : {len(header_stamps)}")
print(f"Unique header timestamps  : {len(set(header_stamps))}")
print(f"Duplicate extra messages  : {duplicate_messages}")
print(f"Duplicate groups          : {len(duplicate_groups)}")
print(f"Unique bag timestamps     : {len(set(bag_stamps))}")

if duplicate_groups:
    print()
    print("First duplicate groups:")
    for stamp, count in duplicate_groups[:20]:
        print(f"  {stamp} : {count} messages")
