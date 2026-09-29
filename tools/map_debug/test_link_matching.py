import json
import math
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


MGEO_DIR = "/workspace/local_data/c_track_mgeo"
BAG_DIR = "/workspace/rosbags/c_track_full"


def point_to_segment_distance(px, py, ax, ay, bx, by):
    dx = bx - ax
    dy = by - ay

    length_sq = dx * dx + dy * dy

    if length_sq == 0.0:
        return math.hypot(px - ax, py - ay)

    t = (
        (px - ax) * dx +
        (py - ay) * dy
    ) / length_sq

    t = max(0.0, min(1.0, t))

    nearest_x = ax + t * dx
    nearest_y = ay + t * dy

    return math.hypot(
        px - nearest_x,
        py - nearest_y,
    )


def distance_to_link(px, py, link):
    points = link["points"]

    min_distance = float("inf")

    for i in range(len(points) - 1):
        ax, ay = points[i][0], points[i][1]
        bx, by = points[i + 1][0], points[i + 1][1]

        distance = point_to_segment_distance(
            px,
            py,
            ax,
            ay,
            bx,
            by,
        )

        if distance < min_distance:
            min_distance = distance

    return min_distance


def find_nearest_link(px, py, links):
    best_link = None
    best_distance = float("inf")

    for link in links:
        if len(link.get("points", [])) < 2:
            continue

        distance = distance_to_link(
            px,
            py,
            link,
        )

        if distance < best_distance:
            best_distance = distance
            best_link = link

    return best_link, best_distance


def load_links():
    path = os.path.join(
        MGEO_DIR,
        "link_set.json",
    )

    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def main():
    links = load_links()

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

    ego_topic = "/ego_vehicle_status"
    ego_type = get_message(topic_types[ego_topic])

    sample_count = 0
    checked_count = 0
    previous_link_id = None

    print(f"Loaded {len(links)} MGeo links")
    print()
    print(
        f"{'sample':>7} "
        f"{'x':>10} "
        f"{'y':>10} "
        f"{'link_id':>15} "
        f"{'distance[m]':>12}"
    )
    print("-" * 62)

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic != ego_topic:
            continue

        sample_count += 1

        # EgoVehicleStatus ≈ 50 Hz
        # 약 1초마다 한 번 검사
        if sample_count % 50 != 0:
            continue

        msg = deserialize_message(
            data,
            ego_type,
        )

        x = float(msg.position.x)
        y = float(msg.position.y)

        link, distance = find_nearest_link(
            x,
            y,
            links,
        )

        if link is None:
            continue

        link_id = link["idx"]

        marker = ""
        if link_id != previous_link_id:
            marker = "  <-- LINK CHANGE"

        print(
            f"{sample_count:7d} "
            f"{x:10.2f} "
            f"{y:10.2f} "
            f"{link_id:>15} "
            f"{distance:12.3f}"
            f"{marker}"
        )

        previous_link_id = link_id
        checked_count += 1

    print()
    print(f"Checked samples: {checked_count}")


if __name__ == "__main__":
    main()
