import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


MGEO_DIR = "/workspace/local_data/c_track_mgeo"
BAG_DIR = "/workspace/rosbags/c_track_full"

OUTPUT_PATH = os.path.join(
    MGEO_DIR,
    "c_track_mgeo_vs_ego.png",
)


def load_mgeo_links():
    link_path = os.path.join(
        MGEO_DIR,
        "link_set.json",
    )

    with open(link_path, "r", encoding="utf-8") as f:
        links = json.load(f)

    return links


def load_ego_trajectory():
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

    if ego_topic not in topic_types:
        raise RuntimeError(
            f"{ego_topic} not found in rosbag"
        )

    msg_type = get_message(
        topic_types[ego_topic]
    )

    xs = []
    ys = []

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic != ego_topic:
            continue

        msg = deserialize_message(
            data,
            msg_type,
        )

        xs.append(msg.position.x)
        ys.append(msg.position.y)

    return xs, ys


def main():
    links = load_mgeo_links()
    ego_x, ego_y = load_ego_trajectory()

    print(f"MGeo links: {len(links)}")
    print(f"Ego samples: {len(ego_x)}")

    plt.figure(figsize=(12, 10))

    first_link = True

    for link in links:
        points = link.get("points", [])

        if len(points) < 2:
            continue

        x = [p[0] for p in points]
        y = [p[1] for p in points]

        plt.plot(
            x,
            y,
            linewidth=1.0,
            alpha=0.7,
            label="MGeo links" if first_link else None,
        )

        first_link = False

    plt.plot(
        ego_x,
        ego_y,
        linewidth=2.0,
        label="Ego trajectory",
    )

    if ego_x:
        plt.scatter(
            [ego_x[0]],
            [ego_y[0]],
            s=60,
            label="Start",
        )

        plt.scatter(
            [ego_x[-1]],
            [ego_y[-1]],
            s=60,
            label="End",
        )

    plt.xlabel("MGeo local X [m]")
    plt.ylabel("MGeo local Y [m]")
    plt.title(
        "C-Track MGeo vs MORAI Ego Trajectory"
    )

    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        OUTPUT_PATH,
        dpi=180,
    )

    print(f"Saved: {OUTPUT_PATH}")

    if ego_x:
        print(
            "Ego X range:",
            min(ego_x),
            "~",
            max(ego_x),
        )
        print(
            "Ego Y range:",
            min(ego_y),
            "~",
            max(ego_y),
        )


if __name__ == "__main__":
    main()
