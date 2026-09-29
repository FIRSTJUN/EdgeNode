import csv
import json
import math
import os

import numpy as np
import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


MGEO_DIR = "/workspace/local_data/c_track_mgeo"
BAG_DIR = "/workspace/rosbags/c_track_full"

OUTPUT_CSV = os.path.join(
    MGEO_DIR,
    "map_match_v02.csv",
)

# ---- v0.2 parameters ----
CHECK_EVERY_N_EGO_MSG = 10      # Ego ≈ 50 Hz -> map matching ≈ 5 Hz
SEARCH_RADIUS_M = 6.0
MAX_MATCH_DISTANCE_M = 4.0
MAX_HEADING_DIFF_DEG = 80.0

HEADING_WEIGHT = 0.025          # deg -> score penalty
SAME_LINK_BONUS = 0.4
CONNECTED_LINK_BONUS = 0.7
UNRELATED_LINK_PENALTY = 1.0


def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def angle_difference_deg(a, b):
    return abs(normalize_angle_deg(a - b))


def point_to_segment(px, py, ax, ay, bx, by):
    dx = bx - ax
    dy = by - ay

    length_sq = dx * dx + dy * dy

    if length_sq <= 1e-12:
        distance = math.hypot(px - ax, py - ay)
        heading = 0.0
        return distance, heading

    t = (
        (px - ax) * dx +
        (py - ay) * dy
    ) / length_sq

    t = max(0.0, min(1.0, t))

    nearest_x = ax + t * dx
    nearest_y = ay + t * dy

    distance = math.hypot(
        px - nearest_x,
        py - nearest_y,
    )

    # ENU:
    # +X = East, +Y = North
    # atan2(dy, dx) -> East = 0 deg, CCW positive
    heading = math.degrees(
        math.atan2(dy, dx)
    )

    return distance, heading


def closest_point_on_link(px, py, link):
    points = link["points"]

    best_distance = float("inf")
    best_heading = 0.0

    for i in range(len(points) - 1):
        ax, ay = points[i][0], points[i][1]
        bx, by = points[i + 1][0], points[i + 1][1]

        distance, heading = point_to_segment(
            px, py,
            ax, ay,
            bx, by,
        )

        if distance < best_distance:
            best_distance = distance
            best_heading = heading

    return best_distance, best_heading


def prepare_links():
    path = os.path.join(
        MGEO_DIR,
        "link_set.json",
    )

    with open(path, "r", encoding="utf-8") as f:
        raw_links = json.load(f)

    links = []
    links_by_id = {}
    outgoing = {}

    for link in raw_links:
        points = link.get("points", [])

        if len(points) < 2:
            continue

        xs = [float(p[0]) for p in points]
        ys = [float(p[1]) for p in points]

        link["_bbox"] = (
            min(xs),
            max(xs),
            min(ys),
            max(ys),
        )

        links.append(link)
        links_by_id[link["idx"]] = link

        from_node = link.get("from_node_idx")

        outgoing.setdefault(
            from_node,
            set(),
        ).add(link["idx"])

    return links, links_by_id, outgoing


def bbox_candidate(px, py, bbox):
    min_x, max_x, min_y, max_y = bbox

    return (
        min_x - SEARCH_RADIUS_M <= px <= max_x + SEARCH_RADIUS_M
        and
        min_y - SEARCH_RADIUS_M <= py <= max_y + SEARCH_RADIUS_M
    )


def get_connected_links(
    previous_link_id,
    links_by_id,
    outgoing,
):
    if previous_link_id is None:
        return set()

    previous = links_by_id.get(
        previous_link_id
    )

    if previous is None:
        return set()

    connected = {previous_link_id}

    to_node = previous.get("to_node_idx")

    connected.update(
        outgoing.get(to_node, set())
    )

    left = previous.get(
        "left_lane_change_dst_link_idx"
    )

    right = previous.get(
        "right_lane_change_dst_link_idx"
    )

    if left:
        connected.add(left)

    if right:
        connected.add(right)

    return connected


def find_best_link(
    px,
    py,
    vehicle_heading,
    links,
    links_by_id,
    outgoing,
    previous_link_id,
):
    connected = get_connected_links(
        previous_link_id,
        links_by_id,
        outgoing,
    )

    best = None

    for link in links:
        if not bbox_candidate(
            px,
            py,
            link["_bbox"],
        ):
            continue

        distance, link_heading = \
            closest_point_on_link(
                px,
                py,
                link,
            )

        if distance > MAX_MATCH_DISTANCE_M:
            continue

        heading_diff = angle_difference_deg(
            vehicle_heading,
            link_heading,
        )

        if heading_diff > MAX_HEADING_DIFF_DEG:
            continue

        score = (
            distance +
            HEADING_WEIGHT * heading_diff
        )

        link_id = link["idx"]

        if previous_link_id is not None:
            if link_id == previous_link_id:
                score -= SAME_LINK_BONUS

            elif link_id in connected:
                score -= CONNECTED_LINK_BONUS

            else:
                score += UNRELATED_LINK_PENALTY

        if best is None or score < best["score"]:
            best = {
                "link_id": link_id,
                "distance": distance,
                "link_heading": link_heading,
                "heading_diff": heading_diff,
                "score": score,
            }

    return best


def main():
    links, links_by_id, outgoing = \
        prepare_links()

    print(
        f"Loaded {len(links)} MGeo links"
    )

    storage_options = \
        rosbag2_py.StorageOptions(
            uri=BAG_DIR,
            storage_id="sqlite3",
        )

    converter_options = \
        rosbag2_py.ConverterOptions(
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

    ego_topic = "/ego_vehicle_status"
    ego_type = get_message(
        topic_types[ego_topic]
    )

    ego_count = 0
    check_count = 0
    matched_count = 0
    unmatched_count = 0
    link_change_count = 0

    previous_link_id = None

    distances = []
    heading_diffs = []

    rows = []

    print()
    print(
        f"{'sample':>7} "
        f"{'x':>9} "
        f"{'y':>9} "
        f"{'heading':>8} "
        f"{'link_id':>15} "
        f"{'dist':>7} "
        f"{'hdiff':>7}"
    )

    print("-" * 78)

    while reader.has_next():
        topic, data, timestamp = \
            reader.read_next()

        if topic != ego_topic:
            continue

        ego_count += 1

        if (
            ego_count %
            CHECK_EVERY_N_EGO_MSG
            != 0
        ):
            continue

        msg = deserialize_message(
            data,
            ego_type,
        )

        x = float(msg.position.x)
        y = float(msg.position.y)
        heading = float(msg.heading)

        result = find_best_link(
            x,
            y,
            heading,
            links,
            links_by_id,
            outgoing,
            previous_link_id,
        )

        check_count += 1

        if result is None:
            unmatched_count += 1

            rows.append([
                timestamp,
                ego_count,
                x,
                y,
                heading,
                "UNMATCHED",
                "",
                "",
                "",
            ])

            # UNMATCHED가 나왔다고 바로
            # previous link를 버리지는 않는다.
            if check_count % 25 == 0:
                print(
                    f"{ego_count:7d} "
                    f"{x:9.2f} "
                    f"{y:9.2f} "
                    f"{heading:8.2f} "
                    f"{'UNMATCHED':>15}"
                )

            continue

        link_id = result["link_id"]

        matched_count += 1
        distances.append(
            result["distance"]
        )
        heading_diffs.append(
            result["heading_diff"]
        )

        changed = (
            previous_link_id is not None
            and
            link_id != previous_link_id
        )

        if changed:
            link_change_count += 1

        rows.append([
            timestamp,
            ego_count,
            x,
            y,
            heading,
            link_id,
            result["distance"],
            result["heading_diff"],
            result["score"],
        ])

        # 변화가 있을 때 + 약 5초마다 출력
        if changed or check_count % 25 == 0:
            marker = (
                " <-- LINK CHANGE"
                if changed
                else ""
            )

            print(
                f"{ego_count:7d} "
                f"{x:9.2f} "
                f"{y:9.2f} "
                f"{heading:8.2f} "
                f"{link_id:>15} "
                f"{result['distance']:7.3f} "
                f"{result['heading_diff']:7.2f}"
                f"{marker}"
            )

        previous_link_id = link_id

    with open(
        OUTPUT_CSV,
        "w",
        newline="",
        encoding="utf-8",
    ) as f:
        writer = csv.writer(f)

        writer.writerow([
            "timestamp_ns",
            "ego_sample",
            "x",
            "y",
            "vehicle_heading_deg",
            "matched_link",
            "distance_m",
            "heading_diff_deg",
            "score",
        ])

        writer.writerows(rows)

    print()
    print("========== SUMMARY ==========")
    print(f"Checked       : {check_count}")
    print(f"Matched       : {matched_count}")
    print(f"Unmatched     : {unmatched_count}")
    print(f"Link changes  : {link_change_count}")

    if distances:
        print(
            "Distance median:",
            f"{np.median(distances):.3f} m"
        )
        print(
            "Distance p95   :",
            f"{np.percentile(distances, 95):.3f} m"
        )
        print(
            "Distance max   :",
            f"{np.max(distances):.3f} m"
        )

    if heading_diffs:
        print(
            "Heading median :",
            f"{np.median(heading_diffs):.2f} deg"
        )
        print(
            "Heading p95    :",
            f"{np.percentile(heading_diffs, 95):.2f} deg"
        )

    print(
        "CSV saved      :",
        OUTPUT_CSV
    )


if __name__ == "__main__":
    main()
