import json
import os

import numpy as np
import pyproj
import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG_DIR = "/workspace/rosbags/c_track_full"
MGEO_GLOBAL_INFO = (
    "/workspace/local_data/c_track_mgeo/global_info.json"
)


def load_mgeo_origin():
    with open(MGEO_GLOBAL_INFO, "r", encoding="utf-8") as f:
        info = json.load(f)

    origin = info["local_origin_in_global"]

    return (
        float(origin[0]),
        float(origin[1]),
        float(origin[2]),
    )


def read_rosbag():
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

    gps_type = get_message(topic_types["/gps"])
    ego_type = get_message(
        topic_types["/ego_vehicle_status"]
    )

    gps_samples = []
    ego_samples = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == "/gps":
            msg = deserialize_message(
                data,
                gps_type,
            )

            gps_samples.append({
                "t": timestamp / 1e9,
                "latitude": float(msg.latitude),
                "longitude": float(msg.longitude),
                "east_offset": float(msg.east_offset),
                "north_offset": float(msg.north_offset),
            })

        elif topic == "/ego_vehicle_status":
            msg = deserialize_message(
                data,
                ego_type,
            )

            ego_samples.append({
                "t": timestamp / 1e9,
                "x": float(msg.position.x),
                "y": float(msg.position.y),
            })

    return gps_samples, ego_samples


def main():
    origin_e, origin_n, _ = load_mgeo_origin()

    gps_samples, ego_samples = read_rosbag()

    print(f"GPS samples : {len(gps_samples)}")
    print(f"Ego samples : {len(ego_samples)}")
    print()

    print("MGeo origin:")
    print(f"  Easting  = {origin_e:.10f}")
    print(f"  Northing = {origin_n:.10f}")

    if gps_samples:
        print()
        print("First GPS offsets:")
        print(
            f"  east_offset  = "
            f"{gps_samples[0]['east_offset']:.10f}"
        )
        print(
            f"  north_offset = "
            f"{gps_samples[0]['north_offset']:.10f}"
        )

        print()
        print("Origin - GPS offset difference:")
        print(
            f"  east  = "
            f"{origin_e - gps_samples[0]['east_offset']:.10f} m"
        )
        print(
            f"  north = "
            f"{origin_n - gps_samples[0]['north_offset']:.10f} m"
        )

    # WGS84 latitude/longitude
    # -> UTM Zone 52N
    transformer = pyproj.Transformer.from_crs(
        "EPSG:4326",
        "EPSG:32652",
        always_xy=True,
    )

    ego_t = np.array(
        [s["t"] for s in ego_samples],
        dtype=np.float64,
    )
    ego_x = np.array(
        [s["x"] for s in ego_samples],
        dtype=np.float64,
    )
    ego_y = np.array(
        [s["y"] for s in ego_samples],
        dtype=np.float64,
    )

    errors = []
    dxs = []
    dys = []

    valid_count = 0

    print()
    print(
        f"{'idx':>5} "
        f"{'gps_x':>10} "
        f"{'gps_y':>10} "
        f"{'ego_x':>10} "
        f"{'ego_y':>10} "
        f"{'error[m]':>10}"
    )
    print("-" * 62)

    for i, gps in enumerate(gps_samples):
        t = gps["t"]

        if t < ego_t[0] or t > ego_t[-1]:
            continue

        easting, northing = transformer.transform(
            gps["longitude"],
            gps["latitude"],
        )

        local_x = easting - origin_e
        local_y = northing - origin_n

        # EgoVehicleStatus를 GPS timestamp 위치로 보간
        ref_x = np.interp(
            t,
            ego_t,
            ego_x,
        )
        ref_y = np.interp(
            t,
            ego_t,
            ego_y,
        )

        dx = local_x - ref_x
        dy = local_y - ref_y

        error = float(
            np.hypot(dx, dy)
        )

        dxs.append(dx)
        dys.append(dy)
        errors.append(error)

        if valid_count % 50 == 0:
            print(
                f"{i:5d} "
                f"{local_x:10.3f} "
                f"{local_y:10.3f} "
                f"{ref_x:10.3f} "
                f"{ref_y:10.3f} "
                f"{error:10.3f}"
            )

        valid_count += 1

    errors = np.array(errors)
    dxs = np.array(dxs)
    dys = np.array(dys)

    print()
    print("========== SUMMARY ==========")
    print(f"Compared samples : {len(errors)}")

    if len(errors):
        print(
            f"Mean dx          : "
            f"{np.mean(dxs):.3f} m"
        )
        print(
            f"Mean dy          : "
            f"{np.mean(dys):.3f} m"
        )
        print(
            f"Mean error       : "
            f"{np.mean(errors):.3f} m"
        )
        print(
            f"Median error     : "
            f"{np.median(errors):.3f} m"
        )
        print(
            f"95% error        : "
            f"{np.percentile(errors, 95):.3f} m"
        )
        print(
            f"Max error        : "
            f"{np.max(errors):.3f} m"
        )


if __name__ == "__main__":
    main()
