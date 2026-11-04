#!/usr/bin/env python3
"""
Extract PCD files from a ROS 2 PointCloud2 rosbag (mcap or sqlite3).
"""

import argparse
import sys
from pathlib import Path
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def parse_args():
    parser = argparse.ArgumentParser(description="Extract PCD files from ROS 2 PointCloud2 bag")
    parser.add_argument(
        "--bag-dir",
        required=True,
        help="Path to input rosbag2 directory (e.g. /mnt/z/.../rosbag2_...)",
    )
    parser.add_argument(
        "--output-dir",
        default=str(Path.cwd() / "pcd"),
        help="Output directory to save .pcd files (default: ./pcd)",
    )
    parser.add_argument(
        "--topic",
        default="",
        help="PointCloud2 topic name (default: auto-detect topic with highest message count)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=15,
        help="Number of PCD frames to extract (default: 15)",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=1,
        help="Frame interval/step between extracted frames (default: 1)",
    )
    return parser.parse_args()


def save_pcd_binary(path: Path, points: np.ndarray):
    points = points.astype(np.float32)
    header = (
        "# .PCD v0.7 - Point Cloud Data file\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {points.shape[0]}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {points.shape[0]}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(points.tobytes())


def main():
    args = parse_args()
    bag_path = Path(args.bag_dir).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()

    if not bag_path.exists():
        print(f"[ERROR] Bag directory not found: {bag_path}")
        print("Tip: If the bag is on Windows Z: drive, check if /mnt/z is mounted:")
        print("     sudo mkdir -p /mnt/z && sudo mount -t drvfs Z: /mnt/z")
        sys.exit(1)

    output_dir.mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=""),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )
    except Exception:
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
            )
        except Exception as e:
            print(f"[ERROR] Failed to open rosbag '{bag_path}': {e}")
            sys.exit(1)

    topics_and_types = reader.get_all_topics_and_types()
    msg_counts = {}
    try:
        meta = reader.get_metadata()
        for t in meta.topics_with_message_count:
            msg_counts[t.topic_metadata.name] = t.message_count
    except Exception:
        pass

    pc2_topics = [t.name for t in topics_and_types if t.type == "sensor_msgs/msg/PointCloud2"]

    if not pc2_topics:
        available = [f"  - {t.name} ({t.type})" for t in topics_and_types]
        print(f"[ERROR] No PointCloud2 topics found in bag. Available topics:\n" + "\n".join(available))
        sys.exit(1)

    print("\n[INFO] Found PointCloud2 topics in bag:")
    for t_name in pc2_topics:
        cnt_str = f"{msg_counts.get(t_name, 'unknown')} msgs"
        print(f"  * {t_name:<40} ({cnt_str})")

    target_topic = args.topic
    if not target_topic:
        valid_topics = [t for t in pc2_topics if msg_counts.get(t, 1) > 0]
        if not valid_topics:
            valid_topics = pc2_topics

        non_debug = [t for t in valid_topics if "debug" not in t.lower()]
        candidates = non_debug if non_debug else valid_topics
        candidates.sort(key=lambda t: msg_counts.get(t, 0), reverse=True)
        target_topic = candidates[0]
        print(f"\n[INFO] Auto-selected main PointCloud2 topic: '{target_topic}'")
    else:
        matched = [t for t in pc2_topics if t.lstrip('/') == target_topic.lstrip('/')]
        if not matched:
            print(f"[ERROR] Specified topic '{target_topic}' not found in bag.")
            sys.exit(1)
        target_topic = matched[0]
        print(f"\n[INFO] Using specified topic: '{target_topic}'")

    print(f"[INFO] Extracting up to {args.count} frames (step: {args.step}) to: {output_dir}\n")

    saved_count = 0
    raw_frame_idx = 0

    while reader.has_next() and saved_count < args.count:
        topic_name, data, timestamp = reader.read_next()
        if topic_name.lstrip('/') != target_topic.lstrip('/'):
            continue

        raw_frame_idx += 1
        if (raw_frame_idx - 1) % args.step != 0:
            continue

        msg = deserialize_message(data, PointCloud2)
        dtype = point_cloud2.dtype_from_fields(msg.fields, msg.point_step)
        cloud = np.ndarray(shape=(msg.width * msg.height,), dtype=dtype, buffer=msg.data)
        xyz = np.column_stack([cloud["x"], cloud["y"], cloud["z"]]).astype(np.float32)

        valid_mask = np.isfinite(xyz).all(axis=1)
        xyz_valid = xyz[valid_mask]

        if xyz_valid.shape[0] < 100:
            print(f"[WARN] Frame {raw_frame_idx} has too few valid points ({xyz_valid.shape[0]}), skipping...")
            continue

        saved_count += 1
        out_filename = output_dir / f"frame_{saved_count:04d}.pcd"
        save_pcd_binary(out_filename, xyz_valid)
        print(f"[{saved_count:02d}/{args.count:02d}] Saved {out_filename.name} (points: {xyz_valid.shape[0]:,})")

    if saved_count == 0:
        print(f"\n[WARN] 0 frames were extracted from '{target_topic}'.")
        print("Try specifying another topic explicitly using --topic <TOPIC_NAME>")
    else:
        print(f"\n[SUCCESS] Successfully extracted {saved_count} PCD files to: {output_dir}")


if __name__ == "__main__":
    main()
