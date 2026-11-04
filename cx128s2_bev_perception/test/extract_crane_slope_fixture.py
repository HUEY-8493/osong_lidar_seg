#!/usr/bin/env python3
"""Read an existing CX128S2 bag without creating a ROS node or replaying topics.

Usage (source ROS 2 first):
  python3 extract_crane_slope_fixture.py BAG EMPTY_OUTPUT_DIR --frames 30

Fixtures retain original 32-byte sensor records. Only the observed driver layout
is accepted; this is an offline test format, not a second production cloud parser.
The current test_crane_pipeline executable runs synthetic/oracle tests only;
it does not consume these fixtures or measure recorded-data performance.
"""
import argparse
import json
import math
from pathlib import Path
import struct

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("output", type=Path)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--topic", default="/ch/lslidar_point_cloud")
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be positive")
    args.output.mkdir(parents=True, exist_ok=True)
    if any(args.output.iterdir()):
        parser.error("output directory must be empty")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    profiles = []
    expected = {"x": (0, 7), "y": (4, 7), "z": (8, 7),
                "intensity": (20, 7), "ring": (24, 4), "time": (28, 7)}
    while reader.has_next() and len(profiles) < args.frames:
        topic, data, _ = reader.read_next()
        if topic != args.topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        fields = {f.name: (f.offset, f.datatype) for f in msg.fields if f.count == 1}
        if (msg.is_bigendian or msg.height != 1 or msg.point_step != 32 or
                msg.row_step != msg.width * 32 or len(msg.data) != msg.row_step or
                any(fields.get(name) != value for name, value in expected.items())):
            raise ValueError("Unexpected driver layout; no fixture written for this frame")
        bounds = {}
        az_lo, az_hi = math.inf, -math.inf
        for offset in range(0, len(msg.data), 32):
            x, y, z = struct.unpack_from("<fff", msg.data, offset)
            if not all(map(math.isfinite, (x, y, z))) or x*x+y*y+z*z < 1e-12:
                continue
            ring, = struct.unpack_from("<H", msg.data, offset+24)
            az = math.degrees(math.atan2(y, x))
            el = math.degrees(math.atan2(z, math.hypot(x, y)))
            az_lo, az_hi = min(az_lo, az), max(az_hi, az)
            lo, hi = bounds.get(ring, (el, el))
            bounds[ring] = (min(lo, el), max(hi, el))
        number = len(profiles)+1
        (args.output / f"{number:03}.bin").write_bytes(msg.data)
        profiles.append({"stamp_sec": msg.header.stamp.sec,
                         "stamp_nanosec": msg.header.stamp.nanosec,
                         "frame": msg.header.frame_id, "points": msg.width,
                         "azimuth_deg": [az_lo, az_hi],
                         "ring_elevation_bounds_deg": bounds})
    if not profiles:
        raise ValueError("No matching point clouds in bag")
    (args.output / "profile.json").write_text(json.dumps(profiles, indent=2)+"\n")
    print(f"Extracted {len(profiles)} frames; first frame has "
          f"{profiles[0]['points']} points, "
          f"{len(profiles[0]['ring_elevation_bounds_deg'])} rings")


if __name__ == "__main__":
    main()
