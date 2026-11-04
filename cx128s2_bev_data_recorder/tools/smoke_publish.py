#!/usr/bin/env python3
"""Synthetic production-interface sequence for event recorder checks (Cases A--G)."""
import struct
import time

import rclpy
from builtin_interfaces.msg import Time
from cx128s2_bev_perception.msg import (
    PerceptionRecordingFrame, RecordingObjectCluster, TrackedObject,
)
from sensor_msgs.msg import PointCloud2, PointField


def cloud(stamp, offset, frame_id="lidar"):
    msg = PointCloud2()
    msg.header.stamp, msg.header.frame_id = stamp, frame_id
    msg.height, msg.width = 1, 3
    msg.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z"))]
    msg.point_step, msg.row_step, msg.is_dense = 12, 36, True
    msg.data = b"".join(struct.pack("<fff", offset + x, y, z)
                        for x, y, z in ((1., 0., 0.), (1.2, .1, .2), (.8, -.1, .1)))
    return msg


def frame(stamp, observed):
    msg = PerceptionRecordingFrame()
    msg.header.stamp, msg.header.frame_id = stamp, "world"
    msg.transformed_cloud = cloud(stamp, 0.0, "world")
    msg.source_point_count = 3
    if observed:
        tracked = TrackedObject()
        tracked.id, tracked.class_name, tracked.lifecycle_state = 8, "VEHICLE", "TRACKED"
        tracked.position.x, tracked.dimensions.x = 5.0, 4.0
        cluster = RecordingObjectCluster()
        cluster.track_id, cluster.cluster_id, cluster.observed = 8, 1, True
        cluster.object, cluster.points = tracked, cloud(stamp, 5.0, "world")
        msg.objects = [cluster]
    return msg


def main():
    rclpy.init()
    node = rclpy.create_node("cx128s2_recorder_smoke_publisher")
    raw_pub = node.create_publisher(PointCloud2, "/recorder_smoke/cloud", 10)
    per_pub = node.create_publisher(PerceptionRecordingFrame, "/recorder_smoke/perception", 10)
    time.sleep(1.0)
    # A no detection; B trigger; C active; D one-frame miss/recovery;
    # E grace -> postbuffer -> finalization; F postbuffer recovery; G a new event.
    for second, observed in ((1000, False), (1001, False), (1002, True), (1003, True),
                             (1004, False), (1005, True), (1006, False), (1008, False),
                             (1009, True), (1010, False), (1012, False), (1017, False),
                             (1019, True)):
        stamp = Time(sec=second, nanosec=0)
        raw_pub.publish(cloud(stamp, float(second)))
        per_pub.publish(frame(stamp, observed))
        rclpy.spin_once(node, timeout_sec=.05)
        time.sleep(.15)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
