#!/usr/bin/env python3
"""Publish only compact perception health/state to Guardian Manager over HTTP.

This node intentionally never serializes PointCloud2 data.  The point-cloud
subscription exists solely to maintain last-receipt health information.
"""

import json
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2
from cx128s2_bev_perception.msg import CraneBeamStatus, TrackedObjectArray
from guardian_lidar_bridge.state_projection import FirstVehicleSelector, crane_summary, general_summary


VALID_MODES = {"crane", "general"}


class GuardianLidarBridge(Node):
    def __init__(self):
        super().__init__("guardian_lidar_bridge")
        self.mode = self.declare_parameter("mode", "crane").value
        if self.mode not in VALID_MODES:
            raise ValueError("mode must be 'crane' or 'general'")
        self.manager_url = self.declare_parameter(
            "manager_url", "http://127.0.0.1:8765/api/internal/bridge"
        ).value
        self.token = self.declare_parameter("bridge_token", "").value
        self.health_timeout_sec = float(self.declare_parameter("health_timeout_sec", 2.0).value)
        self.publish_period_sec = float(self.declare_parameter("publish_period_sec", 0.5).value)
        self.pointcloud_topic = self.declare_parameter(
            "pointcloud_topic", "/ch/lslidar_point_cloud"
        ).value
        self.last_pointcloud_rx = None
        self.last_perception_rx = None
        self.last_vehicle_rx = None
        self.sequence = 0
        self.perception = {}

        self.create_subscription(PointCloud2, self.pointcloud_topic, self._on_cloud, 10)
        if self.mode == "crane":
            self.create_subscription(CraneBeamStatus, "/crane_beam/status", self._on_crane, 10)
        else:
            self.detected = []
            self.tracked = []
            self.motion = []
            self.vehicles = []
            self.vehicle_selector = FirstVehicleSelector()
            self.vehicle_sensor_timestamp_ms = 0
            self.create_subscription(TrackedObjectArray, "/detected_objects", self._on_detected, 10)
            self.create_subscription(TrackedObjectArray, "/tracked_objects", self._on_tracked, 10)
            self.create_subscription(TrackedObjectArray, "/object_motion_states", self._on_motion, 10)
            self.create_subscription(TrackedObjectArray, "/road_vehicle_tracks", self._on_vehicles, 10)
        self.create_timer(max(self.publish_period_sec, 0.1), self._post_state)
        self.get_logger().info(f"Guardian bridge started: mode={self.mode}, manager={self.manager_url}")

    def _on_cloud(self, _message):
        self.last_pointcloud_rx = time.monotonic()

    def _on_crane(self, message):
        try:
            perception = crane_summary(message)
        except ValueError as exc:
            self.get_logger().warning(f"Ignoring invalid Crane status: {exc}", throttle_duration_sec=10.0)
            return
        self.last_perception_rx = time.monotonic()
        self.perception = perception

    def _on_detected(self, message):
        self.last_perception_rx = time.monotonic()
        self.detected = message.objects

    def _on_tracked(self, message):
        self.last_perception_rx = time.monotonic()
        self.tracked = message.objects

    def _on_motion(self, message):
        self.last_perception_rx = time.monotonic()
        self.motion = message.objects

    def _on_vehicles(self, message):
        self.last_perception_rx = time.monotonic()
        self.last_vehicle_rx = self.last_perception_rx
        self.vehicles = message.objects
        self.vehicle_selector.select(self.vehicles)
        stamp = message.header.stamp
        self.vehicle_sensor_timestamp_ms = int(stamp.sec * 1000 + stamp.nanosec // 1_000_000)

    def _age_ms(self, received_at):
        return None if received_at is None else int((time.monotonic() - received_at) * 1000)

    def _post_state(self):
        pointcloud_age = self._age_ms(self.last_pointcloud_rx)
        perception_age = self._age_ms(self.last_perception_rx)
        if self.mode == "general":
            # Other object topics must not refresh a cached vehicle array.
            perception_age = self._age_ms(self.last_vehicle_rx)
        pointcloud_active = pointcloud_age is not None and pointcloud_age <= self.health_timeout_sec * 1000
        perception_active = perception_age is not None and perception_age <= self.health_timeout_sec * 1000
        self.sequence += 1
        perception = (self.perception if self.mode == "crane" else
                      general_summary(self.detected, self.tracked, self.motion, self.vehicles,
                                      self.vehicle_sensor_timestamp_ms,
                                      self.vehicle_selector.select(self.vehicles),
                                      current=pointcloud_active and perception_active))
        payload = {
            "mode": self.mode,
            "sequence": self.sequence,
            "timestamp_ms": int(time.time() * 1000),
            "lidar": {
                "pointcloud_active": pointcloud_active,
                "last_pointcloud_age_ms": pointcloud_age,
                "perception_active": perception_active,
                "last_perception_age_ms": perception_age,
            },
            "perception": perception,
        }
        request = urllib.request.Request(
            self.manager_url, data=json.dumps(payload).encode("utf-8"), method="POST",
            headers={"Content-Type": "application/json", **({"X-Guardian-Bridge-Token": self.token} if self.token else {})},
        )
        try:
            with urllib.request.urlopen(request, timeout=1.0) as response:
                response.read()
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            self.get_logger().warn(f"Manager bridge POST failed: {exc}", throttle_duration_sec=10.0)


def main():
    rclpy.init()
    try:
        rclpy.spin(GuardianLidarBridge())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
