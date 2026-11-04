#!/usr/bin/env python3
"""
crane_hook_detector_node.py
Legacy standalone Hook experiment; not used by production bringup.
Production Hook detection runs in the C++ CraneBeamDetector pipeline.
This script requires an external scratch module and publishes the same topics;
run it separately from production perception.

Subscribes:
  - /ch/lslidar_point_cloud (sensor_msgs/PointCloud2)

Publishes:
  - /crane_hook/left_body   (sensor_msgs/PointCloud2) : Left Hook Body Points
  - /crane_hook/right_body  (sensor_msgs/PointCloud2) : Right Hook Body Points
  - /crane_hook/markers     (visualization_msgs/MarkerArray) : Centers, Labels & ROI Boxes
  - /crane_beam/status      (cx128s2_bev_perception/CraneBeamStatus) : Guardian Bridge compatible
"""

import sys
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header

try:
    from cx128s2_bev_perception.msg import CraneBeamStatus
    HAS_CRANE_MSG = True
except ImportError:
    HAS_CRANE_MSG = False

sys.path.insert(0, '/home/sjjin/.gemini/antigravity/brain/9d312012-efb4-49b4-897a-2901051f8115/scratch')
from crane_hook_detector_test import CraneHookDetector

def pointcloud2_to_xyz(cloud_msg):
    """Fast extraction of X, Y, Z from sensor_msgs/PointCloud2."""
    dtype = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32)
    ])
    # Assume 12 or more point_step with x, y, z at offsets 0, 4, 8
    # If point_step > 12, stride accordingly
    step = cloud_msg.point_step
    data = np.frombuffer(cloud_msg.data, dtype=np.uint8)
    n = cloud_msg.width * cloud_msg.height
    
    # Simple buffer reshape
    x = np.ndarray((n,), dtype='<f4', buffer=data, offset=0, strides=(step,))
    y = np.ndarray((n,), dtype='<f4', buffer=data, offset=4, strides=(step,))
    z = np.ndarray((n,), dtype='<f4', buffer=data, offset=8, strides=(step,))
    
    pts = np.column_stack([x, y, z])
    # Filter non-finite
    valid = np.isfinite(pts).all(axis=1)
    return pts[valid]

def create_cloud_xyz(header, points):
    points = np.asarray(points, dtype=np.float32)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = points.shape[0]
    msg.is_dense = True
    msg.is_bigendian = False
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = 12 * points.shape[0]
    msg.data = points.tobytes()
    return msg

class CraneHookDetectorNode(Node):
    def __init__(self):
        super().__init__('crane_hook_detector_node')
        
        self.input_topic = self.declare_parameter('input_topic', '/ch/lslidar_point_cloud').value
        
        self.detector = CraneHookDetector()
        
        # Publishers
        self.pub_left_body = self.create_publisher(PointCloud2, '/crane_hook/left_body', 10)
        self.pub_right_body = self.create_publisher(PointCloud2, '/crane_hook/right_body', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/crane_hook/markers', 10)
        
        if HAS_CRANE_MSG:
            self.pub_status = self.create_publisher(CraneBeamStatus, '/crane_beam/status', 10)
        else:
            self.pub_status = None

        # Subscriber
        self.sub_cloud = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            10
        )
        
        self.frame_count = 0
        self.get_logger().info(f"Crane Hook Detector Node started. Subscribing to: {self.input_topic}")

    def cloud_callback(self, cloud_msg):
        self.frame_count += 1
        t_start = self.get_clock().now()
        
        # 1. Convert to XYZ
        pts_raw = pointcloud2_to_xyz(cloud_msg)
        if len(pts_raw) == 0:
            return
            
        # 2. Run Hook Detector
        res = self.detector.detect_frame(pts_raw)
        pts_w = res["points_world"]
        l = res["left"]
        r = res["right"]
        
        # Header for outputs
        header = Header()
        header.stamp = cloud_msg.header.stamp
        header.frame_id = "world"
        
        # 3. Publish Hook Bodies
        if l["detected"] and len(l["body_indices"]) > 0:
            self.pub_left_body.publish(create_cloud_xyz(header, pts_w[l["body_indices"]]))
        if r["detected"] and len(r["body_indices"]) > 0:
            self.pub_right_body.publish(create_cloud_xyz(header, pts_w[r["body_indices"]]))
            
        # 4. Publish Markers (Centers & Labels)
        ma = MarkerArray()
        
        # Marker IDs:
        # 0: Left Sphere, 1: Left Text
        # 2: Right Sphere, 3: Right Text
        # 4: Left ROI Box, 5: Right ROI Box
        
        if l["detected"]:
            # Center Sphere
            m0 = Marker()
            m0.header = header
            m0.ns = "hook_centers"
            m0.id = 0
            m0.type = Marker.SPHERE
            m0.action = Marker.ADD
            m0.pose.position.x = l["rep_position_world"][0]
            m0.pose.position.y = l["rep_position_world"][1]
            m0.pose.position.z = l["rep_position_world"][2]
            m0.scale.x = 0.18
            m0.scale.y = 0.18
            m0.scale.z = 0.18
            m0.color.r = 1.0
            m0.color.g = 0.1
            m0.color.b = 0.1
            m0.color.a = 0.95
            ma.markers.append(m0)
            
            # Text Label
            m1 = Marker()
            m1.header = header
            m1.ns = "hook_labels"
            m1.id = 1
            m1.type = Marker.TEXT_VIEW_FACING
            m1.action = Marker.ADD
            m1.pose.position.x = l["rep_position_world"][0]
            m1.pose.position.y = l["rep_position_world"][1]
            m1.pose.position.z = l["rep_position_world"][2] + 0.35
            m1.scale.z = 0.18
            m1.color.r = 1.0
            m1.color.g = 1.0
            m1.color.b = 1.0
            m1.color.a = 1.0
            m1.text = f"LEFT HOOK\nZ: {l['vertical_pos']:.3f} m\nPts: {l['body_point_count']}"
            ma.markers.append(m1)
            
        if r["detected"]:
            # Center Sphere
            m2 = Marker()
            m2.header = header
            m2.ns = "hook_centers"
            m2.id = 2
            m2.type = Marker.SPHERE
            m2.action = Marker.ADD
            m2.pose.position.x = r["rep_position_world"][0]
            m2.pose.position.y = r["rep_position_world"][1]
            m2.pose.position.z = r["rep_position_world"][2]
            m2.scale.x = 0.18
            m2.scale.y = 0.18
            m2.scale.z = 0.18
            m2.color.r = 0.1
            m2.color.g = 1.0
            m2.color.b = 0.1
            m2.color.a = 0.95
            ma.markers.append(m2)
            
            # Text Label
            m3 = Marker()
            m3.header = header
            m3.ns = "hook_labels"
            m3.id = 3
            m3.type = Marker.TEXT_VIEW_FACING
            m3.action = Marker.ADD
            m3.pose.position.x = r["rep_position_world"][0]
            m3.pose.position.y = r["rep_position_world"][1]
            m3.pose.position.z = r["rep_position_world"][2] + 0.35
            m3.scale.z = 0.18
            m3.color.r = 1.0
            m3.color.g = 1.0
            m3.color.b = 1.0
            m3.color.a = 1.0
            m3.text = f"RIGHT HOOK\nZ: {r['vertical_pos']:.3f} m\nPts: {r['body_point_count']}"
            ma.markers.append(m3)
            
        # ROI Wireframe Boxes
        def make_roi_box(m_id, x_min, x_max, y_min, y_max, z_min, z_max, r_col, g_col, b_col):
            b = Marker()
            b.header = header
            b.ns = "hook_rois"
            b.id = m_id
            b.type = Marker.CUBE
            b.action = Marker.ADD
            b.pose.position.x = (x_min + x_max) * 0.5
            b.pose.position.y = (y_min + y_max) * 0.5
            b.pose.position.z = (z_min + z_max) * 0.5
            b.scale.x = abs(x_max - x_min)
            b.scale.y = abs(y_max - y_min)
            b.scale.z = abs(z_max - z_min)
            b.color.r = r_col
            b.color.g = g_col
            b.color.b = b_col
            b.color.a = 0.12
            return b

        ma.markers.append(make_roi_box(4, -1.5, -0.3, -0.8, 0.8, 1.5, 6.2, 0.0, 0.5, 1.0))
        ma.markers.append(make_roi_box(5, 0.3, 1.5, -0.8, 0.8, 1.5, 6.2, 0.0, 1.0, 0.5))
        self.pub_markers.publish(ma)
        
        # 5. Publish Status to Guardian Bridge
        if self.pub_status is not None:
            st = CraneBeamStatus()
            st.header = header
            st.working_status = "working" if (l["detected"] and r["detected"]) else "idle"
            st.beam_cluster_valid = bool(l["detected"] and r["detected"])
            if l["detected"] and r["detected"]:
                dist = np.linalg.norm(np.array(r["rep_position_world"]) - np.array(l["rep_position_world"]))
                st.beam_length = float(dist)
                st.z_beam = float(0.5 * (l["vertical_pos"] + r["vertical_pos"]))
            self.pub_status.publish(st)

        # 6. Terminal Log
        l_str = f"Z={l['vertical_pos']:.3f}m ({l['body_point_count']}pts)" if l["detected"] else "LOST"
        r_str = f"Z={r['vertical_pos']:.3f}m ({r['body_point_count']}pts)" if r["detected"] else "LOST"
        dt_ms = (self.get_clock().now() - t_start).nanoseconds / 1e6
        self.get_logger().info(f"[#{self.frame_count:04d}] Left: {l_str} | Right: {r_str} | Latency: {dt_ms:.1f}ms")

def main():
    rclpy.init()
    node = CraneHookDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
