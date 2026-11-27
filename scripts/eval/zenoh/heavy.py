#!/usr/bin/env python3
"""pub: /zt_cam PointCloud2-sized blobs (bytes, hz) and /zt_lidar (bytes, 12 Hz). sub: one node on both, prints per-second counts."""
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
role = sys.argv[1]
rclpy.init(); n = Node('zt_heavy_' + role)
def cloud(nbytes):
    m = PointCloud2(); m.height = 1; m.width = nbytes // 16; m.point_step = 16; m.row_step = nbytes; m.data = bytes(nbytes); return m
if role == 'pub':
    cam_bytes, cam_hz = int(sys.argv[2]), float(sys.argv[3])
    pc = n.create_publisher(PointCloud2, '/zt_cam', qos_profile_sensor_data)
    pl = n.create_publisher(PointCloud2, '/zt_lidar', qos_profile_sensor_data)
    cm, lm = cloud(cam_bytes), cloud(5300 * 32)
    n.create_timer(1.0 / cam_hz, lambda: pc.publish(cm))
    n.create_timer(1.0 / 12, lambda: pl.publish(lm))
else:
    c = {'cam': 0, 'lidar': 0}
    n.create_subscription(PointCloud2, '/zt_cam', lambda m: c.__setitem__('cam', c['cam'] + 1), qos_profile_sensor_data)
    n.create_subscription(PointCloud2, '/zt_lidar', lambda m: c.__setitem__('lidar', c['lidar'] + 1), qos_profile_sensor_data)
    def report():
        print(f"cam {c['cam']} lidar {c['lidar']}", flush=True); c['cam'] = c['lidar'] = 0
    n.create_timer(1.0, report)
rclpy.spin(n)
