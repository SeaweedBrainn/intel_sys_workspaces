#!/usr/bin/env python3
"""pub or sub on /zt_<qos>; sub prints per-second counts. qos: rel, be (sensor), tl (reliable+transient local)."""
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
role, qos_name = sys.argv[1], sys.argv[2]
qos = {'rel': QoSProfile(depth=10),
       'be': qos_profile_sensor_data,
       'tl': QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)}[qos_name]
rclpy.init(); n = Node(f'zt_{role}_{qos_name}')
topic = f'/zt_{qos_name}'
if role == 'pub':
    p = n.create_publisher(String, topic, qos); i = [0]
    def tick():
        i[0] += 1; p.publish(String(data=str(i[0])))
    n.create_timer(0.1, tick)
else:
    c = [0]
    n.create_subscription(String, topic, lambda m: c.__setitem__(0, c[0] + 1), qos)
    def report():
        print(f'{time.time():.0f} {qos_name} {c[0]}', flush=True); c[0] = 0
    n.create_timer(1.0, report)
rclpy.spin(n)
