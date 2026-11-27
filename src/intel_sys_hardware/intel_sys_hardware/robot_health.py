"""Publish /diagnostics for the robot's data streams, for Foxglove's Diagnostics panel.

Each check (intel_sys_hardware.health) watches one topic's rate and freshness; TF checks
watch how old the latest odom -> base_footprint (Point-LIO) and map -> odom (RTAB-Map)
transforms are. Heavy topics are watched through a light companion (camera_info for the
camera, the L2's IMU and Point-LIO's odometry for the LiDAR), so this node adds almost no
traffic.

    ros2 launch intel_sys_hardware health.launch.py
"""
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformListener

from intel_sys_hardware.health import ERROR, OK, WARN, parse_check

LEVELS = {OK: DiagnosticStatus.OK, WARN: DiagnosticStatus.WARN, ERROR: DiagnosticStatus.ERROR}


class RobotHealth(Node):
    def __init__(self):
        super().__init__('robot_health')
        specs = self.declare_parameter('checks', ['']).value
        # parent|child|stale_s[|optional]
        tf_specs = self.declare_parameter('tf_checks', ['']).value
        self.checks = [parse_check(s) for s in specs if s.strip()]
        self.tf_checks = []
        for s in tf_specs:
            if s.strip():
                p = [x.strip() for x in s.split('|')]
                self.tf_checks.append((p[0], p[1], float(p[2]), len(p) > 3 and p[3] == 'optional'))
        for c in list(self.checks):
            try:
                msg_class = get_message(c.msg_type)
            except (AttributeError, ModuleNotFoundError, ValueError):
                self.get_logger().warn(f'{c.name}: message type {c.msg_type} not installed; check skipped')
                self.checks.remove(c)
                continue
            qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(msg_class, c.topic,
                                     lambda _m, c=c: c.mark(time.monotonic()), qos)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f'{len(self.checks)} stream checks, {len(self.tf_checks)} TF checks')

    def tf_status(self, parent, child, stale_s, optional):
        try:
            t = self.tf.lookup_transform(parent, child, Time())
        except Exception:  # noqa: BLE001
            return (WARN if optional else ERROR), 'no transform'
        stamp = Time.from_msg(t.header.stamp)
        if stamp.nanoseconds == 0:
            return OK, 'static'
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if age > stale_s:
            return (WARN if optional else ERROR), f'last update {age:.1f} s ago'
        return OK, 'updated just now' if age < 0.05 else f'updated {age:.2f} s ago'

    def publish(self):
        now = time.monotonic()
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for c in self.checks:
            level, text = c.status(now)
            age = c.age(now)
            msg.status.append(DiagnosticStatus(
                level=LEVELS[level], name=f'streams: {c.name}', message=text, hardware_id=c.topic,
                values=[KeyValue(key='topic', value=c.topic),
                        KeyValue(key='rate_hz', value=f'{c.rate(now):.2f}'),
                        KeyValue(key='age_s', value='' if age is None else f'{age:.2f}')]))
        for parent, child, stale_s, optional in self.tf_checks:
            level, text = self.tf_status(parent, child, stale_s, optional)
            msg.status.append(DiagnosticStatus(
                level=LEVELS[level], name=f'tf: {parent} -> {child}', message=text,
                hardware_id=f'{parent}->{child}'))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RobotHealth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
