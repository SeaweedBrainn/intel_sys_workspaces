#!/usr/bin/env python3
# encoding: utf-8
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from intel_sys_interfaces.msg import MotorsState

ODOM_POSE_COVARIANCE = [
    1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1e-3
]

ODOM_TWIST_COVARIANCE = [
    1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1e-3
]

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

class MecanumOdometryIntegrator:
    def __init__(self, wheelbase=0.216, track_width=0.195, wheel_diameter=0.097):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter
        self.half_base = (wheelbase + track_width) / 2.0
        self.circumference = math.pi * wheel_diameter

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    def rps_to_speed(self, rps: float) -> float:
        return rps * self.circumference

    def forward_kinematics(self, rps1: float, rps2: float, rps3: float, rps4: float):
        v1 = self.rps_to_speed(rps1)
        v2 = self.rps_to_speed(rps2)
        v3 = -self.rps_to_speed(rps3)
        v4 = -self.rps_to_speed(rps4)

        self.vx = (v1 + v2 + v3 + v4) / 4.0
        self.vy = (-v1 + v2 + v3 - v4) / 4.0
        self.wz = (-v1 - v2 + v3 + v4) / (4.0 * self.half_base)
        return self.vx, self.vy, self.wz

    def update_from_twist(self, vx: float, vy: float, wz: float):
        self.vx = vx
        self.vy = vy
        self.wz = wz

    def integrate(self, dt: float):
        if dt <= 0.0:
            return self.x, self.y, self.yaw

        delta_x = (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        delta_y = (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        delta_yaw = self.wz * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw

        # Normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        return self.x, self.y, self.yaw

class OdomPublisherNode(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        self.declare_parameter('wheelbase', 0.216)
        self.declare_parameter('track_width', 0.195)
        self.declare_parameter('wheel_diameter', 0.097)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('motor_topic', 'set_motor')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('rate_hz', 50.0)

        wheelbase = self.get_parameter('wheelbase').value
        track_width = self.get_parameter('track_width').value
        wheel_diameter = self.get_parameter('wheel_diameter').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        odom_topic = self.get_parameter('odom_topic').value
        motor_topic = self.get_parameter('motor_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        rate_hz = self.get_parameter('rate_hz').value

        self.integrator = MecanumOdometryIntegrator(
            wheelbase=wheelbase, track_width=track_width, wheel_diameter=wheel_diameter
        )

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(MotorsState, motor_topic, self.motor_callback, 10)
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate_hz, self.update_and_publish)

        self.get_logger().info(
            f'Odom publisher started on topic {odom_topic} (frame: {self.odom_frame} -> {self.base_frame}, publish_tf={self.publish_tf})'
        )

    def motor_callback(self, msg: MotorsState):
        if len(msg.data) >= 4:
            self.integrator.forward_kinematics(
                msg.data[0].rps, msg.data[1].rps, msg.data[2].rps, msg.data[3].rps
            )

    def cmd_vel_callback(self, msg: Twist):
        # Fallback if motor speeds are not streaming
        self.integrator.update_from_twist(msg.linear.x, msg.linear.y, msg.angular.z)

    def update_and_publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt > 0.5:
            # Skip huge dt jumps (e.g. after pause or pause on start)
            dt = 0.0

        x, y, yaw = self.integrator.integrate(dt)
        q = yaw_to_quaternion(yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = q
        odom_msg.pose.covariance = ODOM_POSE_COVARIANCE

        odom_msg.twist.twist.linear.x = self.integrator.vx
        odom_msg.twist.twist.linear.y = self.integrator.vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.integrator.wz
        odom_msg.twist.covariance = ODOM_TWIST_COVARIANCE

        self.odom_pub.publish(odom_msg)

        if self.publish_tf and self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
