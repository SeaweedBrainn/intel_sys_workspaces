#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from intel_sys_interfaces.msg import MotorsState
from intel_sys_hardware.mecanum import MecanumChassis

class MecanumVelocityController(Node):
    def __init__(self):
        super().__init__('mecanum_velocity_controller')

        self.declare_parameter('wheelbase', 0.216)
        self.declare_parameter('track_width', 0.195)
        self.declare_parameter('wheel_diameter', 0.097)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('motor_topic', 'set_motor')
        self.declare_parameter('max_linear_x', 1.0)
        self.declare_parameter('max_linear_y', 1.0)
        self.declare_parameter('max_angular_z', 2.5)

        wheelbase = self.get_parameter('wheelbase').value
        track_width = self.get_parameter('track_width').value
        wheel_diameter = self.get_parameter('wheel_diameter').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        motor_topic = self.get_parameter('motor_topic').value

        self.max_linear_x = self.get_parameter('max_linear_x').value
        self.max_linear_y = self.get_parameter('max_linear_y').value
        self.max_angular_z = self.get_parameter('max_angular_z').value

        self.chassis = MecanumChassis(wheelbase=wheelbase, track_width=track_width, wheel_diameter=wheel_diameter)
        self.motor_pub = self.create_publisher(MotorsState, motor_topic, 10)
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)

        self.get_logger().info(f'Mecanum Velocity Controller started. Subscribed to {cmd_vel_topic}, publishing to {motor_topic}.')

    def cmd_vel_callback(self, msg: Twist):
        vx = max(-self.max_linear_x, min(self.max_linear_x, msg.linear.x))
        vy = max(-self.max_linear_y, min(self.max_linear_y, msg.linear.y))
        wz = max(-self.max_angular_z, min(self.max_angular_z, msg.angular.z))

        motors_msg = self.chassis.compute_motor_speeds(vx, vy, wz)
        self.motor_pub.publish(motors_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumVelocityController()
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
