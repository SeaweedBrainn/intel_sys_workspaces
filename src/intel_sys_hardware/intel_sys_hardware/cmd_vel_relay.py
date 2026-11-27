"""Fallback for cmd_vel_mux.launch.py when twist_mux is missing: /cmd_vel -> /cmd_vel_mux."""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node('cmd_vel_relay')
    pub = node.create_publisher(Twist, 'cmd_vel_mux', 10)
    node.create_subscription(Twist, 'cmd_vel', pub.publish, 10)
    node.get_logger().warn('twist_mux missing: relaying /cmd_vel only (no teleop priority, no e-stop)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
