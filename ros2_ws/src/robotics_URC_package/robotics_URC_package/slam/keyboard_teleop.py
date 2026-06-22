#!/usr/bin/env python3
"""
Keyboard teleop node for controlling a robot via /cmd_vel.
Arrow keys to move, Space to stop. Smooth continuous publishing.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading

# Speed settings
LINEAR_SPEED  = 0.3   # m/s
ANGULAR_SPEED = 1.0   # rad/s
SPEED_STEP    = 0.05  # increment per +/- press

INSTRUCTIONS = """
Robot Keyboard Controller
─────────────────────────
  Arrow Up    : Move forward
  Arrow Down  : Move backward
  Arrow Left  : Turn left
  Arrow Right : Turn right
  W / S       : Increase / Decrease linear speed
  A / D       : Increase / Decrease angular speed
  Space       : Stop
  Ctrl+C      : Quit

  Smooth continuous motion.
─────────────────────────
"""

# ANSI escape sequences for arrow keys
ARROW_UP    = '\x1b[A'
ARROW_DOWN  = '\x1b[B'
ARROW_RIGHT = '\x1b[C'
ARROW_LEFT  = '\x1b[D'


def get_key(settings):
    """Read a single keypress from stdin."""
    tty.setraw(sys.stdin.fileno())
    ch1 = sys.stdin.read(1)
    if ch1 == '\x1b':
        ch2 = sys.stdin.read(1)
        ch3 = sys.stdin.read(1)
        key = ch1 + ch2 + ch3
    else:
        key = ch1
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher     = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed  = LINEAR_SPEED
        self.angular_speed = ANGULAR_SPEED

        # State for velocity
        self.target_linear  = 0.0
        self.target_angular = 0.0
        self.running        = True
        self.last_input_time = 0.0

        # Publish continuously at 50Hz (every 20ms)
        self.create_timer(0.02, self._publish_velocity)
        
        # Check for timeout every 100ms
        self.create_timer(0.1, self._check_timeout)

    def _publish_velocity(self):
        """Continuously publish the current velocity."""
        msg = Twist()
        msg.linear.x  = self.target_linear
        msg.angular.z = self.target_angular
        self.publisher.publish(msg)

    def _check_timeout(self):
        """Stop if no input for 0.2 seconds."""
        import time
        if time.time() - self.last_input_time > 0.2:
            self.target_linear = 0.0
            self.target_angular = 0.0

    def set_velocity(self, linear_x=0.0, angular_z=0.0):
        """Set the target velocity and update input time."""
        import time
        self.target_linear  = linear_x
        self.target_angular = angular_z
        self.last_input_time = time.time()

    def stop(self):
        import time
        self.target_linear  = 0.0
        self.target_angular = 0.0
        self.last_input_time = time.time()


def main():
    rclpy.init()
    node = KeyboardTeleop()

    # Spin ROS in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)

    print(INSTRUCTIONS)
    print(f"Linear speed:  {node.linear_speed:.2f} m/s")
    print(f"Angular speed: {node.angular_speed:.2f} rad/s")
    print()

    try:
        while True:
            key = get_key(settings)

            if key == '\x03':  # Ctrl+C
                break
            elif key == ARROW_UP:
                node.set_velocity(linear_x=-node.linear_speed)
                print(f"\r► Forward  | linear={node.linear_speed:.2f}  angular={node.angular_speed:.2f}   ", end='', flush=True)
            elif key == ARROW_DOWN:
                node.set_velocity(linear_x=node.linear_speed)
                print(f"\r◄ Backward | linear={node.linear_speed:.2f}  angular={node.angular_speed:.2f}   ", end='', flush=True)
            elif key == ARROW_LEFT:
                node.set_velocity(angular_z=node.angular_speed)
                print(f"\r↺ Turn L   | linear={node.linear_speed:.2f}  angular={node.angular_speed:.2f}   ", end='', flush=True)
            elif key == ARROW_RIGHT:
                node.set_velocity(angular_z=-node.angular_speed)
                print(f"\r↻ Turn R   | linear={node.linear_speed:.2f}  angular={node.angular_speed:.2f}   ", end='', flush=True)
            elif key == ' ':
                node.stop()
                print(f"\r■ STOP     | linear={node.linear_speed:.2f}  angular={node.angular_speed:.2f}   ", end='', flush=True)
            elif key in ('w', 'W'):
                node.linear_speed = round(node.linear_speed + SPEED_STEP, 2)
                print(f"\r↑ Lin spd  | linear={node.linear_speed:.2f}  angular={node.angular_speed:.2f}   ", end='', flush=True)
            elif key in ('s', 'S'):
                node.linear_speed = max(0.05, round(node.linear_speed - SPEED_STEP, 2))
                print(f"\r↓ Lin spd  | linear={node.linear_speed:.2f}  angular={node.angular_speed:.2f}   ", end='', flush=True)
            elif key in ('a', 'A'):
                node.angular_speed = round(node.angular_speed + SPEED_STEP, 2)
                print(f"\r↑ Ang spd  | linear={node.linear_speed:.2f}  angular={node.angular_speed:.2f}   ", end='', flush=True)
            elif key in ('d', 'D'):
                node.angular_speed = max(0.05, round(node.angular_speed - SPEED_STEP, 2))
                print(f"\r↓ Ang spd  | linear={node.linear_speed:.2f}  angular={node.angular_speed:.2f}   ", end='', flush=True)

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        node.running = False
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nShutdown.")


if __name__ == '__main__':
    main()