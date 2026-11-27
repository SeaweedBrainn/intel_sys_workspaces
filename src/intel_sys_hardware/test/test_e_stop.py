"""mecanum_controller honours /e_stop: wheels to zero at once, commands ignored until released."""
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from intel_sys_hardware.mecanum_controller import MecanumVelocityController


def speeds(msg):
    return [round(m.rps, 6) for m in msg.data]


def test_e_stop_zeroes_wheels_and_blocks_commands():
    rclpy.init()
    try:
        node = MecanumVelocityController()
        sent = []
        node.motor_pub.publish = sent.append
        fwd = Twist()
        fwd.linear.x = 0.2

        node.cmd_vel_callback(fwd)
        assert any(speeds(sent[-1]))                      # driving

        node.e_stop_callback(Bool(data=True))
        assert not any(speeds(sent[-1]))                  # stopped at once
        n = len(sent)
        node.cmd_vel_callback(fwd)
        assert len(sent) == n                             # ignored while stopped

        node.e_stop_callback(Bool(data=False))
        node.cmd_vel_callback(fwd)
        assert any(speeds(sent[-1]))                      # drives again after release
        node.destroy_node()
    finally:
        rclpy.shutdown()
