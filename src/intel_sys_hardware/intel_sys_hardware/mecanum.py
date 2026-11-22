#!/usr/bin/env python3
# encoding: utf-8
import math
from intel_sys_interfaces.msg import MotorState, MotorsState

def forward_sign(forward_is_sensor_side):
    """+1 keeps the STM32's motor convention for "forward"; -1 turns the robot's front around.

    The rover's sensors (L2 and D435) sit on what the motor convention calls the back.
    With forward_is_sensor_side, +x in /cmd_vel and in wheel odometry is the sensor side,
    so teleop, the dashboard, Nav2 and the URDF all agree on one front. Flipping linear x
    and y is a 180 deg turn about z; yaw rate is unchanged.
    """
    return -1.0 if forward_is_sensor_side else 1.0


class MecanumChassis:
    def __init__(self, wheelbase=0.216, track_width=0.195, wheel_diameter=0.097, forward_sign=1.0):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter
        self.forward_sign = forward_sign

    def speed_to_rps(self, speed_mps):
        # speed (m/s) / circumference (m) = revolutions per second
        return speed_mps / (math.pi * self.wheel_diameter)

    def compute_motor_speeds(self, linear_x, linear_y, angular_z):
        linear_x *= self.forward_sign
        linear_y *= self.forward_sign
        half_base = (self.wheelbase + self.track_width) / 2.0
        v1 = linear_x - linear_y - angular_z * half_base
        v2 = linear_x + linear_y - angular_z * half_base
        v3 = linear_x + linear_y + angular_z * half_base
        v4 = linear_x - linear_y + angular_z * half_base

        speeds_rps = [
            self.speed_to_rps(v1),
            self.speed_to_rps(v2),
            -self.speed_to_rps(v3),
            -self.speed_to_rps(v4)
        ]

        motors_msg = MotorsState()
        motors_msg.data = []
        for i, rps in enumerate(speeds_rps):
            m = MotorState()
            m.id = i + 1
            m.rps = float(rps)
            motors_msg.data.append(m)
        return motors_msg
