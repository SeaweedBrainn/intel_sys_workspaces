import math
import pytest
from intel_sys_hardware.mecanum import MecanumChassis

def test_speed_to_rps():
    wheel_diameter = 0.097  # meters
    chassis = MecanumChassis(wheelbase=0.216, track_width=0.195, wheel_diameter=wheel_diameter)
    circumference = math.pi * wheel_diameter

    # If speed is equal to circumference, RPS should be 1.0
    assert pytest.approx(chassis.speed_to_rps(circumference), rel=1e-5) == 1.0
    # Zero speed -> 0 RPS
    assert chassis.speed_to_rps(0.0) == 0.0

def test_forward_motion():
    chassis = MecanumChassis(wheelbase=0.216, track_width=0.195, wheel_diameter=0.097)
    vx = 0.5
    msg = chassis.compute_motor_speeds(linear_x=vx, linear_y=0.0, angular_z=0.0)

    assert len(msg.data) == 4
    # All 4 wheels should rotate with the same positive speed for forward motion (with signs matched to motor orientation)
    expected_rps = chassis.speed_to_rps(vx)
    assert pytest.approx(msg.data[0].rps, rel=1e-4) == expected_rps
    assert pytest.approx(msg.data[1].rps, rel=1e-4) == expected_rps
    assert pytest.approx(msg.data[2].rps, rel=1e-4) == -expected_rps
    assert pytest.approx(msg.data[3].rps, rel=1e-4) == -expected_rps

def test_strafe_motion():
    chassis = MecanumChassis(wheelbase=0.216, track_width=0.195, wheel_diameter=0.097)
    vy = 0.3
    msg = chassis.compute_motor_speeds(linear_x=0.0, linear_y=vy, angular_z=0.0)

    assert len(msg.data) == 4
    expected_rps = chassis.speed_to_rps(vy)
    # v1 = -vy, v2 = +vy, v3 = +vy, v4 = -vy
    assert pytest.approx(msg.data[0].rps, rel=1e-4) == -expected_rps
    assert pytest.approx(msg.data[1].rps, rel=1e-4) == expected_rps
    assert pytest.approx(msg.data[2].rps, rel=1e-4) == -expected_rps
    assert pytest.approx(msg.data[3].rps, rel=1e-4) == expected_rps

def test_pure_rotation():
    chassis = MecanumChassis(wheelbase=0.216, track_width=0.195, wheel_diameter=0.097)
    wz = 1.0
    half_base = (0.216 + 0.195) / 2.0
    msg = chassis.compute_motor_speeds(linear_x=0.0, linear_y=0.0, angular_z=wz)

    assert len(msg.data) == 4
    # v1 = -wz * half_base, v2 = -wz * half_base, v3 = +wz * half_base, v4 = +wz * half_base
    v_rot = wz * half_base
    expected_rps = chassis.speed_to_rps(v_rot)
    assert pytest.approx(msg.data[0].rps, rel=1e-4) == -expected_rps
    assert pytest.approx(msg.data[1].rps, rel=1e-4) == -expected_rps
    assert pytest.approx(msg.data[2].rps, rel=1e-4) == -expected_rps
    assert pytest.approx(msg.data[3].rps, rel=1e-4) == -expected_rps
