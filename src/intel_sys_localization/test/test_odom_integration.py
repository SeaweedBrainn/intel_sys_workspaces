import math
import pytest
from intel_sys_localization.odom_publisher import MecanumOdometryIntegrator, yaw_to_quaternion

def test_yaw_to_quaternion():
    # Yaw = 0 -> identity orientation (0, 0, 0, 1)
    q0 = yaw_to_quaternion(0.0)
    assert q0.x == 0.0
    assert q0.y == 0.0
    assert q0.z == 0.0
    assert q0.w == 1.0

    # Yaw = pi -> (0, 0, 1, 0)
    q_pi = yaw_to_quaternion(math.pi)
    assert pytest.approx(q_pi.z, rel=1e-4) == 1.0
    assert pytest.approx(q_pi.w, abs=1e-4) == 0.0

def test_pure_forward_odometry_integration():
    integrator = MecanumOdometryIntegrator(wheelbase=0.216, track_width=0.195, wheel_diameter=0.097)
    vx = 0.4
    integrator.update_from_twist(vx=vx, vy=0.0, wz=0.0)

    # Integrate for 2.5 seconds
    dt = 0.5
    for _ in range(5):
        integrator.integrate(dt)

    assert pytest.approx(integrator.x, rel=1e-3) == vx * 2.5
    assert pytest.approx(integrator.y, abs=1e-4) == 0.0
    assert pytest.approx(integrator.yaw, abs=1e-4) == 0.0

def test_pure_strafe_odometry_integration():
    integrator = MecanumOdometryIntegrator(wheelbase=0.216, track_width=0.195, wheel_diameter=0.097)
    vy = 0.3
    integrator.update_from_twist(vx=0.0, vy=vy, wz=0.0)

    # Integrate for 2 seconds
    dt = 1.0
    integrator.integrate(dt)
    integrator.integrate(dt)

    assert pytest.approx(integrator.x, abs=1e-4) == 0.0
    assert pytest.approx(integrator.y, rel=1e-3) == vy * 2.0
    assert pytest.approx(integrator.yaw, abs=1e-4) == 0.0

def test_pure_rotation_odometry_integration():
    integrator = MecanumOdometryIntegrator(wheelbase=0.216, track_width=0.195, wheel_diameter=0.097)
    wz = 0.5
    integrator.update_from_twist(vx=0.0, vy=0.0, wz=wz)

    # Integrate for 2 seconds
    integrator.integrate(2.0)

    assert pytest.approx(integrator.x, abs=1e-4) == 0.0
    assert pytest.approx(integrator.y, abs=1e-4) == 0.0
    assert pytest.approx(integrator.yaw, rel=1e-3) == 1.0

def test_forward_kinematics_wheel_speeds():
    integrator = MecanumOdometryIntegrator(wheelbase=0.216, track_width=0.195, wheel_diameter=0.097)
    # All 4 wheels rotating at 1 RPS forward
    vx, vy, wz = integrator.forward_kinematics(rps1=1.0, rps2=1.0, rps3=1.0, rps4=1.0)

    expected_vx = 1.0 * (math.pi * 0.097)
    assert vx == pytest.approx(expected_vx, rel=1e-3)
    assert vy == pytest.approx(0.0, abs=1e-4)
    assert wz == pytest.approx(0.0, abs=1e-4)
