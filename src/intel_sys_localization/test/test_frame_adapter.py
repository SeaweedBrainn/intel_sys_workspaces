"""Frame math of point_lio_frame_adapter.

Point-LIO (gravity_align) reports the LiDAR pose in a level world frame,
camera_init, whose yaw and origin are arbitrary. The adapter must turn that into
a base_footprint pose in odom that starts at the origin, level, with zero yaw.
"""
import numpy as np
import pytest

from intel_sys_localization.point_lio_frame_adapter import (
    base_in_odom, odom_anchor, planar, rot_z, yaw_of)


def rpy(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return rot_z(yaw) @ Ry @ Rx


# lidar_joint from the URDF (tilted, facing the rear): base_link -> lidar_link
MOUNT_R = rpy(1.868822, -1.219393, 1.308338)
MOUNT_T = np.array([0.0959, 0.0, 0.1334])

# camera_init vs the world: level, but any yaw and origin
WORLD_TO_CAMINIT_YAW = 0.7
WORLD_TO_CAMINIT_T = np.array([3.0, -2.0, 0.4])


def raw_pose(R_base_world, p_base_world):
    """What Point-LIO would report for a given true base pose."""
    R_ci = rot_z(WORLD_TO_CAMINIT_YAW)
    R_sensor_world = R_base_world @ MOUNT_R
    p_sensor_world = p_base_world + R_base_world @ MOUNT_T
    R_raw = R_ci.T @ R_sensor_world
    p_raw = R_ci.T @ (p_sensor_world - WORLD_TO_CAMINIT_T)
    return R_raw, p_raw


@pytest.fixture
def start():
    # Robot starts somewhere on flat ground, heading 40 deg in the world.
    return rot_z(np.radians(40)), np.array([1.0, 2.0, 0.0])


def test_first_pose_is_origin_and_level(start):
    R0, p0 = start
    R_raw0, p_raw0 = raw_pose(R0, p0)
    R_oc, p_oc = odom_anchor(MOUNT_R, MOUNT_T, R_raw0, p_raw0)
    R_b, p_b = base_in_odom(R_oc, p_oc, MOUNT_R, MOUNT_T, R_raw0, p_raw0)
    np.testing.assert_allclose(R_b, np.eye(3), atol=1e-9)
    np.testing.assert_allclose(p_b, np.zeros(3), atol=1e-9)


def test_anchor_keeps_odom_gravity_aligned(start):
    R_oc, _ = odom_anchor(MOUNT_R, MOUNT_T, *raw_pose(*start))
    np.testing.assert_allclose(R_oc @ [0, 0, 1], [0, 0, 1], atol=1e-9)


def test_driving_forward_then_turning(start):
    R0, p0 = start
    R_oc, p_oc = odom_anchor(MOUNT_R, MOUNT_T, *raw_pose(R0, p0))

    # Drive 2 m forward and 0.5 m left (mecanum), turning 90 deg left, all in the base frame.
    R1 = R0 @ rot_z(np.radians(90))
    p1 = p0 + R0 @ np.array([2.0, 0.5, 0.0])
    R_b, p_b = base_in_odom(R_oc, p_oc, MOUNT_R, MOUNT_T, *raw_pose(R1, p1))

    np.testing.assert_allclose(p_b, [2.0, 0.5, 0.0], atol=1e-9)
    assert yaw_of(R_b) == pytest.approx(np.radians(90))
    np.testing.assert_allclose(R_b @ [0, 0, 1], [0, 0, 1], atol=1e-9)  # still level


def test_real_slope_shows_as_base_pitch(start):
    R0, p0 = start
    R_oc, p_oc = odom_anchor(MOUNT_R, MOUNT_T, *raw_pose(R0, p0))
    # Nose up 10 deg on a ramp, same heading.
    R1 = R0 @ rpy(0.0, np.radians(-10), 0.0)
    R_b, _ = base_in_odom(R_oc, p_oc, MOUNT_R, MOUNT_T, *raw_pose(R1, p0))
    pitch = np.arcsin(-R_b[2, 0])
    assert pitch == pytest.approx(np.radians(-10))


def test_planar_drops_height_and_tilt_keeps_xy_yaw():
    R = rpy(np.radians(2), np.radians(-3), np.radians(50))
    p = np.array([1.5, -0.7, 0.04])
    R_p, p_p = planar(R, p)
    np.testing.assert_allclose(p_p, [1.5, -0.7, 0.0], atol=1e-12)
    np.testing.assert_allclose(R_p @ [0, 0, 1], [0, 0, 1], atol=1e-12)
    assert yaw_of(R_p) == pytest.approx(np.radians(50))
