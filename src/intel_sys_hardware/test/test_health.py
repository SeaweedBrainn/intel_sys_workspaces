import pytest

from intel_sys_hardware.health import ERROR, OK, WARN, Check, parse_check


def fed(check, hz, seconds, start=0.0):
    t = start
    while t < start + seconds:
        check.mark(t)
        t += 1.0 / hz
    return t


def test_healthy_stream_reports_its_rate():
    c = Check('lidar imu', '/lidar/imu', 'sensor_msgs/msg/Imu', min_hz=100, stale_s=1.0, window=500)
    end = fed(c, 250, 3.0)
    level, msg = c.status(end)
    assert level == OK and msg.startswith('250.')


def test_slow_stream_warns():
    c = Check('odom', '/odom', 'nav_msgs/msg/Odometry', min_hz=8, stale_s=2.0)
    end = fed(c, 4, 5.0)
    assert c.status(end)[0] == WARN


def test_silent_stream_is_an_error_unless_optional():
    c = Check('camera', '/camera/camera_info', 'sensor_msgs/msg/CameraInfo', 10, 1.0)
    end = fed(c, 15, 2.0)
    assert c.status(end + 5.0) == (ERROR, 'silent for 5 s')
    c.optional = True
    assert c.status(end + 5.0)[0] == WARN


def test_never_seen():
    assert Check('map', '/map', 'nav_msgs/msg/OccupancyGrid', 0.2, 10, optional=True).status(0)[0] == WARN
    assert Check('imu', '/imu_raw', 'sensor_msgs/msg/Imu', 50, 1).status(0) == (ERROR, 'no messages yet')


def test_parse_check():
    c = parse_check('map | /map | nav_msgs/msg/OccupancyGrid | 0.2 | 10 | optional')
    assert (c.name, c.topic, c.min_hz, c.stale_s, c.optional) == ('map', '/map', 0.2, 10.0, True)
    assert not parse_check('imu|/imu_raw|sensor_msgs/msg/Imu|50|1').optional
    with pytest.raises(ValueError):
        parse_check('too|few')
