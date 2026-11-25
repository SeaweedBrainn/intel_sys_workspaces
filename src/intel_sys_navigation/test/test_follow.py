import math

from intel_sys_navigation.follow import moved, standoff_goal


def test_goal_is_standoff_short_of_the_tag_facing_it():
    x, y, yaw = standoff_goal((0.0, 0.0), (2.0, 0.0), 0.457)
    assert math.isclose(x, 2.0 - 0.457) and math.isclose(y, 0.0)
    assert math.isclose(yaw, 0.0)


def test_goal_on_a_diagonal():
    x, y, yaw = standoff_goal((1.0, 1.0), (4.0, 5.0), 0.5)   # 5 m away
    assert math.isclose(math.hypot(4.0 - x, 5.0 - y), 0.5)
    assert math.isclose(yaw, math.atan2(4.0, 3.0))


def test_already_close_turns_in_place():
    x, y, yaw = standoff_goal((0.0, 0.0), (0.0, 0.3), 0.457)
    assert (x, y) == (0.0, 0.0)
    assert math.isclose(yaw, math.pi / 2)


def test_moved_thresholds():
    assert moved(None, (0, 0, 0))
    assert not moved((0, 0, 0), (0.05, 0, 0.1))
    assert moved((0, 0, 0), (0.2, 0, 0))
    assert moved((0, 0, 3.1), (0, 0, -2.9))          # wraps: 0.28 rad apart
