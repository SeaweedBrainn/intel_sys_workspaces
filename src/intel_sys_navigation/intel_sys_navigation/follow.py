"""Where to stand to follow a tag (pure geometry, no ROS)."""
import math


def standoff_goal(robot_xy, tag_xy, standoff):
    """(x, y, yaw) standoff metres short of the tag, on the line from the robot, facing the tag.

    The line from the robot (not the tag's own facing) keeps the goal stable for a tag on a
    moving person or robot, whose orientation is noisy. If the robot is already closer than
    the standoff, the goal is the robot's own position, turned toward the tag.
    """
    dx, dy = tag_xy[0] - robot_xy[0], tag_xy[1] - robot_xy[1]
    d = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx) if d > 1e-6 else 0.0
    if d <= standoff:
        return robot_xy[0], robot_xy[1], yaw
    k = (d - standoff) / d
    return robot_xy[0] + k * dx, robot_xy[1] + k * dy, yaw


def moved(a, b, dist=0.10, ang=0.15):
    """True if goal b differs from a by more than dist metres or ang radians (or a is None)."""
    if a is None:
        return True
    dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
    return math.hypot(b[0] - a[0], b[1] - a[1]) > dist or abs(dyaw) > ang
