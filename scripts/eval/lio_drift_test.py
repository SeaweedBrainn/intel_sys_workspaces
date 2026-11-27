#!/usr/bin/env python3
"""Measure Point-LIO drift on straight legs, steering by Gazebo ground truth.

Isolates the odometry from Nav2 and RTAB-Map: the robot is driven straight
along a corridor and back (reversing, no turns) using the simulator's
noise-free odometry (/odom/wheel, from Gazebo's OdometryPublisher) for steering,
and Point-LIO's /odom is compared with it on every leg.

Start the sim first, e.g. in the south corridor of loop.sdf:

    ros2 launch intel_sys_bringup sim.launch.py headless:=true use_foxglove:=false \
        world:=loop.sdf x:=-6.0 y:=-3.5 mode:=follow   # Point-LIO without RTAB-Map

    python3 scripts/eval/lio_drift_test.py --length 12 --legs 4

Per leg it prints the error added on that leg: along-track (+ = Point-LIO
thinks it went further), cross-track, and yaw.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def pose(msg):
    p = msg.pose.pose
    return p.position.x, p.position.y, yaw_of(p.orientation)


def along_cross(start, now, heading):
    dx, dy = now[0] - start[0], now[1] - start[1]
    c, s = math.cos(heading), math.sin(heading)
    return c * dx + s * dy, -s * dx + c * dy


class DriftTest:
    def __init__(self, node, trace=False, settle=0.3):
        self.node = node
        self.settle = settle
        self.trace = trace
        self.truth = None
        self.lio = None
        self.truth_t = 0.0
        self.lio_t = 0.0
        node.create_subscription(Odometry, '/odom/wheel', self.on_truth, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/odom', self.on_lio, 10)
        self.cmd = node.create_publisher(Twist, '/cmd_vel', 10)

    def on_truth(self, m):
        self.truth, self.truth_t = pose(m), stamp(m)

    def on_lio(self, m):
        self.lio, self.lio_t = pose(m), stamp(m)

    def lag(self):
        return self.truth_t - self.lio_t

    def catch_up(self, timeout=60.0):
        """Wait until Point-LIO's output stamp reaches the current sim time."""
        target = self.truth_t
        end = time.time() + timeout
        while self.lio_t < target and time.time() < end:
            self.spin(0.1)
        self.spin(self.settle)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def stop(self):
        for _ in range(5):
            self.cmd.publish(Twist())
            self.spin(0.05)

    def leg(self, length, speed, direction):
        """Drive `length` m forward (direction=+1) or backward (-1) along the start heading."""
        t0, l0 = self.truth, self.lio
        heading_t, heading_l = t0[2], l0[2]
        last = 0.0
        max_lag = 0.0
        while True:
            self.spin(0.05)
            max_lag = max(max_lag, self.lag())
            a, c = along_cross(t0, self.truth, heading_t)
            if a * direction >= length:
                break
            if self.trace and time.time() - last >= 1.0:
                last = time.time()
                la, _ = along_cross(l0, self.lio, heading_l)
                print(f'    truth {a:+6.2f} m   point-lio {la:+6.2f} m', flush=True)
            cmd = Twist()
            cmd.linear.x = direction * speed
            cmd.linear.y = max(-0.2, min(0.2, -1.5 * c))
            cmd.angular.z = max(-0.5, min(0.5, -2.0 * wrap(self.truth[2] - heading_t)))
            self.cmd.publish(cmd)
        self.stop()
        self.spin(0.5)
        self.catch_up()  # compare like with like: Point-LIO may lag the clock
        ta, tc = along_cross(t0, self.truth, heading_t)
        la, lc = along_cross(l0, self.lio, heading_l)
        tyaw = wrap(self.truth[2] - t0[2])
        lyaw = wrap(self.lio[2] - l0[2])
        return ta, la - ta, lc - tc, math.degrees(wrap(lyaw - tyaw)), max_lag


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--length', type=float, default=12.0, help='leg length [m]')
    ap.add_argument('--speed', type=float, default=0.3, help='[m/s]')
    ap.add_argument('--legs', type=int, default=4, help='alternating forward/back legs')
    ap.add_argument('--label', default='')
    ap.add_argument('--trace', action='store_true', help='print progress once per second')
    ap.add_argument('--settle', type=float, default=3.0,
                    help='seconds to wait after stopping before comparing (Point-LIO can still correct)')
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('lio_drift_test')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    t = DriftTest(node, trace=args.trace, settle=args.settle)
    deadline = time.time() + 30
    while (t.truth is None or t.lio is None) and time.time() < deadline:
        t.spin(0.2)
    if t.truth is None or t.lio is None:
        print('No /odom/wheel or /odom; is the sim running?')
        return 2

    rows = []
    for i in range(args.legs):
        direction = 1 if i % 2 == 0 else -1
        dist, e_along, e_cross, e_yaw, max_lag = t.leg(args.length, args.speed, direction)
        rows.append((e_along, e_cross, e_yaw))
        print(f'{args.label} leg {i + 1} ({"fwd" if direction > 0 else "back"} {dist:5.2f} m): '
              f'along {e_along:+.3f} m ({100 * e_along / dist:+.1f}%)  cross {e_cross:+.3f} m  '
              f'yaw {e_yaw:+.2f} deg  max lag {max_lag:.2f} s', flush=True)
    mean_abs = sum(abs(r[0]) for r in rows) / len(rows)
    print(f'{args.label} mean |along| per leg: {mean_abs:.3f} m '
          f'({100 * mean_abs / args.length:.1f}% of distance)', flush=True)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
