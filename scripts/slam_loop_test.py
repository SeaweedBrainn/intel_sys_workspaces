#!/usr/bin/env python3
"""Drive the loop world with Nav2 and check the SLAM chain against Gazebo truth.

Phase 6 check from docs/ROADMAP.md, in simulation. Start the sim first:

    ros2 launch intel_sys_bringup sim.launch.py headless:=true use_foxglove:=false \
        world:=loop.sdf x:=-6.5 y:=0.0

then, in the same container:

    python3 scripts/slam_loop_test.py            # one lap, mapping
    python3 scripts/slam_loop_test.py --check-only   # just compare poses now

It reports RTAB-Map loop/proximity closures seen during the run and, at each
waypoint, the error of map->base_footprint (SLAM) and odom->base_footprint
(Point-LIO alone) against the Gazebo model pose.
"""
import argparse
import math
import re
import subprocess
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.time import Time
from rtabmap_msgs.msg import Info
from tf2_ros import Buffer, TransformListener

# Counter-clockwise lap of loop.sdf, world frame, ending back at the spawn.
LAP = [(-6.5, -3.5), (6.5, -3.5), (6.5, 3.5), (-6.5, 3.5), (-6.5, 0.0)]


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def gazebo_pose(model):
    """(x, y, yaw) of a model from `ign model -p`."""
    out = subprocess.run(['ign', 'model', '-m', model, '-p'],
                         capture_output=True, text=True, timeout=10).stdout
    rows = re.findall(r'\[([-\d.e ]+)\]', out)
    xyz, rpy = [list(map(float, r.split())) for r in rows[-2:]]
    return xyz[0], xyz[1], rpy[2]


class Checker:
    def __init__(self, nav, spawn, model):
        self.nav = nav
        self.spawn = spawn
        self.model = model
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, nav)
        self.loops = []
        nav.create_subscription(Info, '/rtabmap/info', self.on_info, 10)

    def on_info(self, msg):
        if msg.loop_closure_id > 0 or msg.proximity_detection_id > 0:
            self.loops.append((msg.ref_id, msg.loop_closure_id, msg.proximity_detection_id))

    def truth_in_map(self):
        """Gazebo pose expressed in the map frame (map = spawn pose at start)."""
        x, y, yaw = gazebo_pose(self.model)
        sx, sy, syaw = self.spawn
        dx, dy = x - sx, y - sy
        c, s = math.cos(-syaw), math.sin(-syaw)
        return c * dx - s * dy, s * dx + c * dy, wrap(yaw - syaw)

    def lookup(self, parent):
        t = self.tf.lookup_transform(parent, 'base_footprint', Time(), Duration(seconds=2.0))
        tr = t.transform.translation
        return tr.x, tr.y, yaw_from_quat(t.transform.rotation)

    def report(self, label):
        for _ in range(20):
            rclpy.spin_once(self.nav, timeout_sec=0.1)
        tx, ty, tyaw = self.truth_in_map()
        line = f'{label:>12}  truth=({tx:6.2f},{ty:6.2f},{math.degrees(tyaw):6.1f}deg)'
        errs = {}
        for frame in ('map', 'odom'):
            try:
                x, y, yaw = self.lookup(frame)
            except Exception as e:  # noqa: BLE001
                line += f'  {frame}: n/a ({type(e).__name__})'
                continue
            e_xy = math.hypot(x - tx, y - ty)
            e_yaw = math.degrees(abs(wrap(yaw - tyaw)))
            errs[frame] = (e_xy, e_yaw)
            line += f'  {frame}_err={e_xy:5.3f}m/{e_yaw:4.1f}deg'
        print(line, flush=True)
        return errs


def to_map(nav, spawn, wx, wy, yaw):
    sx, sy, syaw = spawn
    c, s = math.cos(-syaw), math.sin(-syaw)
    p = PoseStamped()
    p.header.frame_id = 'map'
    # Stamp 0 = "latest transform". A real stamp goes stale while Nav2 replans
    # toward the goal and TF lookups fail once it falls out of the buffer.
    p.header.stamp.sec = 0
    p.header.stamp.nanosec = 0
    p.pose.position.x = c * (wx - sx) - s * (wy - sy)
    p.pose.position.y = s * (wx - sx) + c * (wy - sy)
    p.pose.orientation.z = math.sin((yaw - syaw) / 2)
    p.pose.orientation.w = math.cos((yaw - syaw) / 2)
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--spawn', type=float, nargs=3, default=[-6.5, 0.0, 0.0],
                    metavar=('X', 'Y', 'YAW'), help='spawn pose given to sim.launch.py (world frame)')
    ap.add_argument('--model', default='intel_sys_robot')
    ap.add_argument('--laps', type=int, default=1)
    ap.add_argument('--check-only', action='store_true')
    ap.add_argument('--max-err', type=float, default=0.25, help='pass threshold for map error at the end [m]')
    args = ap.parse_args()

    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    checker = Checker(nav, tuple(args.spawn), args.model)

    if args.check_only:
        errs = checker.report('now')
        rclpy.shutdown()
        return 0 if errs.get('map', (1e9,))[0] <= args.max_err else 1

    nav._waitForNodeToActivate('bt_navigator')
    checker.report('start')

    for lap in range(args.laps):
        for i, (wx, wy) in enumerate(LAP):
            nxt = LAP[(i + 1) % len(LAP)]
            heading = math.atan2(nxt[1] - wy, nxt[0] - wx)
            nav.goToPose(to_map(nav, checker.spawn, wx, wy, heading))
            while not nav.isTaskComplete():
                rclpy.spin_once(nav, timeout_sec=0.2)
            result = nav.getResult()
            checker.report(f'lap{lap + 1} wp{i + 1}')
            if result != TaskResult.SUCCEEDED:
                print(f'Waypoint {i + 1} failed: {result}', flush=True)
                rclpy.shutdown()
                return 2

    errs = checker.report('end')
    print(f'RTAB-Map closures during run: {len(checker.loops)} '
          f'(loop={sum(1 for _, lc, _ in checker.loops if lc > 0)}, '
          f'proximity={sum(1 for *_, pd in checker.loops if pd > 0)})')
    ok = 'map' in errs and errs['map'][0] <= args.max_err and len(checker.loops) > 0
    print('PASS' if ok else 'FAIL')
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
