#!/usr/bin/env python3
"""Record Point-LIO's output while a bag plays, then report and draw it.

Run by scripts/run_lio_on_bag.sh (bag playback + Point-LIO + adapter). Logs:
  - /point_lio/raw_odom: trajectory (LiDAR pose in Point-LIO's level world frame)
  - lag: how far Point-LIO's output stamp trails the bag clock
  - /point_lio/raw_cloud_registered: accumulated, voxel-thinned map
and on exit writes <out>.png (top view: map grey, path blue, start green,
end red; side view below) and <out>.npz.

Without ground truth, judge it by: does it keep up (lag), is the path smooth
and plausible, are walls in the map single and sharp (doubled or smeared
walls mean drift), and does a run that returns to its start end there.
"""
import argparse
import signal
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from PIL import Image, ImageDraw
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

VOXEL = 0.05


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Recorder:
    def __init__(self, node, odom_topic='/point_lio/raw_odom', cloud_topic='/point_lio/raw_cloud_registered',
                 cloud_latest=False):
        self.cloud_latest = cloud_latest
        self.clock = None
        self.traj = []      # t, x, y, z, qx, qy, qz, qw
        self.lag = []       # clock time, lag
        self.pts = []
        self.last_print = 0.0
        node.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        node.create_subscription(Odometry, odom_topic, self.on_odom, 50)
        node.create_subscription(PointCloud2, cloud_topic, self.on_cloud, qos_profile_sensor_data)

    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = stamp(m.header)
        if self.traj and t - self.traj[-1][0] < 0.05:
            return  # 20 Hz is plenty for the plot
        self.traj.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        if self.clock is not None:
            self.lag.append((self.clock, self.clock - t))

    def on_cloud(self, m):
        # Keep this cheap: it shares the thread that timestamps odometry for the
        # lag measurement. Every 3rd cloud, voxel-thinned with numpy; merged later.
        self.n_clouds = getattr(self, 'n_clouds', 0) + 1
        if self.cloud_latest:
            self.latest_cloud = m  # whole map republished each update: keep only the newest
            return
        if self.n_clouds % 3:
            return
        a = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.stack([a['x'], a['y'], a['z']], -1).astype(np.float32)
        _, idx = np.unique(np.floor(xyz / VOXEL).astype(np.int32), axis=0, return_index=True)
        self.pts.append(xyz[idx])
        if len(self.pts) >= 50:
            self.pts = [self.merged()]

    def merged(self):
        if self.cloud_latest and getattr(self, 'latest_cloud', None) is not None:
            a = pc2.read_points(self.latest_cloud, field_names=('x', 'y', 'z'), skip_nans=True)
            self.pts = [np.stack([a['x'], a['y'], a['z']], -1).astype(np.float32)]
            self.latest_cloud = None
        if not self.pts:
            return np.zeros((0, 3), np.float32)
        xyz = np.concatenate(self.pts)
        _, idx = np.unique(np.floor(xyz / VOXEL).astype(np.int32), axis=0, return_index=True)
        return xyz[idx]

    def status(self):
        if self.clock and time.time() - self.last_print > 10:
            self.last_print = time.time()
            lag = self.lag[-1][1] if self.lag else float('nan')
            print(f'  bag t={self.clock:.0f}  odom msgs={len(self.traj)}  lag={lag:.2f} s',
                  flush=True)


def draw(rec, out, title):
    tr = np.array(rec.traj)
    pts = rec.merged()
    if len(tr) == 0:
        print('no odometry received')
        return
    xy = np.concatenate([pts[:, :2], tr[:, 1:3]]) if len(pts) else tr[:, 1:3]
    lo, hi = np.percentile(xy, 0.5, axis=0) - 1.0, np.percentile(xy, 99.5, axis=0) + 1.0
    span = max(hi - lo)
    px = 1400 / span
    W, H = int((hi[0] - lo[0]) * px) + 20, int((hi[1] - lo[1]) * px) + 20
    zlo, zhi = np.percentile(pts[:, 2], 1) if len(pts) else -1, np.percentile(pts[:, 2], 99) if len(pts) else 2
    SH = int((zhi - zlo + 1) * px) + 20
    img = Image.new('RGB', (W, H + SH + 30), (255, 255, 255))
    d = ImageDraw.Draw(img)

    def top(x, y):
        return 10 + (x - lo[0]) * px, 10 + (hi[1] - y) * px

    for x, y, z in pts:
        d.point(top(x, y), fill=(120, 120, 120))
    d.line([top(x, y) for x, y in tr[:, 1:3]], fill=(0, 90, 255), width=2)
    for (x, y), col in [(tr[0, 1:3], (0, 170, 0)), (tr[-1, 1:3], (220, 0, 0))]:
        u, v = top(x, y)
        d.ellipse([u - 6, v - 6, u + 6, v + 6], fill=col)
    # scale bar
    d.line([(10, H - 5), (10 + 5 * px, H - 5)], fill=(0, 0, 0), width=3)
    d.text((14, H - 20), '5 m', fill=(0, 0, 0))
    # side view (x vs z)
    y0 = H + 20

    def side(x, z):
        return 10 + (x - lo[0]) * px, y0 + (zhi - z) * px

    for x, y, z in pts[::3]:
        d.point(side(x, z), fill=(160, 160, 160))
    d.line([side(x, z) for x, z in tr[:, [1, 3]]], fill=(0, 90, 255), width=2)
    d.text((10, y0 - 16), 'side view (x vs z)', fill=(0, 0, 0))
    d.text((10, H + SH + 12), title, fill=(0, 0, 0))
    img.save(out + '.png')
    np.savez_compressed(out + '.npz', traj=tr, lag=np.array(rec.lag), map=pts)


def summary(rec):
    tr = np.array(rec.traj)
    if len(tr) < 2:
        return
    step = np.linalg.norm(np.diff(tr[:, 1:4], axis=0), axis=1)
    dt = np.diff(tr[:, 0])
    v = step / np.maximum(dt, 1e-3)
    lag = np.array(rec.lag)[:, 1] if rec.lag else np.array([np.nan])
    print(f'duration {tr[-1, 0] - tr[0, 0]:.0f} s, path length {step.sum():.1f} m, '
          f'start-end distance {np.linalg.norm(tr[-1, 1:4] - tr[0, 1:4]):.2f} m '
          f'(xy {np.linalg.norm(tr[-1, 1:3] - tr[0, 1:3]):.2f}, z {tr[-1, 3] - tr[0, 3]:+.2f})')
    print(f'speed: median {np.median(v):.2f} m/s, p99 {np.percentile(v, 99):.2f}, max {v.max():.2f}')
    print(f'z range {tr[:, 3].min():+.2f}..{tr[:, 3].max():+.2f} m')
    print(f'lag: median {np.median(lag):.2f} s, p95 {np.percentile(lag, 95):.2f}, max {lag.max():.2f}')
    print(f'map: {len(rec.merged())} voxels at {VOXEL} m')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True, help='output path prefix (no extension)')
    ap.add_argument('--title', default='')
    ap.add_argument('--odom-topic', default='/point_lio/raw_odom',
                    help='odometry to record (icp_odometry: /odom)')
    ap.add_argument('--cloud-topic', default='/point_lio/raw_cloud_registered',
                    help='registered cloud in the odometry frame (icp_odometry: /odom_local_scan_map)')
    ap.add_argument('--cloud-latest', action='store_true',
                    help='the cloud topic republishes the whole map: keep only the last message')
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('lio_bag_eval')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    rec = Recorder(node, args.odom_topic, args.cloud_topic, args.cloud_latest)
    stop = {'now': False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__('now', True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('now', True))
    while not stop['now']:
        rclpy.spin_once(node, timeout_sec=0.1)
        rec.status()
    summary(rec)
    draw(rec, args.out, args.title)
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
