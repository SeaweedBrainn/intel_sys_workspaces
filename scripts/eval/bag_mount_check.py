#!/usr/bin/env python3
"""Work out how the Unitree L2 was mounted in a recorded bag.

Uses the L2's built-in IMU (gravity direction while still) and the point cloud
(floor plane and where the returns are) in the sensor's own frame:

    python3 scripts/eval/bag_mount_check.py bags/<bag_dir> [--seconds 5]

Reports, in the L2 frame:
  - gravity (the "down" direction) from the first still stretch of /lidar/imu
  - the tilt of the L2's z axis (its dome axis) from straight up
  - the floor plane normal from the lowest cloud points, as a cross-check
  - how much of the motion in the bag looks like driving (gyro / accel spread)
"""
import argparse
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def open_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    return reader


def read_imu(path, seconds):
    reader = open_bag(path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/lidar/imu']))
    acc, gyr, t = [], [], []
    frame = None
    while reader.has_next():
        _, data, stamp = reader.read_next()
        m = deserialize_message(data, Imu)
        frame = m.header.frame_id
        acc.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        gyr.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        t.append(stamp * 1e-9)
    t = np.array(t) - t[0]
    return frame, np.array(acc), np.array(gyr), t


def read_clouds(path, n, start_s=1.0):
    reader = open_bag(path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/lidar/points_raw']))
    clouds, t0, frame, fields = [], None, None, None
    while reader.has_next() and len(clouds) < n:
        _, data, stamp = reader.read_next()
        t0 = stamp if t0 is None else t0
        if (stamp - t0) * 1e-9 < start_s:
            continue
        m = deserialize_message(data, PointCloud2)
        frame, fields = m.header.frame_id, [f.name for f in m.fields]
        a = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
        clouds.append(np.stack([a['x'], a['y'], a['z']], -1).astype(float))
    return frame, fields, clouds


def fit_plane(p):
    c = p.mean(0)
    _, _, vt = np.linalg.svd(p - c)
    nrm = vt[2]
    return nrm, c, np.abs((p - c) @ nrm)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--seconds', type=float, default=5.0, help='still stretch at the start used for gravity')
    args = ap.parse_args()

    frame, acc, gyr, t = read_imu(args.bag, args.seconds)
    rate = len(t) / t[-1]
    still = t < args.seconds
    g = acc[still].mean(0)
    gn = np.linalg.norm(g)
    print(f'IMU frame "{frame}", {len(t)} samples, {rate:.0f} Hz, {t[-1]:.0f} s')
    print(f'  first {args.seconds:.0f} s: mean accel {np.round(g, 3)}  |a| = {gn:.3f}  '
          f'(units: {"m/s^2" if gn > 5 else "g"})')
    print(f'  first {args.seconds:.0f} s: gyro std {np.round(gyr[still].std(0), 4)} rad/s, '
          f'accel std {np.round(acc[still].std(0), 3)}')
    # An accelerometer at rest reads the reaction to gravity: it points UP.
    up = g / gn
    tilt = math.degrees(math.acos(np.clip(up[2], -1, 1)))
    print(f'  "up" in the L2 frame: {np.round(up, 3)}')
    print(f'  L2 z axis (dome axis) is {tilt:.1f} deg from straight up')
    # Activity over the whole bag, per 10 s window
    win = []
    for w0 in np.arange(0, t[-1], 10.0):
        s = (t >= w0) & (t < w0 + 10)
        if s.sum() > 10:
            win.append((w0, np.linalg.norm(gyr[s], axis=1).mean(), np.linalg.norm(acc[s] - acc[s].mean(0), axis=1).mean()))
    print('  motion per 10 s window (mean |gyro| rad/s, mean |accel - mean| m/s^2):')
    print('   ' + '  '.join(f'{w0:4.0f}s:{gm:.2f}/{am:.2f}' for w0, gm, am in win))

    cframe, fields, clouds = read_clouds(args.bag, 10)
    p = np.concatenate(clouds)
    r = np.linalg.norm(p, axis=1)
    p = p[r > 0.3]
    print(f'Cloud frame "{cframe}", fields {fields}, {len(p) // len(clouds)} points/scan (first 10 scans)')
    # Directions of the returns relative to the L2 axes
    u = p / np.linalg.norm(p, axis=1, keepdims=True)
    print('  share of returns by direction in L2 frame: '
          + '  '.join(f'{ax}{s}: {np.mean(u[:, i] * (1 if s == "+" else -1) > 0.5):.0%}'
                      for i, ax in enumerate('xyz') for s in '+-'))
    # Floor: points lowest along the gravity "up" direction
    h = p @ up
    floor = p[h < np.percentile(h, 15)]
    nrm, c, res = fit_plane(floor)
    if nrm @ up < 0:
        nrm = -nrm
    ang = math.degrees(math.acos(np.clip(nrm @ up, -1, 1)))
    print(f'  floor plane (lowest 15% along up): normal {np.round(nrm, 3)}, {ang:.1f} deg from IMU "up", '
          f'L2 is {-(c @ nrm):.2f} m above it, fit rms {np.sqrt((res ** 2).mean()):.3f} m')
    return 0


if __name__ == '__main__':
    sys.exit(main())
