#!/usr/bin/env python3
"""Startup and health board: prints ON or OFF for each part of the robot, refreshed periodically.

  python3 scripts/systems_watch.py              # refresh every 5 s until Ctrl-C
  python3 scripts/systems_watch.py --interval 2
  python3 scripts/systems_watch.py --once       # one snapshot; exit 1 if any required part is OFF

Checks the containers, the USB and serial devices, the LiDAR network link, the Zenoh router, the
dashboard API, and each data stream from the dashboard's own counters. Stdlib only: runs on the
host or in a container. Reads the VIEW token (read-only), never the control token.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime

CONTAINERS = [  # (name, required)
    ('intel-sys-zenohd', True),
    ('robot-description', True),
    ('robot-hardware', True),
    ('robot-dashboard', True),
]
LIDAR_IP = '192.168.1.2'
STREAMS = [  # (dashboard sensor key, label, required)
    ('lidar', 'LiDAR points (/lidar/points_raw)', True),
    ('imu', 'STM32 IMU (/imu_raw)', False),
    ('odom', 'wheel odometry (/odom/wheel)', False),
    ('set_motor', 'motor commands (/set_motor)', False),
    ('camera', 'RealSense colour (/camera/image_raw)', True),
    ('depth', 'RealSense depth (/camera/depth/image_raw)', True),
]
VIEW_TOKEN_FILE = os.path.expanduser('~/.config/robot-dashboard/view_token')
GREEN, RED, DIM, RESET = '\033[32m', '\033[31m', '\033[2m', '\033[0m'


def run(cmd, timeout=6):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except (OSError, subprocess.SubprocessError):
        return ''


def dashboard_base():
    ts = run(['tailscale', 'ip', '-4']).split()
    host = os.environ.get('DASHBOARD_LINK_HOST') or (ts[0] if ts else '127.0.0.1')
    return f"http://{host}:{os.environ.get('DASHBOARD_PORT', '8080')}"


def fetch_status(base, token):
    req = urllib.request.Request(f'{base}/api/status', headers={'X-Token': token})
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, None
    except (urllib.error.URLError, OSError, ValueError):
        return None, None


def check_camera_preview(base, token):
    req = urllib.request.Request(f'{base}/api/camera.jpg', headers={'X-Token': token})
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            return r.status == 200 and len(r.read(4)) > 0
    except (urllib.error.URLError, OSError):
        return False


def snapshot():
    """List of (section, label, on: bool|None, required, detail). None means informational."""
    rows = []
    running = set(run(['docker', 'ps', '--format', '{{.Names}}']).split())
    for name, req in CONTAINERS:
        rows.append(('containers', name, name in running, req, ''))

    rows.append(('devices', 'STM32 serial device /dev/rrc', os.path.exists('/dev/rrc'), False, ''))
    rows.append(('devices', 'RealSense USB 8086:0b3a', '8086:0b3a' in run(['lsusb']), True, ''))
    rows.append(('devices', 'camera node /dev/video0', os.path.exists('/dev/video0'), True, ''))
    ping_ok = subprocess.run(['ping', '-c', '1', '-W', '1', LIDAR_IP], capture_output=True).returncode == 0
    rows.append(('devices', f'LiDAR reachable {LIDAR_IP}', ping_ok, True, ''))

    listening = '127.0.0.1:7447' in run(['ss', '-ltn'])
    rows.append(('network', 'Zenoh router 127.0.0.1:7447', listening, True, ''))

    base = dashboard_base()
    token = ''
    try:
        with open(VIEW_TOKEN_FILE) as f:
            token = f.read().strip()
    except OSError:
        pass
    if not token:
        rows.append(('dashboard', 'view token file', False, True, f'missing {VIEW_TOKEN_FILE}'))
        return rows, base

    code, st = fetch_status(base, token)
    rows.append(('dashboard', f'API {base}', code == 200, True, f'HTTP {code}' if code else 'no answer'))
    if st is None:
        return rows, base

    for key, label, req in STREAMS:
        s = st['sensors'][key]
        age = 'no data yet' if s['age_s'] is None else f"{s['hz']:.1f} Hz, {s['age_s']:.1f} s ago"
        rows.append(('streams', label, not s['stale'], req, age))
    rows.append(('streams', 'LiDAR TF to base_footprint', bool(st.get('tf_lidar')), True, ''))
    rows.append(('streams', 'camera intrinsics (camera_info)', bool(st['camera'].get('intrinsics')), True, ''))
    rows.append(('streams', 'camera preview frame', check_camera_preview(base, token), False, ''))

    if 'servo' not in st:
        rows.append(('servo', 'dashboard build', False, False, 'older than the servo code: restart it'))
        return rows, base
    sv = st['servo']
    rows.append(('servo', f"tag servo mode {sv['mode']}", None, False, f"state {sv['state']}"))
    tg = st.get('tag')
    if tg:
        rows.append(('servo', f"ArUco ID {tg['id']} seen", tg['age_s'] is not None and tg['age_s'] < 1.0, False,
                     f"z {tg['z_m']:.2f} m, dict {tg['dict']}"))
    else:
        rows.append(('servo', 'ArUco tag seen', False, False, 'no detection yet'))
    rows.append(('servo', f"e-stop", None, False, 'ENGAGED' if st['estop'] else 'armed'))
    return rows, base


def render(rows, base, interval):
    lines = [f"robot systems  {datetime.now().strftime('%H:%M:%S')}   dashboard {base}   refresh {interval:g}s",
             '']
    section = None
    for sec, label, on, req, detail in rows:
        if sec != section:
            section = sec
            lines.append(f'  [{sec}]')
        if on is None:
            state = f'{DIM}INFO{RESET}'
        elif on:
            state = f'{GREEN}ON {RESET}'
        else:
            state = f"{RED}OFF{RESET}" if req else f'{DIM}off{RESET}'
        tag = '' if req else f'{DIM} (optional){RESET}'
        lines.append(f'    {state}  {label:<46}{detail}{tag}')
    required_off = [r[1] for r in rows if r[2] is False and r[3]]
    lines.append('')
    lines.append(f'  {len(required_off)} required part(s) OFF' + (': ' + ', '.join(required_off) if required_off else ''))
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interval', type=float, default=5.0, help='seconds between refreshes')
    ap.add_argument('--once', action='store_true', help='print one snapshot and exit')
    args = ap.parse_args()
    if shutil.which('docker') is None:
        print('docker not found on PATH', file=sys.stderr)
    tty = sys.stdout.isatty()
    try:
        while True:
            rows, base = snapshot()
            out = render(rows, base, args.interval)
            if args.once:
                print(out)
                required_off = [r for r in rows if r[2] is False and r[3]]
                sys.exit(1 if required_off else 0)
            if tty:
                sys.stdout.write('\033[H\033[2J')
            print(out, flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    main()
