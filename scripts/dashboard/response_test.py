#!/usr/bin/env python3
"""Directional drive response test, driven through the dashboard API.

For each axis it holds a capped command for DURATION seconds (refreshing at
10 Hz, like the browser pad), samples /api/status at ~20 Hz, and reports:
  * rise_s   time from the first command to odom twist reaching 50% of the command
  * dx, dy, dyaw   odometry displacement over the hold (wheels should be free)
  * cmd_rate  dashboard commands actually sent per second

Stdlib only; run from the host:
  TOKEN=... python3 scripts/dashboard/response_test.py [--url http://127.0.0.1:8080]

Robot must be suspended or on blocks. The test only sends capped commands and
always ends with a stop.
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request

AXES = [
    # name, vx, vy, wz (within dashboard caps: 0.10 m/s, 0.5 rad/s)
    ('forward',  0.10,  0.0,  0.0),
    ('back',    -0.10,  0.0,  0.0),
    ('left',     0.0,   0.10, 0.0),
    ('right',    0.0,  -0.10, 0.0),
    ('turn_L',   0.0,   0.0,  0.40),
    ('turn_R',   0.0,   0.0, -0.40),
]


def call(url, token, path, body=None, method='POST', timeout=2.0):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url + path, data=data, method=method,
                                 headers={'X-Token': token, 'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read() or b'{}')
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b'{}')


def status(url, token):
    return call(url, token, '/api/status', method='GET')[1]


def run_axis(url, token, name, vx, vy, wz, duration, rate_hz=10.0, sample_hz=20.0):
    s0 = status(url, token)
    o0 = s0['odom'] or {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'vx': 0.0, 'wz': 0.0}
    cmd = {'vx': vx, 'vy': vy, 'wz': wz}
    target = max(abs(vx), abs(vy)) if (vx or vy) else abs(wz)
    t0 = time.monotonic()
    next_send = t0
    next_sample = t0
    sent = 0
    rise = None
    samples = []
    while True:
        now = time.monotonic()
        if now - t0 >= duration:
            break
        if now >= next_send:
            code, _ = call(url, token, '/api/drive', cmd)
            if code == 200:
                sent += 1
            next_send += 1.0 / rate_hz
        if now >= next_sample:
            st = status(url, token)
            o = st.get('odom')
            if o:
                samples.append((now - t0, o))
                measured = o['vx'] if vx else (o['vy'] if vy else o['wz'])
                if rise is None and target and abs(measured) >= 0.5 * target:
                    rise = now - t0
            next_sample += 1.0 / sample_hz
        time.sleep(0.005)
    call(url, token, '/api/stop')
    time.sleep(0.6)  # let the stop and the controller watchdog settle
    o1 = (status(url, token).get('odom') or o0)
    return {
        'axis': name,
        'cmd': cmd,
        'sent': sent,
        'cmd_rate_hz': round(sent / duration, 1),
        'rise_s': None if rise is None else round(rise, 3),
        'dx': round(o1['x'] - o0['x'], 3),
        'dy': round(o1['y'] - o0['y'], 3),
        'dyaw_deg': round(math.degrees(o1['yaw'] - o0['yaw']), 1),
        'expected_dx': round(vx * duration, 3),
        'expected_dy': round(vy * duration, 3),
        'expected_dyaw_deg': round(math.degrees(wz * duration), 1),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='http://127.0.0.1:8080')
    ap.add_argument('--duration', type=float, default=1.0)
    ap.add_argument('--out', default='')
    args = ap.parse_args()
    token = os.environ.get('TOKEN', '')
    if not token:
        sys.exit('set TOKEN to the dashboard token')

    st = status(args.url, token)
    if not st.get('odom'):
        sys.exit('no /odom/wheel yet: start the STM32 stack first')
    if st.get('estop'):
        sys.exit('e-stop engaged: reset it first')

    results = []
    for name, vx, vy, wz in AXES:
        r = run_axis(args.url, token, name, vx, vy, wz, args.duration)
        results.append(r)
        print(json.dumps(r))
        time.sleep(0.5)

    print('\naxis       rise_s  cmd_hz   dx(meas/exp)        dy(meas/exp)        dyaw_deg(meas/exp)')
    for r in results:
        print(f"{r['axis']:<10} {str(r['rise_s']):>6}  {r['cmd_rate_hz']:>6}   "
              f"{r['dx']:+.3f}/{r['expected_dx']:+.3f}      "
              f"{r['dy']:+.3f}/{r['expected_dy']:+.3f}      "
              f"{r['dyaw_deg']:+6.1f}/{r['expected_dyaw_deg']:+.1f}")
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(results, f, indent=2)
        print(f'\nwrote {args.out}')


if __name__ == '__main__':
    main()
