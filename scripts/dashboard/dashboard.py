#!/usr/bin/env python3
"""Robot status and control dashboard (stdlib HTTP server + rclpy).

Serves a single page with live status (topic rates, odometry, battery,
recent warnings) and a capped, dead-man-switch drive pad with an e-stop.

Safety model
  * Drive speeds are hard-capped here (MAX_LIN, MAX_ANG); requests above the
    cap are rejected, not clamped.
  * A drive command expires after DEADMAN_S without a fresh request, and the
    node then publishes zero. The browser re-sends every ~100 ms while held.
  * The node publishes to /cmd_vel only while driving, stopping, or in e-stop.
    It never publishes while idle, so Nav2 keeps sole control of the topic.
  * E-stop publishes zero at TICK_HZ until /api/reset. This is software only;
    it is not a hardware e-stop.
  * Every API call needs the token (DASHBOARD_TOKEN, or generated at start).

Env
  DASHBOARD_HOST  bind address (default 0.0.0.0; reach it over Tailscale)
  DASHBOARD_PORT  port (default 8080)
  DASHBOARD_TOKEN fixed token; generated and printed if unset
"""
import collections
import hmac
import json
import math
import os
import secrets
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import UInt16
from intel_sys_interfaces.msg import MotorsState

LOG_WARN = 30       # rcl_interfaces/Log levels
LOG_ERROR = 40
MAX_LIN = 0.10      # m/s, hard cap for vx and vy
MAX_ANG = 0.50      # rad/s, hard cap for wz
DEADMAN_S = 0.3     # drive command expires after this without a refresh
TICK_HZ = 10.0
RATE_WINDOW_S = 2.0

# topic key -> absolute topic name used by the STM32 bridge and sensors
TOPICS = {
    'imu': '/imu_raw',
    'odom': '/odom/wheel',
    'set_motor': '/set_motor',
    'lidar': '/lidar/points_raw',
    'camera': '/camera/image_raw',
}


def clamp(v, lim):
    return max(-lim, min(lim, v))


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.stamps = {k: collections.deque(maxlen=500) for k in TOPICS}
        self.warnings = collections.deque(maxlen=40)
        self.odom = None           # (x, y, yaw, vx, wz)
        self.battery_mv = None
        self.drive = (0.0, 0.0, 0.0)
        self.drive_t = 0.0         # monotonic time of last drive request
        self.estop = False
        self.zero_ticks = 0        # explicit zero publishes left after a stop
        self.moving = False        # whether the last published command was nonzero
        self.spin_error = None     # set if the ROS executor thread dies
        self.started = time.monotonic()
        self.commands_sent = 0
        self.rejections = 0

    def mark(self, key):
        with self.lock:
            self.stamps[key].append(time.monotonic())

    def rate(self, key, now):
        with self.lock:
            n = sum(1 for t in self.stamps[key] if now - t <= RATE_WINDOW_S)
        return round(n / RATE_WINDOW_S, 1)

    def command(self, now):
        """The Twist to publish this tick, or None if nothing should be sent."""
        zero = (0.0, 0.0, 0.0)
        with self.lock:
            if self.estop:
                self.moving = False
                return zero
            if self.drive != zero and now - self.drive_t <= DEADMAN_S:
                self.moving = True
                return self.drive
            # Stopped (by request) or expired. Key the zeros off what we last
            # published, not the drive value, so a stop request still gets sent.
            self.drive = zero
            if self.moving:
                self.moving = False
                self.zero_ticks = 3
            if self.zero_ticks > 0:
                self.zero_ticks -= 1
                return zero
            return None


class DashboardNode(Node):
    def __init__(self, state):
        super().__init__('robot_dashboard')
        self.s = state
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Imu, TOPICS['imu'], lambda m: self.s.mark('imu'), qos_profile_sensor_data)
        self.create_subscription(Odometry, TOPICS['odom'], self._odom, 10)
        self.create_subscription(MotorsState, TOPICS['set_motor'], lambda m: self.s.mark('set_motor'), 10)
        self.create_subscription(PointCloud2, TOPICS['lidar'], lambda m: self.s.mark('lidar'), qos_profile_sensor_data)
        self.create_subscription(Image, TOPICS['camera'], lambda m: self.s.mark('camera'), qos_profile_sensor_data)
        self.create_subscription(UInt16, '/battery', self._battery, 10)
        self.create_subscription(Log, '/rosout', self._log, 100)
        self.create_timer(1.0 / TICK_HZ, self._tick)

    def _odom(self, msg):
        self.s.mark('odom')
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        tw = msg.twist.twist
        with self.s.lock:
            self.s.odom = (round(p.x, 3), round(p.y, 3), round(yaw, 3),
                           round(tw.linear.x, 3), round(tw.linear.y, 3), round(tw.angular.z, 3))

    def _battery(self, msg):
        with self.s.lock:
            self.s.battery_mv = int(msg.data)

    def _log(self, msg):
        # rcl log levels are numeric: WARN=30, ERROR=40 (Log.WARN is bytes in this
        # message package, so do not compare against it).
        level = int(msg.level)
        if level >= LOG_WARN:
            with self.s.lock:
                self.s.warnings.append({'t': round(time.time(), 1),
                                        'level': 'ERROR' if level >= LOG_ERROR else 'WARN',
                                        'node': msg.name, 'msg': msg.msg[:200]})

    def _tick(self):
        now = time.monotonic()
        cmd = self.s.command(now)
        if cmd is None:
            return  # idle: do not touch /cmd_vel
        vx, vy, wz = cmd
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(t)
        with self.s.lock:
            self.s.commands_sent += 1


def make_handler(state, token):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'RobotDashboard/1.0'

        def log_message(self, fmt, *args):
            sys.stderr.write('[dashboard] %s - %s\n' % (self.address_string(), fmt % args))

        def _authed(self, query):
            supplied = self.headers.get('X-Token') or query.get('token', [''])[0]
            return hmac.compare_digest(supplied.encode(), token.encode())

        def _send(self, code, body, ctype='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            url = urlparse(self.path)
            if url.path == '/':
                return self._send(200, PAGE.encode(), 'text/html; charset=utf-8')
            if url.path == '/api/status':
                if not self._authed(parse_qs(url.query)):
                    return self._send(401, {'error': 'bad token'})
                return self._send(200, status_payload(state))
            self._send(404, {'error': 'not found'})

        def do_POST(self):
            url = urlparse(self.path)
            if not self._authed(parse_qs(url.query)):
                return self._send(401, {'error': 'bad token'})
            length = int(self.headers.get('Content-Length') or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except ValueError:
                return self._send(400, {'error': 'invalid json'})

            if url.path == '/api/drive':
                try:
                    vx, vy, wz = (float(body.get(k, 0.0)) for k in ('vx', 'vy', 'wz'))
                except (TypeError, ValueError):
                    return self._send(400, {'error': 'vx, vy, wz must be numbers'})
                if not all(math.isfinite(v) for v in (vx, vy, wz)):
                    return self._send(400, {'error': 'non-finite value'})
                if abs(vx) > MAX_LIN or abs(vy) > MAX_LIN or abs(wz) > MAX_ANG:
                    with state.lock:
                        state.rejections += 1
                    return self._send(400, {'error': f'over cap (lin {MAX_LIN} m/s, ang {MAX_ANG} rad/s)'})
                with state.lock:
                    if state.spin_error:
                        return self._send(503, {'error': f'ROS executor down: {state.spin_error}'})
                    if state.estop:
                        state.rejections += 1
                        return self._send(409, {'error': 'e-stop engaged; reset first'})
                    state.drive = (vx, vy, wz)
                    state.drive_t = time.monotonic()
                return self._send(200, {'ok': True})

            if url.path == '/api/stop':
                with state.lock:
                    state.drive = (0.0, 0.0, 0.0)
                    state.drive_t = time.monotonic()
                return self._send(200, {'ok': True})

            if url.path == '/api/estop':
                with state.lock:
                    state.estop = True
                    state.drive = (0.0, 0.0, 0.0)
                print('[dashboard] E-STOP engaged', flush=True)
                return self._send(200, {'ok': True, 'estop': True})

            if url.path == '/api/reset':
                with state.lock:
                    state.estop = False
                print('[dashboard] e-stop reset', flush=True)
                return self._send(200, {'ok': True, 'estop': False})

            self._send(404, {'error': 'not found'})

    return Handler


def status_payload(state):
    now = time.monotonic()
    with state.lock:
        odom = state.odom
        payload = {
            'uptime_s': round(now - state.started, 1),
            'estop': state.estop,
            'spin_error': state.spin_error,
            'drive': {'vx': state.drive[0], 'vy': state.drive[1], 'wz': state.drive[2],
                      'age_s': round(now - state.drive_t, 2)},
            'caps': {'lin_m_s': MAX_LIN, 'ang_rad_s': MAX_ANG, 'deadman_s': DEADMAN_S},
            'battery_mv': state.battery_mv,
            'odom': None if odom is None else {'x': odom[0], 'y': odom[1], 'yaw': odom[2],
                                              'vx': odom[3], 'vy': odom[4], 'wz': odom[5]},
            'commands_sent': state.commands_sent,
            'rejections': state.rejections,
            'warnings': list(state.warnings)[-20:],
        }
    payload['rates_hz'] = {k: state.rate(k, now) for k in TOPICS}
    return payload


def main():
    host = os.environ.get('DASHBOARD_HOST', '0.0.0.0')
    port = int(os.environ.get('DASHBOARD_PORT', '8080'))
    token = os.environ.get('DASHBOARD_TOKEN') or secrets.token_urlsafe(16)

    state = State()
    rclpy.init()
    node = DashboardNode(state)

    def _spin():
        # If the ROS executor dies, record it: otherwise /cmd_vel silently stops
        # updating and the page looks alive.
        try:
            rclpy.spin(node)
        except Exception as e:  # noqa: BLE001 - surface any executor failure
            with state.lock:
                state.spin_error = repr(e)
            print(f'[dashboard] ROS executor died: {e!r}', flush=True)
    threading.Thread(target=_spin, daemon=True).start()

    # docker stop sends SIGTERM: route it through the same clean-shutdown path as Ctrl-C.
    def _on_term(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_term)

    httpd = ThreadingHTTPServer((host, port), make_handler(state, token))
    print(f'[dashboard] serving on http://{host}:{port}', flush=True)
    print(f'[dashboard] token: {token}', flush=True)
    print(f'[dashboard] caps: lin {MAX_LIN} m/s, ang {MAX_ANG} rad/s, deadman {DEADMAN_S}s', flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        with state.lock:
            state.estop = True  # stop the wheels on shutdown
        time.sleep(0.3)
        httpd.server_close()
        node.destroy_node()
        rclpy.shutdown()


PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Dashboard</title>
<style>
  :root{color-scheme:light dark;--bg:#f6f7f9;--fg:#1b1f24;--card:#fff;--line:#d8dde3;--ok:#1a7f37;--warn:#9a6700;--bad:#cf222e}
  @media (prefers-color-scheme:dark){:root{--bg:#0d1117;--fg:#e6edf3;--card:#161b22;--line:#30363d}}
  body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
  main{padding:16px;max-width:900px;margin:auto;display:grid;gap:12px}
  .card{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:12px}
  h2{font-size:13px;text-transform:uppercase;letter-spacing:.05em;margin:0 0 8px;opacity:.7}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:8px}
  .kv b{display:block;font-size:18px}
  .kv span{opacity:.7;font-size:12px}
  button{font:inherit;border:1px solid var(--line);background:var(--card);color:var(--fg);border-radius:6px;padding:12px;min-height:44px;touch-action:none;user-select:none}
  .estop{background:var(--bad);color:#fff;border:0;font-weight:700;font-size:18px;width:100%;min-height:64px}
  .pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
  .log{font:12px ui-monospace,monospace;max-height:220px;overflow:auto;white-space:pre-wrap}
  .pill{padding:2px 8px;border-radius:99px;font-size:12px;background:var(--line)}
  .ok{color:var(--ok)} .warn{color:var(--warn)} .bad{color:var(--bad)}
  label{display:block;margin-top:8px}
</style></head>
<body><main>
  <div class="card" id="auth"><h2>Access</h2>
    <input id="tok" type="password" placeholder="token" style="width:70%;padding:8px">
    <button onclick="saveTok()">Connect</button>
  </div>

  <div class="card"><h2>E-stop</h2>
    <button class="estop" onclick="post('/api/estop')">EMERGENCY STOP</button>
    <p id="estopState" style="margin:8px 0 0"></p>
    <button onclick="post('/api/reset')" style="margin-top:8px">Reset e-stop</button>
  </div>

  <div class="card"><h2>Drive (dead-man: hold to move)</h2>
    <div class="pad">
      <span></span><button data-vx="0.1" data-vy="0" data-wz="0">Forward</button><span></span>
      <button data-vx="0" data-vy="0.1" data-wz="0">Left</button>
      <button onclick="post('/api/stop')">Stop</button>
      <button data-vx="0" data-vy="-0.1" data-wz="0">Right</button>
      <button data-vx="0" data-vy="0" data-wz="0.4">Turn L</button>
      <button data-vx="-0.1" data-vy="0" data-wz="0">Back</button>
      <button data-vx="0" data-vy="0" data-wz="-0.4">Turn R</button>
    </div>
    <label>Speed scale <input id="scale" type="range" min="0.2" max="1" step="0.1" value="1"> <span id="scaleVal">1.0</span> (max 0.10 m/s, 0.4 rad/s used by pad)</label>
  </div>

  <div class="card"><h2>Status</h2>
    <div class="grid" id="kv"></div>
    <p style="margin:8px 0 0;opacity:.7" id="rates"></p>
  </div>

  <div class="card"><h2>Recent warnings</h2><div class="log" id="log"></div></div>
</main>
<script>
const q = new URLSearchParams(location.search);
let TOKEN = q.get('token') || sessionStorage.getItem('robotTok') || '';
if (TOKEN) { sessionStorage.setItem('robotTok', TOKEN); document.getElementById('auth').hidden = true; }
function saveTok(){ TOKEN = document.getElementById('tok').value.trim(); sessionStorage.setItem('robotTok', TOKEN); document.getElementById('auth').hidden = true; }
const H = () => ({'Content-Type':'application/json','X-Token':TOKEN});
async function post(path, body){
  try { const r = await fetch(path,{method:'POST',headers:H(),body:JSON.stringify(body||{})}); return r.json(); }
  catch(e){ return {error:String(e)}; }
}
let scale = 1;
document.getElementById('scale').oninput = e => { scale = +e.target.value; document.getElementById('scaleVal').textContent = scale.toFixed(1); };
let timer = null;
function startDrive(btn){
  const v = {vx:+btn.dataset.vx*scale, vy:+btn.dataset.vy*scale, wz:+btn.dataset.wz*scale};
  const send = () => post('/api/drive', v);
  send(); clearInterval(timer); timer = setInterval(send, 100);
}
function stopDrive(){ clearInterval(timer); timer = null; post('/api/stop'); }
document.querySelectorAll('button[data-vx]').forEach(b => {
  b.addEventListener('pointerdown', e => { e.preventDefault(); startDrive(b); });
  ['pointerup','pointerleave','pointercancel'].forEach(ev => b.addEventListener(ev, stopDrive));
});
window.addEventListener('blur', stopDrive);
window.addEventListener('pagehide', stopDrive);

const fmt = (v, d=2) => (v === null || v === undefined) ? '—' : (typeof v === 'number' ? v.toFixed(d) : v);
function cell(label, value, cls=''){ return `<div class="kv"><b class="${cls}">${value}</b><span>${label}</span></div>`; }
async function refresh(){
  if (!TOKEN) return;
  let s;
  try {
    const r = await fetch('/api/status?token='+encodeURIComponent(TOKEN));
    if (r.status === 401){ document.getElementById('auth').hidden = false; return; }
    s = await r.json();
  } catch(e){ return; }
  const o = s.odom;
  document.getElementById('estopState').innerHTML = s.estop ? '<span class="bad">E-STOP ENGAGED</span>' : '<span class="ok">armed</span>';
  document.getElementById('kv').innerHTML = [
    cell('uptime s', fmt(s.uptime_s,0)),
    cell('battery mV', fmt(s.battery_mv,0)),
    cell('odom x m', o?fmt(o.x,3):'—'),
    cell('odom y m', o?fmt(o.y,3):'—'),
    cell('yaw rad', o?fmt(o.yaw,3):'—'),
    cell('odom vx m/s', o?fmt(o.vx,3):'—'),
    cell('commands sent', s.commands_sent),
    cell('rejected', s.rejections, s.rejections?'warn':''),
    cell('drive age s', fmt(s.drive.age_s,2)),
  ].join('');
  const r = s.rates_hz;
  document.getElementById('rates').textContent =
    `Hz  imu ${r.imu} · odom ${r.odom} · set_motor ${r.set_motor} · lidar ${r.lidar} · camera ${r.camera}`;
  document.getElementById('log').textContent = s.warnings.length
    ? s.warnings.map(w => `${new Date(w.t*1000).toLocaleTimeString()} ${w.level} ${w.node}: ${w.msg}`).join('\\n')
    : 'no warnings';
}
setInterval(refresh, 500); refresh();
</script></body></html>
"""


if __name__ == '__main__':
    main()
