#!/usr/bin/env python3
"""Robot status, manual drive, and autonomous exploration dashboard (stdlib HTTP + rclpy).

This node is the ONLY publisher of /cmd_vel while it runs. Three sources can drive:
  * e-stop      : publishes zero at TICK_HZ until reset (overrides everything)
  * autonomy    : reactive explorer (scripts/autonomy/explorer.py), modes off|dry|live
  * manual pad  : capped dead-man drive, refused while autonomy is armed (dry or live)
When none is active the node stays silent, so nothing fights Nav2 for the topic.

Safety model
  * Manual speeds are hard-capped (MAX_LIN, MAX_ANG); requests over the cap are rejected.
  * A manual drive command expires after DEADMAN_S without a refresh, then zeros are sent.
  * Autonomy live needs a fresh LiDAR, an explicit {"confirm": "TETHERED"}, and no e-stop.
  * Autonomy stops (zeros) on: e-stop, LiDAR silence > stale_s, executor failure, shutdown.
  * The mecanum controller also zeroes the wheels if /cmd_vel goes silent for 0.5 s.
  * Every API call needs the token (DASHBOARD_TOKEN, or generated at start).
  * E-stop is software only; keep the physical stop within reach.

Env
  DASHBOARD_HOST  bind address (default: Tailscale IP from the wrapper, else 127.0.0.1)
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
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from intel_sys_interfaces.msg import MotorsState
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import UInt16

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'autonomy'))
from explorer import Config as ExplorerConfig  # noqa: E402
from explorer import Explorer  # noqa: E402

MAX_LIN = 0.10      # m/s, hard cap for manual vx and vy
MAX_ANG = 0.50      # rad/s, hard cap for manual wz
DEADMAN_S = 0.3     # manual drive expires after this without a refresh
TICK_HZ = 10.0
RATE_WINDOW_S = 2.0
LOG_WARN = 30       # rcl_interfaces/Log levels
LOG_ERROR = 40

# LiDAR sector geometry, in base_footprint (x forward, y left, z up)
BODY_X = 0.20       # chassis half-length ignored
BODY_Y = 0.16       # chassis half-width ignored
CORRIDOR_HALF = 0.20    # half-width of the forward and rear corridors
SIDE_X = 0.35           # half-length of the side sectors
MIN_Z, MAX_Z = 0.05, 0.60
MIN_POINTS = 3          # use the 3rd-closest return, not a single stray point
LIDAR_TOPIC = '/lidar/points_raw'

TOPICS = {
    'imu': '/imu_raw',
    'odom': '/odom/wheel',
    'set_motor': '/set_motor',
    'lidar': LIDAR_TOPIC,
    'camera': '/camera/image_raw',
}


def clamp(v, lim):
    return max(-lim, min(lim, v))


def nearest(dists):
    """3rd-closest distance, or inf if fewer than MIN_POINTS returns."""
    if dists.size < MIN_POINTS:
        return math.inf
    return float(np.sort(dists)[MIN_POINTS - 1])


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.stamps = {k: collections.deque(maxlen=500) for k in TOPICS}
        self.warnings = collections.deque(maxlen=40)
        self.odom = None           # (x, y, yaw, vx, vy, wz)
        self.battery_mv = None
        self.drive = (0.0, 0.0, 0.0)
        self.drive_t = 0.0
        self.estop = False
        self.zero_ticks = 0        # explicit zeros left after a stop
        self.moving = False        # last published command was nonzero
        self.started = time.monotonic()
        self.commands_sent = 0
        self.rejections = 0
        self.spin_error = None
        # autonomy
        self.ap_mode = 'off'       # off | dry | live
        self.ap_state = 'IDLE'
        self.ap_cmd = (0.0, 0.0)   # (vx, wz) of the last decision
        self.dists = {'front': math.inf, 'rear': math.inf, 'left': math.inf, 'right': math.inf}
        self.lidar_t = None        # monotonic time of last usable cloud

    def mark(self, key):
        with self.lock:
            self.stamps[key].append(time.monotonic())

    def rate(self, key, now):
        with self.lock:
            n = sum(1 for t in self.stamps[key] if now - t <= RATE_WINDOW_S)
        return round(n / RATE_WINDOW_S, 1)

    def manual_command(self, now):
        """Manual pad output for this tick, or None when nothing should be sent."""
        zero = (0.0, 0.0, 0.0)
        with self.lock:
            if self.drive != zero and now - self.drive_t <= DEADMAN_S:
                self.moving = True
                return self.drive
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
        self.explorer = Explorer(ExplorerConfig())
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Imu, TOPICS['imu'], lambda m: self._safe(lambda: self.s.mark('imu')), qos_profile_sensor_data)
        self.create_subscription(Odometry, TOPICS['odom'], lambda m: self._safe(lambda: self._odom(m)), 10)
        self.create_subscription(MotorsState, TOPICS['set_motor'], lambda m: self._safe(lambda: self.s.mark('set_motor')), 10)
        self.create_subscription(PointCloud2, LIDAR_TOPIC, lambda m: self._safe(lambda: self._lidar(m)), qos_profile_sensor_data)
        self.create_subscription(Image, TOPICS['camera'], lambda m: self._safe(lambda: self.s.mark('camera')), qos_profile_sensor_data)
        self.create_subscription(UInt16, '/battery', lambda m: self._safe(lambda: self._battery(m)), 10)
        self.create_subscription(Log, '/rosout', lambda m: self._safe(lambda: self._log(m)), 100)
        self.create_timer(1.0 / TICK_HZ, lambda: self._safe(self._tick))

    # ---- helpers ----------------------------------------------------------
    def _safe(self, fn):
        """Run a callback; log and continue on error so one bad message cannot kill the executor."""
        try:
            fn()
        except Exception as e:  # noqa: BLE001
            with self.s.lock:
                self.s.warnings.append({'t': round(time.time(), 1), 'level': 'ERROR', 'node': 'dashboard',
                                        'msg': f'callback error: {e!r}'[:200]})
            self.get_logger().error(f'callback error: {e!r}\n{traceback.format_exc()}')

    # ---- callbacks --------------------------------------------------------
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
        # rcl log levels are numeric; Log.WARN is bytes in this message package, do not compare to it.
        level = int(msg.level)
        if level >= LOG_WARN:
            with self.s.lock:
                self.s.warnings.append({'t': round(time.time(), 1),
                                        'level': 'ERROR' if level >= LOG_ERROR else 'WARN',
                                        'node': msg.name, 'msg': msg.msg[:200]})

    def _lidar(self, msg):
        self.s.mark('lidar')
        try:
            tf = self.tf_buffer.lookup_transform('base_footprint', msg.header.frame_id,
                                                 rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1))
        except Exception as e:  # noqa: BLE001
            self._warn_once(f'no transform base_footprint<-{msg.header.frame_id}: {e}')
            return
        pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)
        if xyz.size == 0:
            return
        xyz = _to_frame(xyz, tf)
        z = xyz[:, 2]
        keep = (z > MIN_Z) & (z < MAX_Z) & ~((np.abs(xyz[:, 0]) < BODY_X) & (np.abs(xyz[:, 1]) < BODY_Y))
        x, y = xyz[keep, 0], xyz[keep, 1]
        dists = {
            'front': nearest(x[(x > BODY_X) & (np.abs(y) < CORRIDOR_HALF)]),
            'rear': nearest(-x[(x < -BODY_X) & (np.abs(y) < CORRIDOR_HALF)]),
            'left': nearest(y[(y > BODY_Y) & (np.abs(x) < SIDE_X)]),
            'right': nearest(-y[(y < -BODY_Y) & (np.abs(x) < SIDE_X)]),
        }
        with self.s.lock:
            self.s.dists = dists
            self.s.lidar_t = time.monotonic()

    def _warn_once(self, text):
        now = time.monotonic()
        if now - getattr(self, '_last_warn', 0.0) > 2.0:
            self._last_warn = now
            self.get_logger().warn(text)

    # ---- control tick -----------------------------------------------------
    def _publish(self, vx, wz, vy=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(t)
        with self.s.lock:
            self.s.commands_sent += 1

    def _tick(self):
        now = time.monotonic()
        with self.s.lock:
            estop = self.s.estop
            mode = self.s.ap_mode
            dists = dict(self.s.dists)
            lidar_t = self.s.lidar_t

        if estop:
            with self.s.lock:
                self.s.moving = False
                self.s.ap_state = 'ESTOP'
                self.s.ap_cmd = (0.0, 0.0)
            self._publish(0.0, 0.0)
            return

        if mode in ('live', 'dry'):
            age = None if lidar_t is None else now - lidar_t
            state, vx, wz = self.explorer.decide(now, dists['front'], dists['rear'],
                                                 dists['left'], dists['right'], age)
            with self.s.lock:
                self.s.ap_state = state
                self.s.ap_cmd = (vx, wz)
            if mode == 'live':
                self._publish(vx, wz)
                with self.s.lock:
                    self.s.moving = (vx != 0.0 or wz != 0.0)
            return

        # manual pad
        cmd = self.s.manual_command(now)
        if cmd is None:
            return
        vx, vy, wz = cmd
        self._publish(vx, wz, vy)

    def set_autonomy(self, mode):
        with self.s.lock:
            self.s.ap_mode = mode
            self.s.ap_state = 'IDLE'
            self.s.ap_cmd = (0.0, 0.0)
        self.explorer.reset()


def _to_frame(xyz, tf):
    t = tf.transform.translation
    q = tf.transform.rotation
    R = _quat_to_matrix(q.x, q.y, q.z, q.w)
    return xyz @ R.T + np.array([t.x, t.y, t.z])


def _quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def make_handler(state, token, node):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'RobotDashboard/2.0'

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

        def _body(self):
            length = int(self.headers.get('Content-Length') or 0)
            return json.loads(self.rfile.read(length) or b'{}')

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
            try:
                body = self._body()
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
                    if state.ap_mode != 'off':
                        state.rejections += 1
                        return self._send(409, {'error': f'autonomy is {state.ap_mode}; set it off first'})
                    state.drive = (vx, vy, wz)
                    state.drive_t = time.monotonic()
                return self._send(200, {'ok': True})

            if url.path == '/api/stop':
                with state.lock:
                    state.drive = (0.0, 0.0, 0.0)
                    state.drive_t = time.monotonic()
                return self._send(200, {'ok': True})

            if url.path == '/api/autonomy':
                mode = str(body.get('mode', 'off'))
                if mode not in ('off', 'dry', 'live'):
                    return self._send(400, {'error': 'mode must be off, dry or live'})
                with state.lock:
                    estop, spin_error = state.estop, state.spin_error
                    lidar_age = None if state.lidar_t is None else time.monotonic() - state.lidar_t
                if spin_error:
                    return self._send(503, {'error': f'ROS executor down: {spin_error}'})
                if mode == 'live':
                    if body.get('confirm') != 'TETHERED':
                        return self._send(400, {'error': 'live needs {"mode":"live","confirm":"TETHERED"}'})
                    if estop:
                        return self._send(409, {'error': 'e-stop engaged; reset first'})
                    if lidar_age is None or lidar_age > 0.5:
                        return self._send(409, {'error': 'no fresh LiDAR data'})
                node.set_autonomy(mode)
                print(f'[dashboard] autonomy -> {mode}', flush=True)
                return self._send(200, {'ok': True, 'mode': mode})

            if url.path == '/api/estop':
                with state.lock:
                    state.estop = True
                    state.drive = (0.0, 0.0, 0.0)
                    state.ap_mode = 'off'
                print('[dashboard] E-STOP engaged', flush=True)
                return self._send(200, {'ok': True, 'estop': True})

            if url.path == '/api/reset':
                with state.lock:
                    state.estop = False
                node.set_autonomy('off')
                print('[dashboard] e-stop reset, autonomy off', flush=True)
                return self._send(200, {'ok': True, 'estop': False})

            self._send(404, {'error': 'not found'})

    return Handler


def status_payload(state):
    now = time.monotonic()
    with state.lock:
        odom = state.odom
        dists = {k: (None if math.isinf(v) else round(v, 3)) for k, v in state.dists.items()}
        payload = {
            'uptime_s': round(now - state.started, 1),
            'estop': state.estop,
            'spin_error': state.spin_error,
            'drive': {'vx': state.drive[0], 'vy': state.drive[1], 'wz': state.drive[2],
                      'age_s': round(now - state.drive_t, 2)},
            'caps': {'lin_m_s': MAX_LIN, 'ang_rad_s': MAX_ANG, 'deadman_s': DEADMAN_S},
            'autonomy': {
                'mode': state.ap_mode,
                'state': state.ap_state,
                'vx': state.ap_cmd[0],
                'wz': state.ap_cmd[1],
                'dists_m': dists,
                'lidar_age_s': None if state.lidar_t is None else round(now - state.lidar_t, 2),
            },
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
    host = os.environ.get('DASHBOARD_HOST', '127.0.0.1')
    port = int(os.environ.get('DASHBOARD_PORT', '8080'))
    token = os.environ.get('DASHBOARD_TOKEN') or secrets.token_urlsafe(16)

    state = State()
    rclpy.init()
    node = DashboardNode(state)

    def _spin():
        try:
            rclpy.spin(node)
        except Exception as e:  # noqa: BLE001
            with state.lock:
                state.spin_error = repr(e)
                state.ap_mode = 'off'
            print(f'[dashboard] ROS executor died: {e!r}', flush=True)
    threading.Thread(target=_spin, daemon=True).start()

    def _on_term(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_term)

    httpd = ThreadingHTTPServer((host, port), make_handler(state, token, node))
    print(f'[dashboard] serving on http://{host}:{port}', flush=True)
    print(f'[dashboard] token: {token}', flush=True)
    print(f'[dashboard] manual caps: lin {MAX_LIN} m/s, ang {MAX_ANG} rad/s; autonomy starts off', flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        # Zero the wheels before the ROS context goes away: e-stop makes the tick publish zeros.
        with state.lock:
            state.estop = True
            state.ap_mode = 'off'
        time.sleep(0.3)
        httpd.server_close()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
        print('[dashboard] stopped', flush=True)


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
  .ok{color:var(--ok)} .warn{color:var(--warn)} .bad{color:var(--bad)}
  .ap{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
  .ap button.on{outline:2px solid var(--ok)}
</style></head>
<body><main>
  <div class="card" id="auth"><h2>Access</h2>
    <input id="tok" type="password" placeholder="token" style="width:70%;padding:8px">
    <button onclick="saveTok()">Connect</button>
  </div>

  <div class="card"><h2>E-stop</h2>
    <button class="estop" onclick="post('/api/estop')">EMERGENCY STOP</button>
    <p id="estopState" style="margin:8px 0 0"></p>
    <button onclick="post('/api/reset')" style="margin-top:8px">Reset (clears e-stop, autonomy off)</button>
  </div>

  <div class="card"><h2>Autonomy: explore (forward, slow, reverse, turn)</h2>
    <div class="ap">
      <button id="apOff" onclick="setAp('off')">Off</button>
      <button id="apDry" onclick="setAp('dry')">Dry run (no motion)</button>
      <button id="apLive" onclick="setAp('live')">Live (tethered)</button>
    </div>
    <p id="apState" style="margin:8px 0 0"></p>
    <div class="grid" id="apDist" style="margin-top:8px"></div>
    <p style="opacity:.7;margin:8px 0 0">Live drives the wheels at up to 0.10 m/s. Keep the tether taut and your hand near the e-stop. Manual drive is disabled while autonomy is armed.</p>
  </div>

  <div class="card"><h2>Manual drive (dead-man: hold to move)</h2>
    <div class="pad">
      <span></span><button data-vx="0.1" data-vy="0" data-wz="0">Forward</button><span></span>
      <button data-vx="0" data-vy="0.1" data-wz="0">Left</button>
      <button onclick="post('/api/stop')">Stop</button>
      <button data-vx="0" data-vy="-0.1" data-wz="0">Right</button>
      <button data-vx="0" data-vy="0" data-wz="0.4">Turn L</button>
      <button data-vx="-0.1" data-vy="0" data-wz="0">Back</button>
      <button data-vx="0" data-vy="0" data-wz="-0.4">Turn R</button>
    </div>
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
async function setAp(mode){
  let body = {mode};
  if (mode === 'live'){
    // The explicit TETHERED confirmation is the gate; the server rejects live without it.
    body.confirm = 'TETHERED';
  }
  const r = await post('/api/autonomy', body);
  if (r.error) alert(r.error);
}
let timer = null;
function startDrive(btn){
  const v = {vx:+btn.dataset.vx, vy:+btn.dataset.vy, wz:+btn.dataset.wz};
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
  const o = s.odom, a = s.autonomy;
  document.getElementById('estopState').innerHTML = s.estop ? '<span class="bad">E-STOP ENGAGED</span>' : '<span class="ok">armed</span>';
  ['Off','Dry','Live'].forEach(n => document.getElementById('ap'+n).classList.toggle('on', a.mode === n.toLowerCase()));
  document.getElementById('apState').innerHTML =
    `mode <b>${a.mode}</b> · state <b>${a.state}</b> · cmd vx ${fmt(a.vx,3)} m/s, wz ${fmt(a.wz,2)} rad/s` +
    (a.mode === 'dry' ? ' <span class="warn">(dry: not publishing)</span>' : '');
  const d = a.dists_m;
  document.getElementById('apDist').innerHTML = [
    cell('front m', fmt(d.front,2)), cell('rear m', fmt(d.rear,2)),
    cell('left m', fmt(d.left,2)), cell('right m', fmt(d.right,2)),
    cell('lidar age s', fmt(a.lidar_age_s,2), (a.lidar_age_s===null || a.lidar_age_s>0.5)?'bad':''),
  ].join('');
  document.getElementById('kv').innerHTML = [
    cell('uptime s', fmt(s.uptime_s,0)),
    cell('battery mV', fmt(s.battery_mv,0)),
    cell('odom x m', o?fmt(o.x,3):'—'),
    cell('odom y m', o?fmt(o.y,3):'—'),
    cell('yaw rad', o?fmt(o.yaw,3):'—'),
    cell('commands sent', s.commands_sent),
    cell('rejected', s.rejections, s.rejections?'warn':''),
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
