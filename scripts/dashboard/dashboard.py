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
from waypoint import NavConfig, OccupancyMap, WaypointNav  # noqa: E402
from frames import WindowedCloud, align_rotation, tilt_deg  # noqa: E402

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
# The LiDAR's view (base_footprint +x per TF) points at the rover's physical rear: a +vx
# /cmd_vel drives away from what it sees. Autonomy decides in the LiDAR's frame, so its
# command is rotated 180 deg before publishing. Manual drive is not affected.
LIDAR_FACES_REAR = os.environ.get('DASHBOARD_LIDAR_FACES_REAR', '1') == '1'
LIDAR_TOPIC = '/lidar/points_raw'
MAX_WAYPOINTS = 50
MAX_COORD = 100.0  # m from the odometry origin
CLOUD_MAX = 4000       # points sent to the 3D view
CLOUD_RANGE = 20.0     # m, returns farther than this are not drawn
CLOUD_WINDOW_S = 60.0  # s: accumulated 3D points and 2D map cells expire after this
CLOUD_ADD = 1500       # points per scan kept for accumulation
CLOUD_BUFFER_POINTS = 1000000  # rolling buffer cap (about 13 MB), oldest scans dropped first
LIDAR_IMU_TOPIC = '/lidar/imu'
IMU_ALPHA = 0.002      # gravity low-pass per LiDAR IMU sample (about 2 s at 250 Hz)
IMU_G_MIN, IMU_G_MAX = 8.0, 11.5   # only samples near 1 g update gravity (rejects motion)

TOPICS = {
    'imu': '/imu_raw',
    'odom': '/odom/wheel',
    'set_motor': '/set_motor',
    'lidar': LIDAR_TOPIC,
    'camera': '/camera/image_raw',
}


def decimate(points, max_n):
    """Evenly thin an (N, 3) array to at most max_n rows, keeping its spread."""
    step = max(1, points.shape[0] // max_n)
    return points[::step][:max_n]


def clamp(v, lim):
    return max(-lim, min(lim, v))


def autonomy_to_cmd_vel(vx, vy, wz, faces_rear=LIDAR_FACES_REAR):
    """Map an explorer command (LiDAR frame) to /cmd_vel. A 180 deg yaw flips vx, vy; wz is unchanged."""
    return (-vx, -vy, wz) if faces_rear else (vx, vy, wz)


def lidar_pose(odom, faces_rear=LIDAR_FACES_REAR):
    """Odometry (x, y, yaw, ...) as the explorer's pose: yaw of the LiDAR's forward, or None."""
    if odom is None:
        return None
    x, y, yaw = odom[0], odom[1], odom[2]
    if faces_rear:
        yaw = math.atan2(math.sin(yaw + math.pi), math.cos(yaw + math.pi))
    return (x, y, yaw)


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
        self.ap_cmd = (0.0, 0.0, 0.0)  # (vx, vy, wz) of the last decision
        self.ap_stats = {}         # explorer coverage stats for the current run
        self.dists = {'front': math.inf, 'rear': math.inf, 'left': math.inf, 'right': math.inf}
        self.lidar_t = None        # monotonic time of last usable cloud
        # map and point-to-point waypoints (odometry frame)
        self.map = OccupancyMap(window_s=CLOUD_WINDOW_S)
        self.nav_mode = 'off'      # off | dry | live
        self.nav_state = 'IDLE'
        self.nav_cmd = (0.0, 0.0)  # (vx, wz) of the last decision, LiDAR frame
        self.nav_stats = {}
        # latest 3D cloud, base_footprint frame, decimated
        self.cloud = np.zeros((0, 3), dtype=np.float32)
        self.cloud_t = None
        self.cloud_acc = WindowedCloud(CLOUD_WINDOW_S, max_points=CLOUD_BUFFER_POINTS)  # odometry frame
        self.imu_up = None      # gravity-up unit vector in lidar_imu_link (low-passed)
        self.tilt_deg = None    # angle of gravity from base_footprint z, None if unknown

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
        self.nav = WaypointNav(NavConfig())
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Imu, TOPICS['imu'], lambda m: self._safe(lambda: self.s.mark('imu')), qos_profile_sensor_data)
        self.create_subscription(Odometry, TOPICS['odom'], lambda m: self._safe(lambda: self._odom(m)), 10)
        self.create_subscription(MotorsState, TOPICS['set_motor'], lambda m: self._safe(lambda: self.s.mark('set_motor')), 10)
        self.create_subscription(PointCloud2, LIDAR_TOPIC, lambda m: self._safe(lambda: self._lidar(m)), qos_profile_sensor_data)
        self.create_subscription(Imu, LIDAR_IMU_TOPIC, lambda m: self._safe(lambda: self._lidar_imu(m)), qos_profile_sensor_data)
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
            self.s.map.add_pose(p.x, p.y)

    def _lidar_imu(self, msg):
        a = msg.linear_acceleration
        v = np.array([a.x, a.y, a.z], dtype=np.float64)
        n = float(np.linalg.norm(v))
        if not (IMU_G_MIN < n < IMU_G_MAX):
            return                  # moving, or a bad sample: keep the last estimate
        u = v / n
        with self.s.lock:
            if self.s.imu_up is None:
                self.s.imu_up = u
            else:
                b = (1.0 - IMU_ALPHA) * self.s.imu_up + IMU_ALPHA * u
                self.s.imu_up = b / np.linalg.norm(b)

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
        R, tilt = self._gravity_rotation()
        if R is not None:
            xyz = xyz @ R.T         # z is now true vertical
        far = np.linalg.norm(xyz, axis=1) < CLOUD_RANGE
        body3 = (np.abs(xyz[:, 0]) < BODY_X) & (np.abs(xyz[:, 1]) < BODY_Y)
        cloud = decimate(xyz[far & ~body3], CLOUD_MAX).astype(np.float32)
        with self.s.lock:
            self.s.cloud = cloud
            self.s.cloud_t = time.monotonic()
            self.s.tilt_deg = tilt
            pose = lidar_pose(self.s.odom)
            if pose is not None:
                self.s.cloud_acc.add(time.monotonic(), base_to_odom(decimate(cloud, CLOUD_ADD), pose))
        z = xyz[:, 2]
        keep = (z > MIN_Z) & (z < MAX_Z) & ~((np.abs(xyz[:, 0]) < BODY_X) & (np.abs(xyz[:, 1]) < BODY_Y))
        x, y = xyz[keep, 0], xyz[keep, 1]
        with self.s.lock:
            pose = lidar_pose(self.s.odom)
            if pose is not None:
                self.s.map.add_points(pose, x, y)
        dists = {
            'front': nearest(x[(x > BODY_X) & (np.abs(y) < CORRIDOR_HALF)]),
            'rear': nearest(-x[(x < -BODY_X) & (np.abs(y) < CORRIDOR_HALF)]),
            'left': nearest(y[(y > BODY_Y) & (np.abs(x) < SIDE_X)]),
            'right': nearest(-y[(y < -BODY_Y) & (np.abs(x) < SIDE_X)]),
        }
        with self.s.lock:
            self.s.dists = dists
            self.s.lidar_t = time.monotonic()

    def _gravity_rotation(self):
        """(R, tilt_deg): R takes gravity-up to +z in base_footprint, or (None, None) if unknown."""
        with self.s.lock:
            up_i = None if self.s.imu_up is None else self.s.imu_up.copy()
        if up_i is None:
            return None, None
        try:
            tf = self.tf_buffer.lookup_transform('base_footprint', 'lidar_imu_link',
                                                 rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1))
        except Exception:  # noqa: BLE001
            return None, None
        q = tf.transform.rotation
        up_b = _quat_to_matrix(q.x, q.y, q.z, q.w) @ up_i
        return align_rotation(up_b), tilt_deg(up_b)

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
            odom = self.s.odom
            nav_mode = self.s.nav_mode

        if estop:
            with self.s.lock:
                self.s.moving = False
                self.s.ap_state = 'ESTOP'
                self.s.ap_cmd = (0.0, 0.0, 0.0)
            self._publish(0.0, 0.0)
            return

        if mode in ('live', 'dry'):
            age = None if lidar_t is None else now - lidar_t
            state, vx, vy, wz = self.explorer.decide(now, dists['front'], dists['rear'],
                                                     dists['left'], dists['right'], age,
                                                     lidar_pose(odom))
            with self.s.lock:
                self.s.ap_state = state
                self.s.ap_cmd = (vx, vy, wz)
                self.s.ap_stats = self.explorer.stats()
            if mode == 'live':
                cvx, cvy, cwz = autonomy_to_cmd_vel(vx, vy, wz)
                self._publish(cvx, cwz, cvy)
                with self.s.lock:
                    self.s.moving = (vx != 0.0 or vy != 0.0 or wz != 0.0)
            return

        if nav_mode in ('live', 'dry'):
            age = None if lidar_t is None else now - lidar_t
            with self.s.lock:
                state, vx, wz = self.nav.step(now, lidar_pose(odom), dists['front'], age)
                self.s.nav_state = state
                self.s.nav_cmd = (vx, wz)
                self.s.nav_stats = self.nav.stats()
            if nav_mode == 'live':
                cvx, cvy, cwz = autonomy_to_cmd_vel(vx, 0.0, wz)
                self._publish(cvx, cwz, cvy)
                with self.s.lock:
                    self.s.moving = (vx != 0.0 or wz != 0.0)
            return

        # manual pad
        cmd = self.s.manual_command(now)
        if cmd is None:
            return
        vx, vy, wz = cmd
        self._publish(vx, wz, vy)

    def set_nav(self, mode):
        """Arm or disarm waypoint following. Each arm restarts the run from the first goal."""
        with self.s.lock:
            was_live = self.s.nav_mode == 'live'
            self.s.nav_mode = mode
            self.s.nav_state = 'IDLE'
            self.s.nav_cmd = (0.0, 0.0)
            self.nav.reset()
            self.s.nav_stats = self.nav.stats()
            if was_live:
                self.s.moving = True   # makes the manual path send a few zeros on the way out

    def set_waypoints(self, points):
        """Replace the goal list. Refused while live navigation is driving."""
        with self.s.lock:
            if self.s.nav_mode == 'live':
                return False
            self.nav.set_points(points)
            self.s.nav_state = 'IDLE'
            self.s.nav_stats = self.nav.stats()
            return True

    def set_autonomy(self, mode):
        with self.s.lock:
            self.s.ap_mode = mode
            self.s.ap_state = 'IDLE'
            self.s.ap_cmd = (0.0, 0.0, 0.0)
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


def token_role(supplied, control, view=''):
    '''control or view for a matching token, else None. Empty tokens never match.'''
    if supplied and control and hmac.compare_digest(supplied.encode(), control.encode()):
        return 'control'
    if supplied and view and hmac.compare_digest(supplied.encode(), view.encode()):
        return 'view'
    return None


def make_handler(state, token, node, view_token=''):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'RobotDashboard/2.0'

        def log_message(self, fmt, *args):
            sys.stderr.write('[dashboard] %s - %s\n' % (self.address_string(), fmt % args))

        def _role(self, query):
            supplied = self.headers.get('X-Token') or query.get('token', [''])[0]
            return token_role(supplied, token, view_token)

        def _authed(self, query):
            return self._role(query) is not None

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
                role = self._role(parse_qs(url.query))
                if role is None:
                    return self._send(401, {'error': 'bad token'})
                payload = status_payload(state)
                payload['role'] = role
                return self._send(200, payload)
            if url.path == '/api/cloud':
                if not self._authed(parse_qs(url.query)):
                    return self._send(401, {'error': 'bad token'})
                return self._send(200, cloud_payload(state))
            if url.path == '/api/map':
                if not self._authed(parse_qs(url.query)):
                    return self._send(401, {'error': 'bad token'})
                return self._send(200, map_payload(state, node))
            self._send(404, {'error': 'not found'})

        def do_POST(self):
            url = urlparse(self.path)
            role = self._role(parse_qs(url.query))
            if role is None:
                return self._send(401, {'error': 'bad token'})
            if role != 'control':
                return self._send(403, {'error': 'view-only token: use the control link to drive or arm'})
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
                    if state.nav_mode != 'off':
                        state.rejections += 1
                        return self._send(409, {'error': f'waypoint path is {state.nav_mode}; stop it first'})
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
                with state.lock:
                    nav_mode = state.nav_mode
                if nav_mode != 'off' and mode != 'off':
                    return self._send(409, {'error': f'waypoint path is {nav_mode}; stop it first'})
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

            if url.path == '/api/nav':
                mode = str(body.get('mode', 'off'))
                if mode not in ('off', 'dry', 'live'):
                    return self._send(400, {'error': 'mode must be off, dry or live'})
                with state.lock:
                    estop, spin_error, ap_mode = state.estop, state.spin_error, state.ap_mode
                    lidar_age = None if state.lidar_t is None else time.monotonic() - state.lidar_t
                    have_odom = state.odom is not None
                    n_goals = len(node.nav.points)
                if spin_error:
                    return self._send(503, {'error': f'ROS executor down: {spin_error}'})
                if mode != 'off':
                    if ap_mode != 'off':
                        return self._send(409, {'error': f'autonomy is {ap_mode}; set it off first'})
                    if estop:
                        return self._send(409, {'error': 'e-stop engaged; reset first'})
                    if n_goals == 0:
                        return self._send(409, {'error': 'set waypoints first'})
                if mode == 'live':
                    if body.get('confirm') != 'TETHERED':
                        return self._send(400, {'error': 'live needs {"mode":"live","confirm":"TETHERED"}'})
                    if not have_odom:
                        return self._send(409, {'error': 'no odometry'})
                    if lidar_age is None or lidar_age > 0.5:
                        return self._send(409, {'error': 'no fresh LiDAR data'})
                node.set_nav(mode)
                print(f'[dashboard] waypoint path -> {mode}', flush=True)
                return self._send(200, {'ok': True, 'mode': mode})

            if url.path == '/api/waypoints':
                pts = body.get('points')
                if not isinstance(pts, list) or len(pts) > MAX_WAYPOINTS:
                    return self._send(400, {'error': f'points: list of up to {MAX_WAYPOINTS} [x, y] pairs'})
                clean = []
                for p in pts:
                    try:
                        x, y = float(p[0]), float(p[1])
                    except (TypeError, ValueError, IndexError, KeyError):
                        return self._send(400, {'error': 'each point is [x, y] in metres'})
                    if not (math.isfinite(x) and math.isfinite(y)) or abs(x) > MAX_COORD or abs(y) > MAX_COORD:
                        return self._send(400, {'error': f'coordinates must be finite and within {MAX_COORD} m'})
                    clean.append((x, y))
                if not node.set_waypoints(clean):
                    return self._send(409, {'error': 'stop live navigation before editing waypoints'})
                return self._send(200, {'ok': True, 'points': [[x, y] for x, y in clean]})

            if url.path == '/api/map/clear':
                with state.lock:
                    state.map.clear()
                return self._send(200, {'ok': True})

            if url.path == '/api/estop':
                with state.lock:
                    state.estop = True
                    state.drive = (0.0, 0.0, 0.0)
                    state.ap_mode = 'off'
                    state.nav_mode = 'off'
                print('[dashboard] E-STOP engaged', flush=True)
                return self._send(200, {'ok': True, 'estop': True})

            if url.path == '/api/reset':
                with state.lock:
                    state.estop = False
                node.set_autonomy('off')
                node.set_nav('off')
                print('[dashboard] e-stop reset, autonomy and waypoint path off', flush=True)
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
                'vy': state.ap_cmd[1],
                'wz': state.ap_cmd[2],
                'lidar_faces_rear': LIDAR_FACES_REAR,
                'stats': dict(state.ap_stats),
                'cmd_vel': dict(zip(('vx', 'vy', 'wz'), autonomy_to_cmd_vel(*state.ap_cmd))),
                'dists_m': dists,
                'lidar_age_s': None if state.lidar_t is None else round(now - state.lidar_t, 2),
            },
            'nav': {'mode': state.nav_mode, 'state': state.nav_state,
                    'vx': state.nav_cmd[0], 'wz': state.nav_cmd[1],
                    'cmd_vel': dict(zip(('vx', 'vy', 'wz'), autonomy_to_cmd_vel(state.nav_cmd[0], 0.0, state.nav_cmd[1]))),
                    'stats': dict(state.nav_stats)},
            'battery_mv': state.battery_mv,
            'odom': None if odom is None else {'x': odom[0], 'y': odom[1], 'yaw': odom[2],
                                              'vx': odom[3], 'vy': odom[4], 'wz': odom[5]},
            'commands_sent': state.commands_sent,
            'rejections': state.rejections,
            'warnings': list(state.warnings)[-20:],
        }
    payload['rates_hz'] = {k: state.rate(k, now) for k in TOPICS}
    return payload


def base_to_odom(pts, pose):
    """Base-frame points (N, 3) into the odometry frame for pose = (x, y, yaw). z unchanged."""
    x0, y0, yaw = pose
    c, sn = math.cos(yaw), math.sin(yaw)
    out = np.empty((pts.shape[0], 3), dtype=np.float32)
    out[:, 0] = x0 + c * pts[:, 0] - sn * pts[:, 1]
    out[:, 1] = y0 + sn * pts[:, 0] + c * pts[:, 1]
    out[:, 2] = pts[:, 2]
    return out


def odom_to_base(pts, pose):
    """Odometry-frame points (N, 3) into the robot frame at pose. Inverse of base_to_odom."""
    x0, y0, yaw = pose
    c, sn = math.cos(yaw), math.sin(yaw)
    dx = pts[:, 0] - x0
    dy = pts[:, 1] - y0
    out = np.empty((pts.shape[0], 3), dtype=np.float32)
    out[:, 0] = c * dx + sn * dy
    out[:, 1] = -sn * dx + c * dy
    out[:, 2] = pts[:, 2]
    return out


def cloud_payload(state):
    """Recent cloud (CLOUD_WINDOW_S) in the current robot frame, thinned for the page."""
    now = time.monotonic()
    with state.lock:
        odom = state.odom
        t = state.cloud_t
        acc = state.cloud_acc.stack(now)
        buffer_points = len(state.cloud_acc)
        raw = state.cloud
        tilt = state.tilt_deg
    pose = lidar_pose(odom)
    accumulated = pose is not None and acc.shape[0] > 0
    arr = raw
    if accumulated:
        arr = odom_to_base(acc, pose)
        arr = arr[np.linalg.norm(arr, axis=1) < CLOUD_RANGE]
    arr = decimate(arr, CLOUD_MAX)
    return {
        'frame': 'base_footprint',
        'n': int(arr.shape[0]),
        'accumulated': accumulated,
        'window_s': CLOUD_WINDOW_S,
        'buffer_points': buffer_points,
        'buffer_max': CLOUD_BUFFER_POINTS,
        'gravity': 'aligned' if tilt is not None else 'unavailable',
        'tilt_deg': None if tilt is None else round(tilt, 2),
        'age_s': None if t is None else round(now - t, 2),
        'points': np.round(arr, 3).ravel().tolist(),
    }


def map_payload(state, node):
    with state.lock:
        snap = state.map.snapshot()
        odom = state.odom
        pose = lidar_pose(odom)
        snap['pose'] = None if pose is None else {
            'x': odom[0], 'y': odom[1], 'yaw_odom': odom[2], 'yaw_lidar': pose[2]}
        snap['waypoints'] = [[x, y] for x, y in node.nav.points]
        snap['nav'] = {'mode': state.nav_mode, 'state': state.nav_state,
                       'index': node.nav.index, 'arrived': node.nav.arrived,
                       'halted': node.nav.halted}
    return snap


def main():
    host = os.environ.get('DASHBOARD_HOST', '127.0.0.1')
    port = int(os.environ.get('DASHBOARD_PORT', '8080'))
    token = os.environ.get('DASHBOARD_TOKEN') or secrets.token_urlsafe(16)
    view_token = os.environ.get('DASHBOARD_VIEW_TOKEN', '')

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

    httpd = ThreadingHTTPServer((host, port), make_handler(state, token, node, view_token))
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
  main{padding:16px;max-width:1200px;margin:auto;display:grid;gap:12px}
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
    <p id="authMsg" class="bad" style="margin:8px 0 0"></p>
    <p style="opacity:.7;margin:6px 0 0">Get the links from scripts/dashboard_url.sh on the robot. The control link can drive and arm; the view link only reads.</p>
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

  <div class="card"><h2>3D LiDAR (Unitree L2, robot frame, gravity aligned, Y up)</h2>
    <div id="cloud3d" style="width:100%;height:clamp(260px,52vh,560px);border:1px solid var(--line);border-radius:6px;overflow:hidden"></div>
    <p style="margin:8px 0 0;opacity:.7" id="cloudInfo">waiting for LiDAR</p>
    <div class="ap" style="margin-top:8px">
      <button onclick="resetCloudView()">Reset 3D view</button>
    </div>
    <p style="opacity:.7;margin:6px 0 0">Drag to orbit, scroll to zoom. Colour is height: blue low, red high. Points are thinned to 4000 and returns beyond 20 m are dropped.</p>
  </div>

  <div class="card"><h2>Map and waypoints (odometry frame)</h2>
    <canvas id="map" style="display:block;width:100%;aspect-ratio:1/1;max-height:clamp(260px,46vh,520px);border:1px solid var(--line);border-radius:6px;touch-action:none;cursor:crosshair"></canvas>
    <p style="margin:8px 0 0;opacity:.7" id="mapInfo"></p>
    <p style="opacity:.7;margin:6px 0 8px">Tap the map to add a waypoint. Dark squares are LiDAR returns, the blue line is wheel odometry, and the arrow is the robot's LiDAR-forward heading. Odometry drifts, so this is a local picture for short runs, not a survey.</p>
    <div class="ap">
      <button onclick="setNav('dry')">Run path (dry)</button>
      <button onclick="setNav('live')">Run path (live, tethered)</button>
      <button onclick="setNav('off')">Stop path</button>
      <button onclick="clearWps()">Clear waypoints</button>
      <button onclick="clearMap()">Clear map</button>
    </div>
    <p id="navState" style="margin:8px 0 0"></p>
    <div class="log" id="wpList" style="margin-top:8px"></div>
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

  <div class="card" id="statusCard"><h2>Status</h2>
    <p id="roleTag" style="margin:0 0 8px;opacity:.7"></p>
    <div class="grid" id="kv"></div>
    <button onclick="forgetTok()" style="margin-top:8px">Forget token on this device</button>
    <p style="margin:8px 0 0;opacity:.7" id="rates"></p>
  </div>

  <div class="card"><h2>Recent warnings</h2><div class="log" id="log"></div></div>
</main>
<script>
const q = new URLSearchParams(location.search);
let TOKEN = q.get('token') || sessionStorage.getItem('robotTok') || '';
if (q.get('token')) history.replaceState(null, '', location.pathname);  // keep the token out of the address bar
if (TOKEN) { sessionStorage.setItem('robotTok', TOKEN); document.getElementById('auth').hidden = true; }
function forgetTok(){ sessionStorage.removeItem('robotTok'); TOKEN = ''; location.reload(); }
function authFail(msg){ document.getElementById('auth').hidden = false; document.getElementById('authMsg').textContent = msg; }
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

// ---- map and waypoints ----
let MAP = {cell_m: 0.1, hits: [], trail: [], waypoints: [], pose: null, nav: {}, cells: 0};
let VIEW = null;
const cssVar = n => getComputedStyle(document.body).getPropertyValue(n).trim() || '#888';
async function getMap(){
  if (!TOKEN) return;
  try { const r = await fetch('/api/map?token='+encodeURIComponent(TOKEN)); if (r.ok) MAP = await r.json(); } catch(e){}
  drawMap(); renderWps();
}
function mapBounds(){
  let x0=Infinity, x1=-Infinity, y0=Infinity, y1=-Infinity;
  const add = (x, y) => { if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y; };
  MAP.trail.forEach(p => add(p[0], p[1]));
  MAP.waypoints.forEach(p => add(p[0], p[1]));
  MAP.hits.forEach(h => add(h[0]*MAP.cell_m, h[1]*MAP.cell_m));
  if (MAP.pose) add(MAP.pose.x, MAP.pose.y);
  if (x0 === Infinity) return {x0:-2, x1:2, y0:-2, y1:2};
  return {x0:x0-0.5, x1:x1+0.5, y0:y0-0.5, y1:y1+0.5};
}
function drawMap(){
  const c = document.getElementById('map');
  const w = c.clientWidth, h = c.clientHeight;
  if (!w || !h) return;
  const dpr = window.devicePixelRatio || 1;
  if (c.width !== Math.round(w*dpr) || c.height !== Math.round(h*dpr)) { c.width = Math.round(w*dpr); c.height = Math.round(h*dpr); }
  const g = c.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  const b = mapBounds();
  const span = Math.max(b.x1-b.x0, b.y1-b.y0, 4);
  const cx = (b.x0+b.x1)/2, cy = (b.y0+b.y1)/2;
  const s = Math.min(w, h) / span;
  const X = x => w/2 + (x-cx)*s, Y = y => h/2 - (y-cy)*s;
  VIEW = {inv: (px, py) => ({x: cx + (px-w/2)/s, y: cy - (py-h/2)/s})};
  // 1 m grid
  g.strokeStyle = cssVar('--line'); g.lineWidth = 1; g.globalAlpha = 0.5;
  for (let gx = Math.ceil(b.x0); gx <= b.x1; gx++) { g.beginPath(); g.moveTo(X(gx), 0); g.lineTo(X(gx), h); g.stroke(); }
  for (let gy = Math.ceil(b.y0); gy <= b.y1; gy++) { g.beginPath(); g.moveTo(0, Y(gy)); g.lineTo(w, Y(gy)); g.stroke(); }
  // LiDAR hits
  const cell = MAP.cell_m, cpx = Math.max(1, cell*s);
  g.fillStyle = cssVar('--fg');
  for (const [ix, iy, n] of MAP.hits) {
    g.globalAlpha = Math.min(1, 0.25 + 0.1*n);
    g.fillRect(X(ix*cell), Y(iy*cell + cell), cpx, cpx);
  }
  g.globalAlpha = 1;
  // odometry trail
  if (MAP.trail.length > 1) {
    g.strokeStyle = '#0969da'; g.lineWidth = 2; g.beginPath();
    MAP.trail.forEach((p, i) => { if (i) g.lineTo(X(p[0]), Y(p[1])); else g.moveTo(X(p[0]), Y(p[1])); });
    g.stroke();
  }
  // waypoints: done (green), current (red), pending (amber)
  const idx = (MAP.nav && MAP.nav.index) || 0;
  const live = MAP.nav && MAP.nav.mode !== 'off';
  MAP.waypoints.forEach((p, i) => {
    g.beginPath(); g.arc(X(p[0]), Y(p[1]), 11, 0, 2*Math.PI);
    g.fillStyle = i < idx ? cssVar('--ok') : (i === idx && live ? cssVar('--bad') : '#9a6700');
    g.fill();
    g.fillStyle = '#fff'; g.font = 'bold 12px system-ui'; g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText(String(i+1), X(p[0]), Y(p[1]));
  });
  // robot: triangle along the LiDAR-forward heading (canvas y is flipped, so negate the angle)
  if (MAP.pose) {
    g.save(); g.translate(X(MAP.pose.x), Y(MAP.pose.y)); g.rotate(-MAP.pose.yaw_lidar);
    g.beginPath(); g.moveTo(12, 0); g.lineTo(-8, 7); g.lineTo(-8, -7); g.closePath();
    g.fillStyle = '#0969da'; g.fill(); g.restore();
  }
  document.getElementById('mapInfo').textContent =
    `${MAP.cells} occupied cells · ${MAP.trail.length} trail points · view ${span.toFixed(1)} m across`;
}
function renderWps(){
  const idx = (MAP.nav && MAP.nav.index) || 0;
  document.getElementById('wpList').textContent = MAP.waypoints.length
    ? MAP.waypoints.map((p, i) => `${i+1}. x ${p[0].toFixed(2)} m, y ${p[1].toFixed(2)} m` + (i < idx ? '  done' : '')).join(String.fromCharCode(10))
    : 'no waypoints: tap the map to add one';
}
async function putWps(pts){
  const r = await post('/api/waypoints', {points: pts});
  if (r.error) alert(r.error); else MAP.waypoints = r.points;
  drawMap(); renderWps();
}
function clearWps(){ return putWps([]); }
async function clearMap(){ const r = await post('/api/map/clear'); if (r.error) alert(r.error); getMap(); }
async function setNav(mode){
  if (mode === 'live' && !confirm('Drive the robot through the waypoints (live, tethered)? Keep your hand near the e-stop.')) return;
  const body = {mode};
  if (mode === 'live') body.confirm = 'TETHERED';
  const r = await post('/api/nav', body);
  if (r.error) alert(r.error);
}
document.getElementById('map').addEventListener('click', e => {
  if (!VIEW) return;
  const r = e.currentTarget.getBoundingClientRect();
  const p = VIEW.inv(e.clientX - r.left, e.clientY - r.top);
  putWps([...MAP.waypoints, [Math.round(p.x*100)/100, Math.round(p.y*100)/100]]);
});

const fmt = (v, d=2) => (v === null || v === undefined) ? '—' : (typeof v === 'number' ? v.toFixed(d) : v);
function cell(label, value, cls=''){ return `<div class="kv"><b class="${cls}">${value}</b><span>${label}</span></div>`; }
async function refresh(){
  if (!TOKEN) return;
  let s;
  try {
    const r = await fetch('/api/status?token='+encodeURIComponent(TOKEN));
    if (r.status === 401){ authFail(TOKEN ? 'Token rejected. Use the control or view link from scripts/dashboard_url.sh.' : 'Enter the token.'); return; }
    s = await r.json();
  } catch(e){ return; }
  const o = s.odom, a = s.autonomy;
  const viewOnly = s.role === 'view';
  document.getElementById('roleTag').textContent = viewOnly ? 'View-only token: controls are disabled.' : 'Control token.';
  document.querySelectorAll('main button').forEach(b => { if (!b.closest('#auth, #statusCard')) b.disabled = viewOnly; });
  document.getElementById('authMsg').textContent = '';
  document.getElementById('estopState').innerHTML = s.estop ? '<span class="bad">E-STOP ENGAGED</span>' : '<span class="ok">armed</span>';
  ['Off','Dry','Live'].forEach(n => document.getElementById('ap'+n).classList.toggle('on', a.mode === n.toLowerCase()));
  document.getElementById('apState').innerHTML =
    `mode <b>${a.mode}</b> · state <b>${a.state}</b> · cmd vx ${fmt(a.vx,3)} vy ${fmt(a.vy,3)} m/s, wz ${fmt(a.wz,2)} rad/s (LiDAR frame)` +
    (a.lidar_faces_rear ? ' · <span class="warn">LiDAR faces rear: /cmd_vel vx, vy flipped</span>' : '') +
    (a.mode === 'dry' ? ' <span class="warn">(dry: not publishing)</span>' : '');
  const d = a.dists_m;
  const n = s.nav, ns = n.stats || {};
  document.getElementById('navState').innerHTML =
    `path mode <b>${n.mode}</b> · state <b>${n.state}</b> · cmd (LiDAR frame) vx ${fmt(n.vx,3)} m/s, wz ${fmt(n.wz,2)} rad/s` +
    (ns.goals ? ` · goal ${Math.min((ns.index||0)+1, ns.goals)}/${ns.goals}, arrived ${ns.arrived||0}` : '') +
    (n.mode === 'dry' ? ' <span class="warn">(dry: not publishing)</span>' : '') +
    (ns.halted ? ` <span class="bad">${ns.halted}: stop and clear the path to rerun</span>` : '');
  document.getElementById('apDist').innerHTML = [
    cell('front m', fmt(d.front,2)), cell('rear m', d.rear===null ? 'blind' : fmt(d.rear,2)),
    cell('left m', fmt(d.left,2)), cell('right m', fmt(d.right,2)),
    cell('lidar age s', fmt(a.lidar_age_s,2), (a.lidar_age_s===null || a.lidar_age_s>0.5)?'bad':''),
    cell('covered m²', fmt((a.stats||{}).area_m2,2)), cell('driven m', fmt((a.stats||{}).distance_m,2)),
    cell('scans / looks', `${(a.stats||{}).scans ?? '—'} / ${(a.stats||{}).looks ?? '—'}`),
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
setInterval(getMap, 1000); getMap();
async function getCloud(){
  if (!TOKEN || !window.updateCloud) return;
  try {
    const r = await fetch('/api/cloud?token='+encodeURIComponent(TOKEN));
    if (!r.ok) return;
    const d = await r.json();
    window.updateCloud(d.points, d.n);
    document.getElementById('cloudInfo').textContent =
      d.n ? `${d.n} points shown · buffer ${d.buffer_points.toLocaleString()} / ${d.buffer_max.toLocaleString()} over last ${d.window_s} s · gravity ${d.gravity}` + (d.tilt_deg === null ? '' : ` (tilt ${d.tilt_deg}°)`) + ` · ${d.accumulated ? 'accumulated' : 'single scan'}` : 'no LiDAR points yet';
  } catch(e){}
}
setInterval(getCloud, 1000); getCloud();
window.addEventListener('resize', drawMap);
</script>
<script type="importmap">
{"imports": {
  "three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
  "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"
}}
</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
const el = document.getElementById('cloud3d');
const renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
el.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, 1, 0.05, 200);
const HOME_POS = [5, 4, 5], HOME_TARGET = [0, 0.3, 0];
camera.position.set(...HOME_POS);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(...HOME_TARGET);
const grid = new THREE.GridHelper(20, 20, 0x888888, 0x555555);
scene.add(grid);
scene.add(new THREE.AxesHelper(1));
const body = new THREE.Mesh(new THREE.BoxGeometry(0.4, 0.2, 0.32),
  new THREE.MeshBasicMaterial({color: 0x0969da, wireframe: true}));
body.position.y = 0.1;
scene.add(body);
const geom = new THREE.BufferGeometry();
const pts = new THREE.Points(geom, new THREE.PointsMaterial({size: 0.04, vertexColors: true}));
scene.add(pts);
function resize() {
  const w = el.clientWidth, h = el.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
// height ramp: blue at -0.5 m, green at 0.5 m, red at 1.5 m (base_footprint z)
function heightColor(z) {
  const t = Math.min(1, Math.max(0, (z + 0.5) / 2.0));
  return [t, 1 - Math.abs(t - 0.5) * 2, 1 - t];
}
// ROS coordinates (x forward, y left, z up) are shown with Y up: three = (x, z, -y).
window.updateCloud = (flat, n) => {
  if (!n) { geom.setDrawRange(0, 0); return; }
  const pos = new Float32Array(n * 3);
  const col = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    const x = flat[i * 3], y = flat[i * 3 + 1], z = flat[i * 3 + 2];
    pos[i * 3] = x;
    pos[i * 3 + 1] = z;
    pos[i * 3 + 2] = -y;
    col.set(heightColor(z), i * 3);
  }
  geom.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geom.setAttribute('color', new THREE.BufferAttribute(col, 3));
  geom.setDrawRange(0, n);
};
window.resetCloudView = () => {
  camera.position.set(...HOME_POS);
  controls.target.set(...HOME_TARGET);
  controls.update();
};
</script>
</body></html>
"""


if __name__ == '__main__':
    main()
