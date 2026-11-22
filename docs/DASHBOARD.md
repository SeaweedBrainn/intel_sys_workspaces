# Robot Dashboard

A small web page for watching and driving the robot during tests. It runs in the rover container and serves a single page with status, a capped drive pad, and an e-stop.

For 3D views and sensor plots, use Foxglove (`ws://<robot>:8765`, layout in `src/intel_sys_bringup/config/foxglove_layout.json`). The dashboard covers status and control.

## Start and stop

```bash
scripts/dashboard.sh            # starts it in the background, prints the URL
scripts/dashboard.sh --stop     # stops it; the wheels are zeroed on the way out
docker logs robot-dashboard 2>&1 | grep token     # the access token
```

The STM32 stack must be running for live data and drive commands:

```bash
docker compose -f docker-compose.yml -f docker-compose.zenoh.yml run -d --rm --name stm32stack \
  intel-sys-rover bash -c 'source /opt/ros/humble/setup.bash && source /workspaces/intel_sys/install/setup.bash && exec ros2 launch intel_sys_hardware stm32.launch.py'
```

## Reaching it

Two tokens, both in `~/.config/robot-dashboard/` on the robot (mode 600, stable across restarts):

- `token`: **control**. Drive, path, autonomy, e-stop.
- `view_token`: **view-only**. Status and map. Use it to watch the robot without being able to move it. Control buttons are disabled in the page.

Get ready-to-open links on the robot, or from a laptop with the `jetson` ssh alias:

```bash
ssh jetson '~/intel_sys_workspaces/scripts/dashboard_url.sh'           # both
ssh jetson '~/intel_sys_workspaces/scripts/dashboard_url.sh control'
```

Opening a `?token=` link stores the token for that browser tab and removes it from the address bar. "Forget token on this device" clears it. A rejected token shows a message on the page and changes nothing on the robot.


- **Tailscale:** `http://<tailscale-ip>:8080/`, where the IP is `tailscale ip -4` on the robot. By default the dashboard binds to this address only, so other interfaces cannot reach it. The tailnet also includes other people's devices, so keep the token private.
- **SSH tunnel:** `ssh -L 8080:localhost:8080 <user>@<robot>`, then `http://localhost:8080/`.
- Pass the token in the page, or as `?token=...` the first time. The page stores it for the session only.

Env overrides: `DASHBOARD_HOST`, `DASHBOARD_PORT` (default 8080), `DASHBOARD_TOKEN` (generated at startup if unset).

## Safety

- **Caps:** linear 0.10 m/s, angular 0.5 rad/s. Requests over the cap are rejected with HTTP 400, not clamped.
- **Dead-man:** a drive command expires 0.3 s after the last refresh, and the node then sends zeros. The pad refreshes at 10 Hz while held and stops on release, on blur, and on page hide.
- **Nav2 coexistence:** the dashboard publishes to `/cmd_vel` only while driving, stopping, or in e-stop. It is silent when idle.
- **E-stop:** software only. It publishes zero at 10 Hz until reset. It is not a hardware e-stop, so keep the physical stop within reach.
- **Controller watchdog:** the mecanum controller zeroes the wheels if `/cmd_vel` is silent for 0.5 s, independent of the dashboard.
- **Executor failure:** if the ROS executor dies, `spin_error` is shown in status and drive requests return HTTP 503.

## Autonomous exploration

Reactive LiDAR exploration runs inside the dashboard node (logic in `scripts/autonomy/explorer.py`), so the dashboard stays the only `/cmd_vel` publisher. Modes: `off`, `dry` (decides and shows commands, publishes nothing), and `live` (needs `{"mode":"live","confirm":"TETHERED"}`, a clear e-stop, and fresh LiDAR). Manual drive is refused while autonomy is armed.

The Unitree L2 is mounted tilted and only sees the front half (azimuth -90° to +90°). The rear sector is never observed (shown as `blind`), and only the front half of each side sector is. Since 2026-11-22 the robot's front is the sensor side (the L2 and the D435): `mecanum_controller` has `forward_is_sensor_side` (default true, set in `stm32.launch.py`), so `+vx` on `/cmd_vel`, in teleop and in the manual pad drives toward what the LiDAR sees, and the URDF agrees. The explorer decides in a frame whose +x is the side the LiDAR sees (taken from TF). The dashboard asks the running controller for `forward_is_sensor_side`: with the new front it publishes autonomy commands as they are; with an older controller (no such parameter) it rotates them 180° (`vx`, `vy` negated, `wz` unchanged), as before. If the controller does not answer, or the URDF has the dome facing `-x` while no flip is in effect, live autonomy and waypoint runs are refused. `/api/status` shows `autonomy.controller_front`, `autonomy.lidar_faces_rear` (the flip in effect) and both commands (`autonomy.vx/vy/wz`, `autonomy.cmd_vel`). `DASHBOARD_LIDAR_FACES_REAR=1`/`0` forces the flip; leave it unset.

Scene capture is done offline from recorded bags (`record_data.sh`), so the explorer aims to reach unvisited space, sweep surfaces sideways so the LiDAR sees them from many positions, and keep motion slow. It keeps a coarse visit grid (0.5 m cells) from wheel odometry and uses it to rank headings and strafe directions. Wheel odometry drifts, especially when strafing, so the grid is only a hint over a run of a few minutes.

| State | Behavior |
|---|---|
| `LOOK` | Spin one full turn in place at 0.3 rad/s so the LiDAR sweeps all around. Each heading is scored by front room (up to 5 m) plus the share of unvisited cells along it; headings with less than 0.9 m of room rank last. Runs at start, after each `SCAN`, and every 3 m of travel (60 s fallback). Needs odometry. |
| `ORIENT` | Turn to the best heading from `LOOK`. |
| `FORWARD` | 0.10 m/s, slowing linearly inside 1.5 m. Drifts sideways (vy 0.04 m/s) away from a side closer than 0.30 m without changing heading. |
| `SCAN` | Surface ahead (≤ 1.0 m) and a side has ≥ 0.6 m: strafe along it at 0.08 m/s toward the side with more unvisited ground, holding 0.9 m from the surface by approaching only (never backing into the blind rear). Ends after 2 m, when the side closes to 0.30 m, or when the surface ends (front opens past 2 m); then `LOOK`. |
| `REVERSE` | Off by default (`rear_observed=False`), since the rear is blind. |
| `TURN` | Front ≤ 0.6 m and no side room: turn in place toward the more open side until the front clears. |
| `STUCK` | A turn ran past 10 s. Latched until Reset. |
| `STALE` | No LiDAR for 0.5 s: stop. |

`/api/status` reports run stats under `autonomy.stats`: cells visited, area covered, distance driven, scan and look counts, and time per state. They reset when the mode changes. Tune from `Config` in `explorer.py`.

Before running `live`, check each sector in `dry`: hold an object about 0.5 m in front and to each side and confirm the distance shows up.

## Map and waypoints

The page has a map of the area the robot has seen, plus point-to-point waypoints.

- **Frame:** odometry. The origin is where wheel odometry started (boot, or a reset of the odometry). Odometry drifts, so the map is a local picture for short runs, not a survey. There is no SLAM or Nav2 yet (see `docs/ROADMAP.md`, Phases 4 and 8).
- **Map:** LiDAR returns counted per 0.1 m cell (hits only), plus the wheel-odometry trail. Dark squares are returns, the blue line is the path travelled, and the arrow is the robot's LiDAR-forward heading (the same convention as autonomous exploration).
- **Waypoints:** tap the map to add a goal (up to 50, within 100 m of the origin). The robot turns to face each goal, drives forward at up to 0.10 m/s, slows near the goal, and counts it reached within 0.10 m. It stops for anything at or inside 0.45 m in front and resumes when the path clears. It also stops for good if a goal takes longer than 60 s, or if LiDAR data goes stale (0.5 s).
- **Dry run** computes and shows the commands without publishing. **Live (tethered)** needs `{"mode":"live","confirm":"TETHERED"}`, fresh LiDAR, and odometry. Keep the tether taut and your hand near the e-stop.
- While the path is armed, manual drive and autonomous exploration are refused. E-stop and reset disarm it. Waypoints are kept across a reset.

Before the first live run, check the frame in dry mode: push the robot by hand and confirm that the map's arrow turns and moves the way the robot does, and that obstacles appear where they are. If the arrow moves the wrong way relative to the robot, the controller's front (`forward_is_sensor_side`) and the URDF disagree, and waypoint runs would drive the wrong way.

Endpoints (token required): `GET /api/map`, `POST /api/waypoints` with `{"points": [[x, y], ...]}`, `POST /api/nav` with `{"mode": "off" | "dry" | "live"}`, `POST /api/map/clear`.

## Status

`GET /api/status` returns JSON with: e-stop state, last drive command and age, caps, battery (raw), odometry pose and twist, topic rates (IMU, odom, set_motor, LiDAR, camera), commands sent, rejections, and the last 20 WARN/ERROR lines from `/rosout`.

## Response test

`scripts/dashboard/response_test.py` drives each axis (forward, back, left, right, turn L and R) at the capped speed for one second, samples odometry, and prints rise time, displacement, and command rate:

```bash
TOKEN=<token> python3 scripts/dashboard/response_test.py --url http://<tailscale-ip>:8080 --duration 1.0 --out results/response.json
```

It needs the robot suspended or on blocks. The displacement is integrated from commanded wheel speeds, so it checks the software path, not physical motion.

## Tests

```bash
docker compose -f docker-compose.yml -f docker-compose.zenoh.yml run --rm intel-sys-rover bash -c \
  'source /opt/ros/humble/setup.bash && source /workspaces/intel_sys/install/setup.bash && cd /workspaces/intel_sys && python3 -m pytest -q scripts/dashboard/test_dashboard.py'
```

The tests cover the command gating: deadman expiry, stop-zero delivery, e-stop, caps, and the rosout level regression.
