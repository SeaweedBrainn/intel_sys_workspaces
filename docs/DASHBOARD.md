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
