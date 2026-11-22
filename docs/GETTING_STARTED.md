# Getting started

What this repo runs, how to set up a new robot (or a second machine) from scratch, and how to bring up the stack and dashboard day to day.

## What it is

The robot is a mecanum base with:

- **STM32 board** on `/dev/rrc` (`ttyACM0`): motor control and wheel odometry, plus the IMU.
- **Unitree L2 LiDAR** at `192.168.1.2` on the robot's wired network. It publishes `/lidar/points_raw` at about 12 Hz.
- **Dashboard** (`scripts/dashboard/dashboard.py`): a web page for status, manual drive, an e-stop, reactive exploration, a map, and point-to-point waypoints. It is the only thing that publishes `/cmd_vel` while it runs.

Everything runs in Docker containers, with ROS 2 Humble and `rmw_zenoh_cpp` as the middleware. A `zenohd` router container runs alongside them.

Autonomy is still early. Localization and Nav2 are disabled, and there is no SLAM map yet. See [ROADMAP.md](ROADMAP.md) for what works and what is next.

## Hardware and network checklist

| Item | Expected |
|---|---|
| Computer | Jetson Orin NX (ARM64) or an x86_64 PC for simulation |
| STM32 | USB, appears as `/dev/rrc` (symlink to `ttyACM0`) |
| LiDAR | Ethernet to the Jetson, IP `192.168.1.2`. The Jetson's wired interface is on `192.168.1.0/24` (here `enP8p1s0`) |
| Remote access | Tailscale on the robot and your laptop. The robot's tailnet name is `autonomy-desktop`. Use the IP if MagicDNS does not resolve (see [REMOTE_ACCESS.md](REMOTE_ACCESS.md)) |
| Battery | Robot powered. Lift the wheels clear of the floor for any drive test |
| Emergency stop | Within reach. The dashboard e-stop is software only |

## Setting up a new machine

1. **Install prerequisites.** Docker Engine with the compose plugin, Git, and Tailscale. On the Jetson, also install the NVIDIA container runtime.

2. **Clone the repo** and check out the working branch (`ahmad-zenoh` at the time of writing):

   ```bash
   git clone <repo-url> intel_sys_workspaces
   cd intel_sys_workspaces
   git checkout ahmad-zenoh
   ```

3. **Build the rover image.** This takes a while and needs about 20 GB of disk:

   ```bash
   ./run_docker.sh --build
   ```

   `run_docker.sh` detects x86_64 or ARM64 and picks the compose overlay. Use `--cpu` or `--nvidia` to force one.

4. **Give the STM32 a stable device name.** Confirm the board shows up as `/dev/rrc`. If it does not, check the udev rule or the USB cable before going further.

5. **Give the LiDAR a fixed address.** Set the wired interface to `192.168.1.x/24` and confirm the sensor answers:

   ```bash
   ping -c 2 192.168.1.2
   ```

6. **Join the tailnet** on the robot (`sudo tailscale up`) and on your laptop. Note the robot's Tailscale IP with `tailscale ip -4`.

7. **Run the checks** before you try to move anything:

   ```bash
   scripts/preflight.sh     # reports PASS, FAIL, or SKIP per device
   scripts/run_tests.sh     # unit tests inside the container
   ```

   A SKIP means a device is offline (for example the LiDAR), not broken. Only a FAIL needs fixing.

## Daily bring-up

Start the layers in this order. The dashboard must start last, after its inputs exist, because it reads TF and topics once when it starts.

```bash
# 1. STM32 bridge, mecanum controller, wheel odometry
docker compose -f docker-compose.yml -f docker-compose.zenoh.yml run -d --rm --name stm32stack \
  intel-sys-rover bash -c 'source /opt/ros/humble/setup.bash && source /workspaces/intel_sys/install/setup.bash && exec ros2 launch intel_sys_hardware stm32.launch.py'

# 2. LiDAR driver plus robot description (TF tree)
docker compose -f docker-compose.yml -f docker-compose.zenoh.yml run -d --rm --name lidarstack \
  intel-sys-rover bash -c 'source /opt/ros/humble/setup.bash && source /workspaces/intel_sys/install/setup.bash && (ros2 launch intel_sys_description robot_description.launch.py &) ; exec ros2 launch intel_sys_hardware lidar.launch.py'

# 3. Dashboard (last)
scripts/dashboard.sh
```

Check the state before you use it:

```bash
docker ps --format '{{.Names}}\t{{.Status}}'   # stm32stack, lidarstack, robot-dashboard, intel-sys-zenohd
```

In the dashboard status, `rates_hz` should show roughly `odom 50`, `imu 50`, and `lidar 12`. A `base_footprint<-lidar_link` warning means TF is missing. Restart the dashboard (`scripts/dashboard.sh --stop`, then `scripts/dashboard.sh`) once the robot description is up.

Stop everything with `scripts/dashboard.sh --stop` and `docker stop stm32stack lidarstack`. The dashboard zeros the wheels on the way out.

## Opening the dashboard

Get ready-to-open links from the robot. From a laptop with the `jetson` ssh alias, this works:

```bash
ssh jetson '~/intel_sys_workspaces/scripts/dashboard_url.sh'
```

You get two links:

- **control**: drive, arm the waypoint path or exploration, e-stop.
- **view**: read-only status and map. Use it to watch someone else's run.

Open a link once per browser tab. The token is removed from the address bar and kept for that tab. "Forget token on this device" clears it. If the page says the token was rejected, copy the link again from `dashboard_url.sh`.

Tokens live in `~/.config/robot-dashboard/` on the robot (`token` and `view_token`, mode 600). They do not change when the dashboard restarts. Rotate them by deleting the file and restarting the dashboard, then sharing the new link.

## First checks before moving the robot

1. **Lift the robot** clear of the floor and run the response test (`scripts/dashboard/response_test.py`). It sends one-second pulses per axis and reports the odometry.
2. **Dry run a waypoint path.** Set one or two waypoints and choose "Run path (dry)". The page shows the commands it would send, and nothing is published.
3. **Only then** run a tethered live path or the manual pad on the floor, with the e-stop in reach.

## Where things are

| Path | What |
|---|---|
| `scripts/dashboard/dashboard.py` | Dashboard node, HTTP API, page |
| `scripts/autonomy/explorer.py` | Reactive exploration |
| `scripts/autonomy/waypoint.py` | Map and waypoint following (pure logic) |
| `scripts/dashboard.sh`, `scripts/dashboard_url.sh` | Start or stop the dashboard; print its links |
| `src/intel_sys_hardware/launch/` | STM32 and LiDAR launch files |
| `src/intel_sys_navigation/` | Nav2 config (not running yet) |
| `docs/DASHBOARD.md` | Dashboard API, safety rules, and tests |
| `docs/TESTING.md`, `docs/REMOTE_ACCESS.md` | Test tiers and remote access |
| `docs/ROADMAP.md` | Status and next phases |

## Known gotchas

- **Start order matters.** Start the dashboard after the STM32 and LiDAR stacks, or restart it afterward. Under the zenoh middleware, subscribers do not always re-match after a publisher restarts (see the roadmap).
- **Camera is off.** `/camera/image_raw` is at 0 Hz because no camera node is launched. Nothing in the dashboard depends on it.
- **The front is the sensor side, and the rear is blind.** Since 2026-11-22, `+vx` (teleop "forward") drives toward the L2 and the D435; the switch is `forward_is_sensor_side` in `stm32.launch.py` (default true; false restores the old motor-convention front). Only the half the sensors face is seen. Check the sector readings in dry mode before any live run.
- **Odometry drifts.** The map and waypoints are in the odometry frame. They are good for short runs only.
- **Mecanum watchdog warnings.** While the dashboard is idle, the controller logs "cmd_vel stale … wheels commanded to zero" about every two seconds. That is the safety watchdog working.
