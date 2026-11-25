# Getting started

What this repo runs, how to set up a new robot (or a second machine) from scratch, and how to bring up the stack and dashboard day to day.

## What it is

The robot is a mecanum base with:

- **STM32 board** on `/dev/rrc` (`ttyACM0`): motor control and wheel odometry, plus the IMU.
- **Unitree L2 LiDAR** at `192.168.1.2` on the robot's wired network. It publishes `/lidar/points_raw` at about 12 Hz.
- **Dashboard** (`scripts/dashboard/dashboard.py`): a web page for status, manual drive, an e-stop, reactive exploration, a map, and point-to-point waypoints. It is the only thing that publishes `/cmd_vel` while it runs.

Everything runs in Docker containers, with ROS 2 Humble and `rmw_zenoh_cpp` as the middleware. A `zenohd` router container runs alongside them.

Autonomy is early. Point-LIO, RTAB-Map and Nav2 run in the sim and on recorded bags but have not yet run on the robot. See [ROADMAP.md](ROADMAP.md) for what works and what is next.

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

2. **Clone the repo** and check out the working branch (`slam-chain` at the time of writing):

   ```bash
   git clone <repo-url> intel_sys_workspaces
   cd intel_sys_workspaces
   git checkout slam-chain
   ```

3. **Build the rover image.** This takes a while and needs about 20 GB of disk:

   ```bash
   ./run_docker.sh --build
   ```

   `run_docker.sh` detects x86_64 or ARM64 and picks the compose overlay. Use `--cpu` or `--nvidia` to force one.

4. **Give the STM32 a stable device name.** Install the udev rules once, then confirm the board shows up as `/dev/rrc`. If it does not, check the USB cable before going further.

   ```bash
   sudo src/intel_sys_hardware/udev/install_udev_rules.sh
   ```

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

Each stack is a compose service in `docker-compose.robot.yml`, driven by `scripts/robot.sh`. Start the dashboard last, after its inputs exist, because it reads TF and topics once when it starts.

```bash
scripts/robot.sh up                 # Zenoh router, STM32 stack, LiDAR and camera drivers, robot description
scripts/dashboard.sh                # dashboard (last)
scripts/robot.sh up localization    # optional: Point-LIO + RTAB-Map, mapping a new map
scripts/robot.sh up navigation      # optional: also Nav2 on RTAB-Map's map
```

Start localization with the robot standing still. To re-localize on the last saved map instead of mapping, use `SLAM_LOCALIZATION=true scripts/robot.sh up localization`; the database is `data/rtabmap.db`, and starting a new mapping session keeps the previous one as `data/rtabmap_<date>.db`. RTAB-Map uses the D435i as well as the LiDAR by default, for visual loop closures and place recognition. It waits for camera frames and stops updating without them, so `scripts/robot.sh up localization` refuses to start unless `/camera/camera_info` is publishing; `SLAM_CAMERA=false` runs it on the LiDAR alone. A map made without the camera has no visual words, so map again before relying on the camera in localization mode. `scripts/robot.sh up navigation` never restarts a running localization.

**Tag following** (ArUco tag 10, 5×5, 6.5 in): `scripts/robot.sh up follow` starts Point-LIO and Nav2 without RTAB-Map (planning in `odom` on the live LiDAR) plus `tag_follower`; stop a running mapping session first. Following is off until `ros2 service call /tag_follower/enable std_srvs/srv/SetBool "{data: true}"` (or Foxglove's service call panel); `{data: false}` stops it, and so does losing the tag for 3 s. `/tag_follower/tag_pose` and `/tag_follower/goal` show in Foxglove.

Also useful: `scripts/robot.sh up foxglove` (Foxglove bridge on the Tailscale address, port 8765), `scripts/record_bag.sh <name>` (records into `bags/` until Ctrl-C) and `scripts/robot.sh ros2 <args>` (the `ros2` CLI on the robot's middleware).

The drivers, the robot description and the dashboard restart on their own after a crash or a reboot. Localization and navigation do not: a restart would re-anchor odometry wherever the robot is, and a mapping restart wipes the map.

**Nav2 publishes `/cmd_vel` too.** While navigation runs, the dashboard e-stop does not stop Nav2's goal. Stop it with `scripts/robot.sh stop navigation`, and keep a hand on the physical stop.

Check the state before you use it:

```bash
scripts/robot.sh ps
scripts/robot.sh logs -f lidar      # follow one stack's log
```

In the dashboard status, `rates_hz` should show roughly `odom 50`, `imu 50`, and `lidar 12`. A `base_footprint<-lidar_link` warning means TF is missing. Restart the dashboard (`scripts/dashboard.sh`) once the robot description is up.

Stop a stack with `scripts/robot.sh stop <service>`; stop everything with `scripts/dashboard.sh --stop` and `scripts/robot.sh stop`. The dashboard zeros the wheels on the way out.

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
| `src/intel_sys_navigation/` | Nav2 config |
| `docker-compose.robot.yml`, `scripts/robot.sh` | The robot's stacks as compose services |
| `scripts/eval/` | Localization evaluation on bags and in the sim |
| `docs/DASHBOARD.md` | Dashboard API, safety rules, and tests |
| `docs/TESTING.md`, `docs/REMOTE_ACCESS.md` | Test tiers and remote access |
| `docs/ROADMAP.md` | Status and next phases |
| `docs/LOCALIZATION_RESULTS.md` | Localization measurements |

## Foxglove

The sim and `robot.launch.py` start a Foxglove bridge on port 8765. In [Foxglove](https://app.foxglove.dev), open a connection to `ws://localhost:8765` (or `ws://<robot-ip>:8765`) and import `src/intel_sys_bringup/config/foxglove_layout.json` for a 3D view, the camera, IMU plots and teleop.

## Known gotchas

- **Start order matters.** Start the dashboard after the STM32 and LiDAR stacks, or restart it afterward. Under the zenoh middleware, subscribers do not always re-match after a publisher restarts (see the roadmap).
- **Camera.** The D435i runs as its own service (`camera`, container `camerastack`) at 640×480, 15 fps, color plus depth aligned to color and a depth cloud for Nav2. It must be on a USB 3 port; `scripts/robot.sh logs camera` shows `Device USB type: 3.2`. The robot description publishes its frames; the driver's TF is off.
- **The front is the sensor side, and the rear is blind.** Since 2026-11-22, `+vx` (teleop "forward") drives toward the L2 and the D435; the switch is `forward_is_sensor_side` in `stm32.launch.py` (default true; false restores the old motor-convention front). Only the half the sensors face is seen. Check the sector readings in dry mode before any live run.
- **Odometry drifts.** The map and waypoints are in the odometry frame. They are good for short runs only.
- **Mecanum watchdog warnings.** While the dashboard is idle, the controller logs "cmd_vel stale … wheels commanded to zero" about every two seconds. That is the safety watchdog working.
