# Getting started

How to set up a new robot (or a second machine) from scratch, and how to bring up the stacks day to day.

## What it is

A mecanum base with:

- **STM32 board** on `/dev/rrc` (`ttyACM0`): motor control, the IMU and the battery reading. Wheel odometry is integrated from **commanded** speeds, not encoders.
- **Unitree L2 LiDAR** at `192.168.1.2` on the robot's wired network, `/lidar/points_raw` at about 12 Hz, plus its own IMU.
- **RealSense D435i** on USB 3: colour and aligned depth for RTAB-Map's visual loop closures and the tag follower.

Everything runs in Docker with ROS 2 Humble and `rmw_zenoh_cpp`; a `zenohd` router container runs alongside. Localization is Point-LIO (odometry) → RTAB-Map (map, loop closures) → Nav2. [Foxglove](FOXGLOVE.md) is the UI. See [ROADMAP.md](ROADMAP.md) for what works and what is next.

## Hardware and network checklist

| Item | Expected |
|---|---|
| Computer | Jetson Orin NX (ARM64), or an x86_64 PC for simulation |
| STM32 | USB, appears as `/dev/rrc` (symlink to `ttyACM0`) |
| LiDAR | Ethernet to the Jetson, IP `192.168.1.2`. The Jetson's wired interface is on `192.168.1.0/24` (here `enP8p1s0`) |
| Camera | D435i on a USB 3 port (`scripts/robot.sh logs camera` shows `Device USB type: 3.2`) |
| Remote access | Tailscale on the robot and your laptop; the robot is `autonomy-desktop` on the tailnet ([REMOTE_ACCESS.md](REMOTE_ACCESS.md)) |
| Emergency stop | The physical stop within reach. Foxglove's E-STOP is software |

## Setting up a new machine

1. **Install prerequisites.** Docker Engine with the compose plugin, Git, and Tailscale. On the Jetson, also the NVIDIA container runtime.

2. **Clone the repo** and check out the working branch:

   ```bash
   git clone <repo-url> intel_sys_workspaces
   cd intel_sys_workspaces
   git checkout slam-chain
   ```

3. **Build the image** (a while, and about 20 GB of disk). `run_docker.sh` detects x86_64 or ARM64; `--cpu` or `--nvidia` forces one.

   ```bash
   ./run_docker.sh --build
   ```

   On the Jetson this builds `intel-sys-rover:latest`, the image every robot stack runs.

4. **Give the STM32 a stable device name.** Install the udev rules once, then check the board shows up as `/dev/rrc`:

   ```bash
   sudo src/intel_sys_hardware/udev/install_udev_rules.sh
   ```

5. **Give the LiDAR a fixed address.** Set the wired interface to `192.168.1.x/24` and check the sensor answers: `ping -c 2 192.168.1.2`.

6. **Join the tailnet** on the robot (`sudo tailscale up`) and on your laptop.

7. **Run the checks** before moving anything:

   ```bash
   scripts/preflight.sh                       # host: serial, image, disk, LiDAR, camera, clock
   ./run_docker.sh bash scripts/run_tests.sh  # build and unit tests in the container
   ```

   A SKIP means a device is offline, not broken. Only a FAIL needs fixing.

## Daily bring-up

Each stack is a compose service in `docker-compose.robot.yml`, driven by `scripts/robot.sh`:

```bash
scripts/robot.sh up                 # Zenoh router, STM32 stack, LiDAR and camera drivers, robot description, health
scripts/robot.sh up foxglove        # Foxglove bridge on the Tailscale address, port 8765 (docs/FOXGLOVE.md)
scripts/robot.sh up localization    # Point-LIO + RTAB-Map, mapping a new map
scripts/robot.sh up navigation      # also Nav2 on RTAB-Map's map, and the explorer (idle until started)
```

Start localization with the robot standing still. The drivers, the robot description and health restart on their own after a crash or a reboot. Localization and navigation don't: a restart re-anchors odometry wherever the robot is, and a mapping restart starts a new map. `scripts/robot.sh up navigation` starts localization if it isn't running and never restarts a running one.

**Maps.** The database is `data/rtabmap.db`. Each new mapping session keeps the previous map as `data/rtabmap_<date>.db`.
- `SLAM_LOCALIZATION=true scripts/robot.sh up localization` re-localizes on the saved map instead of mapping.
- `SLAM_CONTINUE=true` keeps mapping into the saved map; the new session joins it once the camera recognizes a place.
- RTAB-Map uses the camera as well as the LiDAR by default and waits for camera frames, so `up localization` refuses to start unless `/camera/camera_info` is publishing. `SLAM_CAMERA=false` runs it on the LiDAR alone. A map made without the camera has no visual words, so map again before relying on the camera to re-localize.

**Exploration.** With `up navigation`, press **Explore: START** in Foxglove. The explorer drives to frontiers until nothing is left, then drives home.

**Tag following** (ArUco 5×5, id 10, 6.5 in): `scripts/robot.sh up follow` runs Point-LIO and Nav2 without RTAB-Map (planning in `odom` on the live LiDAR) plus `tag_follower`; stop a running mapping session first (`scripts/robot.sh stop explorer navigation localization`). Following is off until **Follow tag: ON** in Foxglove (or `ros2 service call /tag_follower/enable std_srvs/srv/SetBool "{data: true}"`); losing the tag for 3 s stops it.

Also useful:

```bash
scripts/robot.sh ps                    # what's running
scripts/robot.sh logs -f lidar         # follow one stack's log
scripts/robot.sh ros2 topic hz /odom   # the ros2 CLI on the robot's middleware
scripts/record_bag.sh <name>           # record into bags/ until Ctrl-C
scripts/robot.sh stop <service>        # stop one stack; scripts/robot.sh down stops everything
```

Foxglove's Diagnostics panel shows `robot_health`: stream rates and freshness, and the `odom` and `map` transforms.

## Conventions

- **The front is the sensor side.** `+x`, `/cmd_vel` forward and teleop forward point toward the L2 and the D435. The STM32's own convention is the opposite; `forward_is_sensor_side` in `stm32.launch.py` handles it in `mecanum_controller` and `odom_publisher`. The rear is blind: only the half the sensors face is seen.
- **Who owns which transform.** Point-LIO (through its frame adapter) owns `odom→base_footprint`, planar; RTAB-Map owns `map→odom`. Nothing else publishes those transforms.
- **Wheel odometry is integrated from commanded speeds**, not encoders. Don't treat it as ground truth; nothing fuses it.
- **The URDF's sensor mounts are estimates** until the mechanical team sends measured values. Read mounts from TF rather than hard-coding them.

## Working on the code

- **Where changes happen.** Develop on a laptop, push to GitHub, and `git pull` on the Jetson. Teammates sometimes edit and commit directly on the Jetson: don't edit the same files in both places, and never commit or discard someone else's uncommitted work there.
- **The repo is public:** no tokens, IP addresses or personal details in commits.
- **Settings with a story.** Several settings look arbitrary but were paid for; the comment next to each says why. Read it before changing one (Point-LIO in `point_lio.yaml`, RTAB-Map in `rtabmap.yaml` and `slam.launch.py`, Nav2 in `nav2_params.yaml`).

## Known gotchas

- **A plain shell's `ros2` sees nothing.** The stacks use Zenoh; run `ros2` through `scripts/robot.sh ros2 ...` or `docker exec <container> ...`.
- **Mecanum watchdog warnings.** With nothing driving, the controller logs "cmd_vel stale … wheels commanded to zero" about every two seconds. That is the safety watchdog working.
- **RTAB-Map takes every parameter as a string.** From a launch file, pass them as `ParameterValue(..., value_type=str)`, or it crashes on bools.
- **No colons in URDF/xacro comments.** `robot_description` goes through a YAML parse and breaks on them.
- **Zenoh restarts.** Subscribers don't always re-match after a publisher restarts (see the roadmap). If a stack goes quiet after restarting another, restart the quiet one.
