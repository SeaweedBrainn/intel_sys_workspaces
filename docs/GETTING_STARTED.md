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

Each stack is a compose service in `docker-compose.robot.yml`, driven by `scripts/robot.sh`. There are two modes; start either with the robot standing still:

```bash
scripts/robot.sh up             # map and explore: drivers, health, Foxglove, Point-LIO, RTAB-Map, Nav2, explorer
scripts/robot.sh up follow      # follow a tag: drivers, health, Foxglove, Point-LIO, Nav2 in odom, follower
scripts/robot.sh up drivers     # drivers, health and Foxglove only (teleop, recording, sensor checks)
```

Foxglove comes up with every command, on the robot's Tailscale address, port 8765 ([FOXGLOVE.md](FOXGLOVE.md)).

**Switching modes** is one command: `up follow` while mapping (or `up` while following) stops the other mode's stacks and starts this one's. Point-LIO restarts with it, so odometry starts fresh from where the robot stands. Running `up` again in the same mode changes nothing that is running. To restart mapping with other options, `scripts/robot.sh stop localization`, then `up` with the new options.

After a crash or a reboot the drivers, the robot description, health and Foxglove come back on their own. The mode's stacks don't: a restart re-anchors odometry wherever the robot is, and a mapping restart starts a new map.

**Maps.** The database is `data/rtabmap.db`. Each new mapping session keeps the previous map as `data/rtabmap_<date>.db`. Options for `up`:
- `SLAM_LOCALIZATION=true` re-localizes on the saved map instead of mapping.
- `SLAM_CONTINUE=true` keeps mapping into the saved map; the new session joins it once the camera recognizes a place.
- `SLAM_CAMERA=false` runs RTAB-Map on the LiDAR alone. By default it uses the camera too and waits for its frames, so `up` stops with an error if the camera isn't publishing. A map made without the camera has no visual words, so map again before relying on the camera to re-localize.

**Exploration** (`up`): press **Explore: START** in Foxglove. The explorer drives to frontiers until nothing is left, then drives home.

**Tag following** (`up follow`; ArUco 5×5, id 10, 6.5 in): Nav2 plans in `odom` on the live LiDAR only, without RTAB-Map, so a person who pauses never becomes a stored obstacle. Following is off until **Follow tag: ON** in Foxglove; losing the tag for 3 s stops it.

Also useful:

```bash
scripts/robot.sh ps                    # what's running
scripts/robot.sh logs -f lidar         # follow one stack's log
scripts/robot.sh ros2 topic hz /odom   # the ros2 CLI on the robot's middleware
scripts/record_bag.sh <name>           # record into bags/ until Ctrl-C
scripts/robot.sh down                  # stop everything
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
