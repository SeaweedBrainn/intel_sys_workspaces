# Intel Sys mobile robot

ROS 2 Humble workspace for an autonomous mecanum rover: a Jetson Orin NX with an STM32 motor board, a Unitree L2 LiDAR and a RealSense D435. Localization is **Point-LIO → RTAB-Map → Nav2**. It runs in the Gazebo sim and on recorded bags; it has not yet run on the robot. Everything runs in Docker.

- [Getting started](docs/GETTING_STARTED.md): set up a machine, bring up the robot, open the dashboard.
- [Roadmap](docs/ROADMAP.md): current state, phases, decisions, open items.
- [Localization results](docs/LOCALIZATION_RESULTS.md): what has been measured, in the sim and on bags.
- [Dashboard](docs/DASHBOARD.md), [testing](docs/TESTING.md), [remote access](docs/REMOTE_ACCESS.md).

## Layout

| Path | What |
|---|---|
| `src/intel_sys_hardware` | STM32 bridge, mecanum controller, wheel odometry, LiDAR and camera launch files, udev rules |
| `src/intel_sys_interfaces` | STM32 board messages and services |
| `src/intel_sys_description` | URDF and meshes |
| `src/intel_sys_localization` | Point-LIO and RTAB-Map launch and config, frame adapter, LiDAR body filter |
| `src/intel_sys_navigation` | Nav2 config (STVL costmaps) |
| `src/intel_sys_sim` | Gazebo Fortress worlds, simulated L2, `ros_gz_bridge` |
| `src/intel_sys_bringup` | Top-level `sim.launch.py`, `robot.launch.py`, teleop |
| `scripts/` | Dashboard, autonomy, robot checks and tests; `scripts/eval/` evaluates localization |
| `third_party/robot.repos` | Pinned Point-LIO, Unitree SDK and RealSense sources, built into the images |
| `Dockerfile.rover`, `Dockerfile.sim` | Jetson (arm64) and laptop (amd64, with Gazebo) images |
| `docker-compose*.yml` | Dev containers, GPU, WSL and Zenoh overlays, and the robot's stacks (`docker-compose.robot.yml`) |

## Quick start

On a laptop (simulation):

```bash
./run_docker.sh --build                  # first time: build the sim image (~20 GB)
./run_docker.sh bash scripts/run_tests.sh
./run_docker.sh ros2 launch intel_sys_bringup sim.launch.py world:=loop.sdf x:=-6.5 y:=0.0
```

`run_docker.sh` picks the image for the machine (sim on x86_64, rover on the Jetson) and adds the NVIDIA and WSL overlays when it finds them.

On the robot:

```bash
scripts/robot.sh up                  # Zenoh router, STM32, LiDAR, robot description
scripts/dashboard.sh                 # dashboard, last
scripts/robot.sh up localization     # Point-LIO + RTAB-Map, robot standing still
```

The front of the robot is the sensor side: `+x` and teleop "forward" drive toward the LiDAR and camera.
