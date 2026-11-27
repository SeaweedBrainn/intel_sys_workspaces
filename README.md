# Intel Sys mobile robot

ROS 2 Humble workspace for an autonomous mecanum rover: a Jetson Orin NX with an STM32 motor board, a Unitree L2 LiDAR and a RealSense D435. Localization is **Point-LIO → RTAB-Map → Nav2**, with frontier exploration and tag following on top. It runs on the robot, in the Gazebo sim and on recorded bags. Everything runs in Docker; Foxglove is the UI.

- [Getting started](docs/GETTING_STARTED.md): set up a machine, bring up the robot.
- [Foxglove](docs/FOXGLOVE.md): the UI, and who drives (teleop, Nav2, e-stop).
- [Navigation](docs/NAVIGATION.md): Nav2 setup, the frontier explorer and its results.
- [Roadmap](docs/ROADMAP.md): current state, open items, decisions.
- [Localization results](docs/LOCALIZATION_RESULTS.md): what has been measured, on the robot, in the sim and on bags.
- [Robot test day](docs/ROBOT_TEST_DAY.md): the next checklist to run on the rover.
- [Testing](docs/TESTING.md), [remote access](docs/REMOTE_ACCESS.md).

## Layout

| Path | What |
|---|---|
| `src/intel_sys_hardware` | STM32 bridge, mecanum controller, `twist_mux`, LiDAR and camera launch files, `robot_health`, udev rules |
| `src/intel_sys_interfaces` | STM32 board messages and services |
| `src/intel_sys_description` | URDF and meshes |
| `src/intel_sys_localization` | Point-LIO and RTAB-Map launch and config, frame adapter, LiDAR body filter |
| `src/intel_sys_navigation` | Nav2 config (STVL costmaps), frontier explorer, tag follower |
| `src/intel_sys_sim` | Gazebo Fortress worlds, simulated L2 and camera, `ros_gz_bridge` |
| `src/intel_sys_bringup` | `sim.launch.py` and the Foxglove layout |
| `scripts/` | `robot.sh` (the robot's stacks), tests, bag recording; `scripts/eval/` evaluates localization |
| `third_party/robot.repos` | Pinned Point-LIO, Unitree SDK and RealSense sources, built into the images |
| `Dockerfile.rover`, `Dockerfile.sim` | Jetson (arm64) and laptop (amd64, with Gazebo) images |
| `docker-compose.yml` | Dev containers: the sim on a laptop, a shell on the Jetson (`run_docker.sh`; GPU and WSL overlays in `docker/`) |
| `docker-compose.robot.yml` | The robot's stacks and the Zenoh router (`scripts/robot.sh`) |

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
scripts/robot.sh up                  # drivers, Foxglove, mapping and exploring (robot standing still)
scripts/robot.sh up follow           # or: tag following
```

In the sim, `sim.launch.py` takes the same modes: `mode:=explore` (default) or `mode:=follow`.

The front of the robot is the sensor side: `+x` and teleop "forward" drive toward the LiDAR and camera.
