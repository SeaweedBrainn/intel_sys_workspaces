# Testing

| What | Runs where | Entry point |
|---|---|---|
| Preflight: serial, image, disk, LiDAR, camera, clock | Jetson host | `scripts/preflight.sh` |
| Build and unit tests | container | `./run_docker.sh bash scripts/run_tests.sh` |
| Localization on recorded bags | container | `scripts/eval/run_lio_on_bag.sh`, `scripts/eval/run_slam_on_bag.sh` |
| Sim runs (localization, navigation, exploration) | container | `ros2 launch intel_sys_bringup sim.launch.py` ([ROADMAP.md](ROADMAP.md) has the worlds and results) |
| Zenoh restart and heavy-traffic checks | robot | `scripts/eval/zenoh/zenoh_check.sh` |
| On the robot | robot | [ROBOT_TEST_DAY.md](ROBOT_TEST_DAY.md) |

**Repeated sim or bag runs:** kill every ROS process in the container between runs and check none are left; stale nodes corrupt the next run's `/odom`. Run `pkill` as `docker exec <container> pkill -f <pattern>`: inside `bash -c "..."` the pattern also matches that shell's own command line and kills it.

## Preflight

```bash
scripts/preflight.sh
```

Runs on the Jetson host, not in a container. Each check prints PASS, FAIL or SKIP; a SKIP means a device is offline (for example the LiDAR), not broken. It exits nonzero only on FAIL. It checks that `/dev/rrc` resolves to a character device, that the robot image (`intel-sys-rover:latest`) exists, free disk (at least `MIN_FREE_GB`, default 10), LiDAR reachability, the RealSense on USB, `/dev/video*`, and clock sync.

## Build and unit tests

```bash
./run_docker.sh bash scripts/run_tests.sh              # build, then test
./run_docker.sh bash scripts/run_tests.sh --no-build   # test only
```

It builds with `colcon build --symlink-install`, sources `install/setup.bash` (the hardware tests import `rclpy` and the `intel_sys_interfaces` messages, so the overlay must be built and sourced), runs `colcon test`, prints the results, and exits nonzero if anything fails.

Tests live in each package's `test/` directory: `intel_sys_hardware` (STM32 protocol, mecanum mixing, odometry, watchdog, e-stop, health), `intel_sys_localization` (frame adapter) and `intel_sys_navigation` (frontiers, tour planner, tag detection, following, cross-package config). CI (`.github/workflows/tests.yml`) runs the same three packages on pull requests.
