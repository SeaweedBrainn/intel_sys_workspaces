# Testing and Warmup

Tests are grouped in tiers. Each tier runs only if the previous one passes.

| Tier | What | Where | Command |
|---|---|---|---|
| T0 | Unit tests (`pytest`) | container | `scripts/run_tests.sh` |
| T1 | Launch tests (`launch_testing`) | container | `scripts/run_tests.sh` |
| T2 | Bag replay (localization output checked) | container | `scripts/replay_bag.sh <bag>` (added in Phase 3) |
| T3 | Sim scripted goal run | container | `ros2 launch intel_sys_bringup sim.launch.py headless:=true` |
| T4 | On-robot warmup | robot | `scripts/warmup.sh` |

## Existing tests

- `src/intel_sys_hardware/test/test_mecanum.py`
- `src/intel_sys_hardware/test/test_odom_integration.py`
- `src/intel_sys_hardware/test/test_protocol.py`

These import `rclpy` and the `intel_sys_interfaces` messages. They need a built workspace overlay, not just ROS sourced. The current `install/` tree is stale, so run `colcon build` first.

## Running on the host (not yet verified)

```bash
./run_docker.sh colcon build --symlink-install
./run_docker.sh colcon test
./run_docker.sh colcon test-result --all --verbose
```

## Warmup procedure (Phase 3)

Run in order. Each stage prints PASS, FAIL, or SKIP. A SKIP means the device is offline, not broken.

1. **Preflight (host)**: `scripts/preflight.sh`
   - `/dev/rrc` exists and maps to the STM32
   - Docker image present, disk above a threshold
   - LiDAR ping (`192.168.1.2`), D435 on USB, `/dev/video*`
2. **Serial and IMU (container)**: STM32 handshake, battery reading, IMU rate.
3. **Motors**: robot on blocks, spin each wheel at low speed, then lift the blocks.
4. **Short drive**: 0.5 m forward and back at low speed. Check that wheel odometry returns close to the start.
5. **LiDAR** (if present): points rate, LiDAR IMU rate.
6. **Camera** (if present): RGB and depth rates.
7. **TF**: the expected frames exist (`base_footprint`, `odom`, `imu_link`, `lidar_link`, `camera_link`).
8. **Mode report**: which `localization_mode` is available: `wheel`, `vio`, `lio`, or `lvio`.

## Recording

`record_data.sh` records camera and LiDAR topics with TF. Phase 3 adds `/odom/wheel`, the STM32 IMU, and `/cmd_vel` so bags can drive Phases 4 through 7 offline.

## Pass criteria

- T0 and T1 green in the container before any hardware run.
- T4 warmup with only the STM32 present: stages 1 through 4 PASS, stages 5 and 6 SKIP, mode report shows `wheel`.
