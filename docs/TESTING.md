# Testing, Warmup, and Latency

Tests are grouped in tiers. Run a tier only after the one below it passes.

| Tier | What | Runs where | Entry point |
|---|---|---|---|
| Host | Preflight: serial, image, disk, LiDAR, RealSense, clock | Jetson host | `scripts/preflight.sh` |
| T0 | Unit tests (`pytest` via colcon) | container | `scripts/run_tests.sh` |
| T1 | Launch tests (`launch_testing`) | container | `scripts/run_tests.sh` |
| T2 | Bag replay (localization output checked) | container | added in Phase 3 |
| T3 | Sim scripted goal run | container | `ros2 launch intel_sys_bringup sim.launch.py headless:=true` |
| T4 | On-robot warmup (read-only) | container, robot | `scripts/warmup.sh` |
| T4m | Speed-capped drive test (moves the robot) | container, robot | `scripts/drive_test.sh --confirm-wheels-free` |
| Perf | Latency and responsiveness, per RMW | container | `scripts/compare_rmw.sh` |

Run everything from the host with `./run_docker.sh bash scripts/<name>.sh`, except `preflight.sh`, which runs on the host directly.

## Host preflight

```bash
scripts/preflight.sh
```

Each check prints PASS, FAIL, or SKIP. A SKIP means a device is offline (for example the LiDAR at `192.168.1.2`), not broken. The script exits nonzero only on FAIL.

Checks: `/dev/rrc` resolves to a character device, the `intel-sys-rover:latest` image exists, free disk is at least `MIN_FREE_GB` (default 10), LiDAR reachability, Intel USB devices (RealSense), `/dev/video*`, and clock sync.

## T0 and T1: build and test

```bash
./run_docker.sh bash scripts/run_tests.sh              # build, then test
./run_docker.sh bash scripts/run_tests.sh --no-build   # test only
```

The script builds with `colcon build --symlink-install --parallel-workers 6`, sources `install/setup.bash`, runs `colcon test`, prints `colcon test-result --verbose`, and exits nonzero if any stage fails.

**Overlay requirement:** the hardware unit tests (`src/intel_sys_hardware/test/`) import `rclpy` and the `intel_sys_interfaces` messages. They fail unless the workspace overlay is built and sourced. That is why the script sources `install/setup.bash` before `colcon test`. The `install/` tree on the host is stale, so the first run must build.

Package tests are in `src/intel_sys_hardware/test/` and `src/intel_sys_localization/test/`; the script also runs the dashboard and autonomy tests in `scripts/` with `pytest`. CI (`.github/workflows/tests.yml`) runs the same tests on pull requests.

## T4 warmup (read-only)

```bash
./run_docker.sh bash scripts/warmup.sh
```

Stages, each PASS, FAIL, or SKIP:

| Stage | Check | SKIP when |
|---|---|---|
| a | `/dev/rrc` visible in the container; `ros_robot_controller` node starts (the script starts `stm32_bridge` if it is not already running) | never; missing serial is FAIL |
| b | `imu_raw` rate above 50 Hz over a 3 s window | bridge not running |
| c | `/cmd_vel` and `/set_motor` topics exist. Commands are not sent. | mecanum controller not running |
| d | TF resolves `base_footprint -> imu_link` and `odom -> base_footprint` | robot_state_publisher or odometry source absent |
| e | `/lidar/points_raw` and `/camera/image_raw` rates | topic absent |
| f | Mode report: lists `wheel`, plus `lio`, `vio`, or `lvio` as sensors allow | never (INFO line) |

Stage e and the mode report must agree: `lio` needs the LiDAR, `vio` needs the camera, and `lvio` needs both.

## T4m drive test (moves the robot)

```bash
./run_docker.sh bash scripts/drive_test.sh --confirm-wheels-free
./run_docker.sh bash scripts/drive_test.sh --confirm-wheels-free --speed 0.10 --duration 1.5
```

Safety:
- Refuses to run without `--confirm-wheels-free`.
- Hard caps: speed at most 0.15 m/s, duration at most 2 s. Values above the caps are rejected, not clamped.
- A trap publishes zero `Twist` on any exit (normal, error, Ctrl-C, or SIGTERM), and sends the stop three times.
- Refuses to start if `/odom/wheel` has no message, so it never drives blind.

Pass: `/odom/wheel` displacement along x is between 50% and 150% of `speed × duration`. Run it with the robot suspended or on blocks first.

## Latency and responsiveness

The probe is `scripts/latency_probe/latency_probe.py`. It writes JSON, so runs can be compared.

| Mode | Measures | Notes |
|---|---|---|
| `stamp` | `receive_time − header.stamp` on an existing topic, plus rate | Sensor-to-application latency. Depends on the sensor's clock; the LiDAR is configured to use system timestamps. |
| `loopback` | Round trip of a `PoseStamped` published and subscribed in one process, through the RMW | Pure middleware latency. No hardware needed. |
| `responsiveness` | Time from a zero `Twist` on `--cmd-topic` to the first message on `--echo-topic` after it | Upper bound: includes the controller's loop period. Publishes commands, so use it only with the wheels free. |

Output fields: `samples`, `rate_hz`, `p50_ms`, `p95_ms`, `p99_ms`, `max_ms`, `mean_ms`, plus `rmw`, `topic`, and `timestamp`. The probe exits nonzero if it collected no samples.

Examples:

```bash
python3 scripts/latency_probe/latency_probe.py loopback --rate 100 --seconds 10 --out results/loop.json
python3 scripts/latency_probe/latency_probe.py stamp --topic /imu_raw --type sensor_msgs/msg/Imu --seconds 10 --out results/imu.json
python3 scripts/latency_probe/latency_probe.py responsiveness --cmd-topic /cmd_vel \
  --echo-topic /odom/wheel --type nav_msgs/msg/Odometry --rate 10 --seconds 10 --out results/resp.json
```

### Comparing RMW implementations

```bash
./run_docker.sh bash scripts/compare_rmw.sh loopback --rate 100
./run_docker.sh bash scripts/compare_rmw.sh stamp --topic /imu_raw --type sensor_msgs/msg/Imu
```

`compare_rmw.sh` runs the same probe twice, once under `rmw_zenoh_cpp` with the environment from `docker/zenoh.env` and once under `rmw_cyclonedds_cpp`. It writes one JSON per run to `results/` (the directory is committed with a `.gitkeep`; the JSON files are not meant to be committed) and prints a side-by-side p50/p95/p99/max summary. It exits nonzero if either leg fails. If `docker/zenoh.env` is missing, the zenoh leg fails and the cyclonedds leg still runs.

Run comparisons with the same topic, rate, and duration for both legs, and with the same system load. Repeat at least three times before drawing conclusions from a single number.

## Recording

`record_data.sh` records camera and LiDAR topics with TF. Phase 3 adds `/odom/wheel`, the STM32 IMU (`imu_raw`), and `/cmd_vel`, so offline bags can drive Phases 4 through 7.

## Pass criteria

- Host preflight: no FAIL.
- T0 and T1: `run_tests.sh` exits 0 in the container.
- T4 warmup with only the STM32 present: stages a, b, and d pass; stages c, e, and f report SKIP or INFO as described above.
- T4m: passes only after the warmup passes, with the wheels free.
- Perf: loopback round trip is measured on both RMWs, with no FAIL legs.
