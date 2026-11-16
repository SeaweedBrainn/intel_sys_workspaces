# Autonomy Roadmap

Goal: autonomous driving with sensor-adaptive localization: **LVIO** when LiDAR and camera are healthy, falling back to **LIO** (Point-LIO), then **VIO**, then wheel+IMU odometry. Mapping and movement come first; the LiDAR is offline and will return later.

## Current state (branch `ahmad-testing`, from `reorg`)

- Working: STM32 bridge, mecanum controller, wheel odometry, Gazebo sim, Foxglove bridge, Nav2 config with STVL.
- Disabled: STM32 launch (uncommitted change in `hardware.launch.py`), localization and navigation in `robot.launch.py`.
- Missing: any `map` frame (no SLAM), any VIO, any mode selection, a warmup procedure, and a working unit-test run.
- Hardware seen on the Jetson: STM32 at `/dev/rrc` (`ttyACM0`). LiDAR at `192.168.1.2` is offline. RealSense D435 (no IMU) is not enumerated.

## Phases

| # | Phase | Needs hardware | Exit criterion |
|---|---|---|---|
| 1 | Environment: dependencies in the Docker rover image; workspace builds; tests run in the container | none | `scripts/run_tests.sh` passes inside `intel-sys-rover` |
| 2 | Drive: STM32-only bringup; teleop; wheel odom plus STM32 IMU fallback | STM32 | Robot drives 1 m out and back; `/odom` tracks the path within tolerance |
| 3 | Warmup and tests: `preflight.sh`, `warmup.sh`, test tiers, bag recording | STM32 for T4 | Warmup reports PASS/FAIL/SKIP per device and per mode |
| 4 | Map and move without LiDAR: slam_toolbox, Nav2 on a `map` frame, map save/load | none (sim and bags) | Sim: map built, goal reached, map reloads |
| 5 | LIO on the real LiDAR: frames and extrinsics verified | LiDAR | Drift bound met on a 10 m loop |
| 6 | VIO on the D435 plus STM32 IMU: extrinsics calibrated | D435 | VIO odom stable on a loop |
| 7 | Supervisor and LVIO: health-based selection, one `/odom` and `map->odom` | both | Unplug test degrades and recovers without a Nav2 restart |
| 8 | Autonomy on the robot: waypoint runs with recovery | all | Scripted waypoint run passes |

Phases 1 through 4 can proceed now. Phases 5 through 8 are gated on hardware.

## Decisions

- Dependencies live in the Docker rover image (`Dockerfile.rover`), built with `run_docker.sh --build`.
- Mode switching is a supervisor over separate estimators. LIO is Point-LIO, VIO is a new node, and LVIO is a loosely coupled fusion of the two. The supervisor publishes one `/odom`.
- The D435 has no IMU, so VIO uses the STM32 IMU and needs a camera-to-IMU extrinsic.

## Phase 1 details: dependencies

Apt additions to `Dockerfile.rover` (and `Dockerfile.sim` for parity):
- `ros-humble-slam-toolbox`, `ros-humble-robot-localization`, `ros-humble-imu-filter-madgwick`, `ros-humble-nav2-map-server`
- `ros-humble-rosbag2-storage-mcap`, `ros-humble-tf2-tools`
- `python3-pytest`, `ros-humble-launch-testing-ament-cmake`

VIO dependency (decision gate before Phase 6): reserve a pinned entry in `third_party/robot.repos`.
- Preferred: Isaac ROS Visual SLAM (GPU-accelerated on the Orin NX).
- Fallback: OpenVINS, if Isaac's container requirements conflict with this image.

Disk: about 27 GB free, and each image is about 20 GB. Pruning old images (`autonomy-jetson:arm64`, `intel-sys:arm64`) needs approval first.

## TODO

- [ ] **CycloneDDS comparison.** Add `ros-humble-rmw-cyclonedds-cpp` to `Dockerfile.rover` (it is missing from the image, so `scripts/compare_rmw.sh` fails its CycloneDDS leg). Then run loopback and sensor-path latency under Zenoh, CycloneDDS, and FastRTPS, three runs each. Current baseline: Zenoh and FastRTPS are about equal on loopback (p50 ~1.05 ms vs ~1.08 ms).
- [ ] Confirm the drive kinematics. The hardware is modeled as mecanum (four fixed continuous wheels); the wheels may be swerve modules. Lateral and yaw tests depend on the answer.
- [ ] **Subscribers do not re-match after a publisher restart under rmw_zenoh.** Observed: after the STM32 stack was restarted, the dashboard's IMU and odometry subscriptions stayed at 0 Hz until the dashboard itself was restarted. Nav2 and Point-LIO would have the same failure. Investigate the rmw_zenoh session/liveliness behaviour and decide on a fix (or a supervised restart of consumers).
- [ ] **STM32 USB drop is not detected.** The board re-enumerated (`/dev/ttyACM0` recreated) while the bridge kept running on the dead file descriptor and published nothing, with no error. Add a serial liveness watchdog in `ros_robot_controller_node.py` that reopens the port and raises a diagnostic.
- [ ] **Dashboard rosout callback bug (fixed in this branch).** Comparing `Log.level` against `Log.WARN` (bytes) killed the executor. Keep the `spin_error` surface and the regression test.
- [ ] **Battery reading units.** The bridge passes the raw board value through; it reads about 4.1–4.2 V, which looks like one cell rather than a 2S pack. Confirm with a multimeter before trusting it.
- [ ] Wheel odometry is integrated from the commanded speeds, so `response_test.py` validates the software path, not physical motion. Physical motion needs a floor check or encoder feedback.
- [ ] Decide the autonomy gate: no untethered Nav2 run until obstacle sensing (LiDAR or camera) is live.

## Open questions

- Confirm the VIO library choice (Isaac ROS vs OpenVINS).
- Confirm that the STM32 publishes an IMU topic (verify in Phase 2).
- Confirm the map storage path under `/data`.
