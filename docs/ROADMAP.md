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

## Open questions

- Confirm the VIO library choice (Isaac ROS vs OpenVINS).
- Confirm that the STM32 publishes an IMU topic (verify in Phase 2).
- Confirm the map storage path under `/data`.
