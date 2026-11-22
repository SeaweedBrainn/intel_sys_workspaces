# Autonomy Roadmap

Goal: autonomous driving on one localization chain: **Point-LIO** (LiDAR-inertial odometry) feeding **RTAB-Map** (loop closure, 3D map, 2D grid) feeding **Nav2**. Start LiDAR-only, then add the camera to RTAB-Map. Reactive exploration (see [Exploration](#exploration-scene-capture)) carries scene capture until this is in place.

> Revised 2026-11-21: the localization plan moved from slam_toolbox + Isaac ROS Visual SLAM + an estimator-switching supervisor to Point-LIO → RTAB-Map → Nav2. See [Decisions](#decisions).

## Current state (branch `ahmad-zenoh`, updated 2026-11-21)

- Working: STM32 bridge, mecanum controller, Gazebo sim, Foxglove bridge, Nav2 config with STVL, rmw_zenoh runtime, tailnet dashboard ([DASHBOARD.md](DASHBOARD.md)), remote SSH access ([REMOTE_ACCESS.md](REMOTE_ACCESS.md)).
- Working, not yet field-proven: reactive LiDAR exploration in the dashboard (`scripts/autonomy/explorer.py`). The first version ran live; the coverage-aware version (visit grid, SCAN passes) is deployed but has not run live. Waypoint following (`scripts/autonomy/waypoint.py`) is committed (`a25ad87`).
- Dashboard map and 3D view (`waypoint.py`, `frames.py`, committed in `a25ad87`): a 2D hit-count grid (10 cm cells) plus an odometry trail, and a 3D LiDAR view. Clouds are gravity-levelled from the L2 IMU; cells and points expire after 60 s. It is placed using the commanded wheel odometry, so it is a live operator view, not SLAM: no scan matching, no loop closure, no free space, not saved.
- Wheel odometry is estimated from **commanded** wheel speeds (`set_motor`), not encoders. It publishes `/odom/wheel` with TF off.
- Nothing publishes `/odom` or `map`. Localization and navigation are commented out in `robot.launch.py`.
- Runtime on the Jetson: separate containers from `intel-sys-rover:latest`: `intel-sys-zenohd` (compose, `restart: unless-stopped`), plus `stm32stack`, `lidarstack`, `robot-dashboard` started by hand with `docker run` (`restart: no`). `hardware.launch.py` has `stm32_launch` commented out (uncommitted); the STM32 stack is launched on its own via `stm32.launch.py`.
- Hardware on the Jetson: STM32 at `/dev/rrc` (`ttyACM0`), publishing `/imu_raw`. Unitree L2 at `192.168.1.2`, publishing `/lidar/points_raw` at about 12 Hz and its built-in IMU on `/lidar/imu`. Camera topic `/camera/image_raw` is at 0 Hz (no camera node running); the RealSense D435 (no IMU) was not enumerated when last checked.

## Phases

| # | Phase | Needs hardware | Exit criterion |
|---|---|---|---|
| 1 | Environment: dependencies (incl. `rtabmap_ros`) in the Docker rover image; workspace builds; tests run in the container | none | `scripts/run_tests.sh` passes inside `intel-sys-rover` |
| 2 | Drive: STM32-only bringup; teleop; check whether the STM32 can report encoder speeds | STM32 | Robot drives 1 m out and back; encoder feedback available, or ruled out |
| 3 | Warmup and tests: `preflight.sh`, `warmup.sh`, test tiers, bag recording | STM32 for T4 | Warmup reports PASS/FAIL/SKIP per device |
| 4 | Frames: L2 tilt done (gravity-measured `lidar_joint`, 2026-11-21). Remaining: settle the facing direction (it faces the rear) in the URDF, remove the dashboard's 180° flip, re-check Point-LIO extrinsics | LiDAR | L2 cloud lines up with `base_footprint` in Foxglove; dashboard sector tests pass without `DASHBOARD_LIDAR_FACES_REAR` |
| 5 | LIO: Point-LIO on the real L2 with its built-in IMU; owns `/odom` and `odom→base_footprint` | LiDAR | Drift bound met on a 10 m loop |
| 6 | SLAM: RTAB-Map, LiDAR-only (ICP), with Point-LIO as its odometry; owns `map→odom`; 3D map + 2D grid; database under `/data` | LiDAR (bags OK for development) | A building loop closes; relaunch in localization mode re-localizes on the saved map |
| 7 | Navigation: Nav2 global costmap on RTAB-Map's `/map`, local costmap on `odom` with STVL; dashboard explorer and waypoints use the real pose and send goals to Nav2 | STM32 + LiDAR | Scripted waypoint run with recovery passes |
| 8 | Camera: D435 running, camera↔LiDAR extrinsics calibrated, RGB-D enabled in RTAB-Map for visual loop closures | D435 | Visual loop closures detected; no regression against LiDAR-only |

Phases 1 through 4 can proceed now. The chain for Phases 5–7 already runs in the Gazebo sim; see [Simulation results](#simulation-results-branch-slam-chain-2026-11-21). Phase 6 can be developed on recorded bags once Phase 5 runs. Later, optional: one `robot_localization` filter fusing Point-LIO + IMU + real wheel encoders as a fallback when the LiDAR degrades.

## Simulation results (branch `slam-chain`, 2026-11-21)

The whole chain runs in Gazebo on the laptop (`intel-sys-sim`, amd64): `ros2 launch intel_sys_bringup sim.launch.py world:=loop.sdf x:=-6.5 y:=0.0` (SLAM is on by default in the sim bringup; off by default in `robot.launch.py`). `scripts/slam_loop_test.py` drives a lap through Nav2 and scores the pose against Gazebo ground truth.

- **Phase 1 (sim image):** dependencies added to both Dockerfiles; `scripts/run_tests.sh` passes in `intel-sys-sim` (23 tests). The rover image has not been rebuilt.
- **Mapping lap (Phase 6, first half): PASS.** About 40 m around `loop.sdf`. SLAM error stayed at 6–10 cm, grew to 48 cm by the last corner, and one proximity closure brought it to 7 cm at the end. Point-LIO alone drifted about 0.9 m.
- **Localization mode (Phase 6, second half): starts correctly, lap not passing yet.** `slam_localization:=true database_path:=...` loads the saved map and starts localized (5 mm). In the south corridor Point-LIO drifts about 1.4 m and localization mode does not correct it until the next corner, so the robot overshoots into the east wall.
- **Fixed along the way** (each would also have hit the robot):
  - Point-LIO adapter applied the LiDAR mount tilt twice (`base_footprint` pitched 84°).
  - Point-LIO `path_en` republished the full path every update; odometry fell 100 s behind within minutes, TF went stale and Nav2 stalled.
  - No `velocity_smoother` section: Nav2's defaults clamp `vy` to 0, silently dropping mecanum strafing, so the robot drifted off its path.
  - Static global costmap shrank to the explored map, so goals in unexplored space never planned; it is now a rolling window in `map`.
  - Point-LIO's vertical drift lifted the floor into STVL as phantom obstacles; `odom→base_footprint` is now planar (x, y, yaw; REP-120), `planar: false` keeps 6-DoF.
  - RTAB-Map localization mode assumed the robot restarted at the last mapped pose; `RGBD/StartAtOrigin` is set.

Next in the sim:
- [ ] Point-LIO drift in long corridors (~1 m over 13 m). Check whether it is the sim LiDAR (18 rings, about 3,800 points per scan; the real L2 is much denser) or the Point-LIO parameters; record a real L2 bag down a corridor to compare.
- [ ] Localization mode: localize more often or accept larger corrections (`Rtabmap/DetectionRate`, `Icp/MaxCorrespondenceDistance`, `RGBD/LocalRadius`) so drift is pulled back mid-corridor, then pass the lap.
- [ ] Without a camera there is no global relocalization: the robot must start at the map origin or be given `/rtabmap/initialpose`. Add a "set pose" control to the dashboard, or a fixed home spot.

## Decisions

- Dependencies live in the Docker rover image (`Dockerfile.rover`), built with `run_docker.sh --build`.
- **One localization chain, no estimator switching.** Point-LIO owns `odom→base_footprint`, RTAB-Map owns `map→odom`, Nav2 consumes both. Switching between separate estimators makes the pose jump, which Nav2 handles badly; fallbacks go into one fused filter instead. (Replaces the supervisor/LVIO plan.)
- **RTAB-Map instead of slam_toolbox.** It takes Point-LIO as external odometry, does 3D LiDAR loop closure, produces a 2D grid for Nav2, and adds the camera later as a config change instead of a new estimator.
- **The camera goes into RTAB-Map as RGB-D, not as a separate VIO.** (Replaces the Isaac ROS Visual SLAM / OpenVINS plan; Isaac VSLAM is a standalone estimator and does not take external odometry.)
- Point-LIO uses the L2's built-in IMU (`/lidar/imu`), so it does not depend on the STM32 IMU.
- Wheel odometry is not fused into anything until it comes from encoder feedback; today's version is derived from commands.
- Each stack runs in its own container, defined as a compose service (see [Infrastructure](#infrastructure)).

## Phase 1 details: dependencies

Apt additions to `Dockerfile.rover` (and `Dockerfile.sim` for parity):
- `ros-humble-rtabmap-ros` (confirm the arm64 package runs on the Orin NX)
- `ros-humble-robot-localization` (later fallback filter), `ros-humble-imu-filter-madgwick`
- `ros-humble-rosbag2-storage-mcap`, `ros-humble-tf2-tools`
- `python3-pytest`, `ros-humble-launch-testing-ament-cmake`

Disk: about 27 GB free, and each image is about 20 GB. Pruning old images (`autonomy-jetson:arm64`, `intel-sys:arm64`) needs approval first.

## Infrastructure

- [ ] **Stacks as compose services.** Add `stm32`, `lidar` (camera later), `localization` (Point-LIO + RTAB-Map), `navigation` and `dashboard` services alongside `zenohd`, sharing one block of common settings (image, host network/IPC, `docker/zenoh.env`, workspace mount). Each service runs one launch file. Replaces the hand-typed `docker run` commands.
- [ ] **Restart policy.** `restart: unless-stopped` and `depends_on: zenohd` on every service so the robot comes back after a reboot or crash. Until the rmw_zenoh resubscribe bug (TODO below) is fixed, restart in order: stm32 → lidar → dashboard.
- [ ] **Least privilege.** Drop `privileged` from the dashboard (it is the tailnet-exposed one); give hardware containers only the devices they need (`/dev/rrc`, etc.).
- [ ] **Host `ros2` CLI.** A plain SSH shell uses FastRTPS and sees nothing. Use `docker exec` into a stack container, or source `docker/zenoh.env` first.
- [ ] **`/odom` ownership.** Nav2 expects `/odom` and an `odom` frame; today only `/odom/wheel` exists with TF off. Point-LIO takes this over in Phase 5.
- [ ] **`hardware.launch.py` STM32 line.** Revert the comment-out, or commit it with a note, so `robot.launch.py` does not silently start without motor control.
- [ ] **Push the branch.** `ahmad-zenoh` has no upstream; its commits exist only on the Jetson.

## Exploration (scene capture)

Reactive exploration drives the rover around so `record_data.sh` captures the scene for offline processing. It runs inside the dashboard node, which stays the only `/cmd_vel` publisher. States and tuning are in [DASHBOARD.md](DASHBOARD.md#autonomous-exploration).

### Findings (2026-11-16)

- The L2 is mounted tilted and only sees one half-plane (azimuth -90° to +90° in its frame). Measured from a live cloud: no returns at all in the other half.
- That seen half is the rover's **physical rear**: a `+vx` on `/cmd_vel` drives away from what the LiDAR sees. The first exploration version drove blind because of this. The dashboard now rotates autonomy commands 180° (`DASHBOARD_LIDAR_FACES_REAR=1`, the default).
- The URDF still puts the LiDAR on the front (`lidar_joint`: xyz 0.0959 0 0.1284, rpy 0 -pi/2 pi). Anything that uses `base_link` (Nav2, Point-LIO extrinsics, RTAB-Map, offline processing of bags) inherits the wrong orientation.
  - *Update 2026-11-21 (`a25ad87`):* roll and pitch now come from measured gravity (`rpy 1.868822 -1.219393 1.308338`, previously `0 -pi/2 pi`). The facing direction was left as it was, so the dashboard's 180° flip is still needed.

### Limitations

| # | Area | Limitation | Impact |
|---|---|---|---|
| E1 | Sensing | Half the surroundings are blind, including the back half of each side sector. | Turning in place and strafing sweep parts of the body through unseen space. Reverse is disabled. |
| E2 | Sensing | Obstacle band is 0.05–0.60 m above the floor. | Cables, sills, drop-offs (stairs, ledges), and overhangs (tables) are not detected. |
| E3 | Sensing | Sector distances are unfiltered. Front read 2.4 → 0.67 → 2.4 m within a second in a live run; a persistent return near 0.2 m front-left may be the robot itself (unverified). | Speed oscillates; false SCAN/TURN triggers. |
| E4 | Pose | Wheel odometry only, integrated from commanded speeds (see TODO below). No SLAM, no loop closure. | Slip is invisible; strafing drifts. |
| E5 | Coverage | The visit grid (0.5 m cells) marks where the robot drove, not what the LiDAR saw. It resets each run. No frontiers, no planning. | "Unvisited" is a rough hint, not a coverage guarantee. |
| E6 | Speed | 0.10 m/s cap; a full LOOK spin takes about 21 s at 0.3 rad/s. | Slow coverage of large spaces. |
| E7 | Frames | The 180° flip is a dashboard patch, not a URDF fix (see Findings). | Other consumers of `base_link` are wrong. |
| E8 | Safety | E-stop is software only. Live mode is gated by a `TETHERED` string, not a check. | Relies on a person at the robot. |
| E9 | Safety | Autonomy keeps running if the operator's laptop or the network drops. The 0.3 s deadman covers manual drive only. | No stop on link loss. |
| E10 | Operations | Anyone on the tailnet with the token can drive; no lock or ownership. Two people changed modes during one test session. | Conflicting commands; surprise motion. |
| E11 | Sensing | Camera is at 0 Hz. | Bags carry LiDAR only. |
| E12 | Validation | The coverage-aware explorer has unit tests only. | Behaviour on the real robot is unknown. |

### Exploration TODO (priority order)

- [ ] **Fix the URDF LiDAR orientation (E7, Phase 4).** Tilt done 2026-11-21 (gravity-measured); the facing direction remains. Either correct `lidar_joint` to the measured mount (the seen half then becomes base_footprint `-x`, and the dashboard sector math and the flip need updating together), or redefine the rover's front as the LiDAR side. Pick one so there is a single source of truth, then remove `DASHBOARD_LIDAR_FACES_REAR` and re-check the Point-LIO extrinsics.
- [ ] **Link-loss stop for autonomy (E9).** Require a heartbeat from the operator page (or the Mac) while `live`; drop to `off` and publish zeros if it lapses for about 1 s.
- [ ] **Filter sector distances (E3).** Median over the last 3–5 clouds plus hysteresis on the SCAN/TURN thresholds. Check the 0.2 m front-left return against the chassis and enlarge the body mask if it is self-hit.
- [ ] **First recorded live run of the coverage-aware explorer (E12).** 3–5 min with `./record_data.sh -s local`; save `autonomy.stats` to `results/`; tune `look_every_m`, `look_speed`, `scan_len`, and `scan_standoff` from it.
- [ ] **Operator lock (E10).** One driver at a time: the first client to arm autonomy or drive holds a lease; others get read-only status.
- [ ] **Camera in the bags (E11).** Bring up the camera node before recording.
- [ ] **Low obstacles and drop-offs (E2).** Lower `MIN_Z`, add a floor-plane check for missing ground ahead, or add a downward sensor.
- [ ] **Real pose and coverage (E4, E5).** Switch the explorer and waypoint follower to the Point-LIO / RTAB-Map pose (Phases 5–6), mark the grid from what the LiDAR observed, then switch heading choice to frontiers. This replaces the visit grid. The dashboard map and 3D view are the first things to switch to the Point-LIO pose; once they do, the 60 s expiry can be lengthened or dropped.
- [ ] **Hardware e-stop (E8).** A physical or radio stop that cuts the motors independent of software, before any unattended run.

## TODO

- [ ] **CycloneDDS comparison.** Add `ros-humble-rmw-cyclonedds-cpp` to `Dockerfile.rover` (it is missing from the image, so `scripts/compare_rmw.sh` fails its CycloneDDS leg). Then run loopback and sensor-path latency under Zenoh, CycloneDDS, and FastRTPS, three runs each. Current baseline: Zenoh and FastRTPS are about equal on loopback (p50 ~1.05 ms vs ~1.08 ms).
- [x] Drive kinematics: mecanum (confirmed by the team, 2026-11-16). Exploration uses lateral motion (`vy`).
- [ ] **Subscribers do not re-match after a publisher restart under rmw_zenoh.** Observed: after the STM32 stack was restarted, the dashboard's IMU and odometry subscriptions stayed at 0 Hz until the dashboard itself was restarted. Nav2, Point-LIO and RTAB-Map would have the same failure. Investigate the rmw_zenoh session/liveliness behaviour and decide on a fix (or a supervised restart of consumers).
- [ ] **STM32 USB drop is not detected.** The board re-enumerated (`/dev/ttyACM0` recreated) while the bridge kept running on the dead file descriptor and published nothing, with no error. Add a serial liveness watchdog in `ros_robot_controller_node.py` that reopens the port and raises a diagnostic.
- [ ] **Dashboard rosout callback bug (fixed in this branch).** Comparing `Log.level` against `Log.WARN` (bytes) killed the executor. Keep the `spin_error` surface and the regression test.
- [ ] **Battery reading units.** The bridge passes the raw board value through. It read about 4.1–4.2 V earlier; on 2026-11-16 it read 12.1–12.3 V (`battery_mv` 12118–12320). Confirm with a multimeter which is right before trusting it.
- [ ] **Wheel odometry is integrated from the commanded speeds** (`odom_publisher` subscribes to `set_motor`, the command topic), so `response_test.py` validates the software path, not physical motion. Check whether the STM32 SDK can report encoder speeds; if so, publish them and switch `odom_publisher` to that topic. Its covariances (1e-3) are also too confident for fusion.
- [ ] Decide the autonomy gate: no untethered Nav2 run until obstacle sensing (LiDAR or camera) is live. Reactive exploration has run untethered at 0.10 m/s with a person at the robot; write down the rule for when that is allowed.

## Open questions

- Does `rtabmap_ros` (arm64) run fast enough on the Orin NX alongside Point-LIO? Measure CPU load in Phase 6.
- Can the STM32 report encoder speeds (Phase 2)?
- Which way does the D435 face? It helps RTAB-Map most if it covers the half the L2 cannot see.
- Confirm the map storage path under `/data` (RTAB-Map database).
- Was the L2 tilt calibration (2026-11-21) measured with the robot on level ground? Gravity-based roll and pitch are only as good as the floor they were measured on.
