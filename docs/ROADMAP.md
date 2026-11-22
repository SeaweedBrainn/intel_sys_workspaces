# Autonomy Roadmap

Goal: autonomous driving on one localization chain: **Point-LIO** (LiDAR-inertial odometry) feeding **RTAB-Map** (loop closure, 3D map, 2D grid) feeding **Nav2**. Start LiDAR-only, then add the camera to RTAB-Map. Reactive exploration (see [Exploration](#exploration-scene-capture)) carries scene capture until this is in place.

> Revised 2026-11-21: the localization plan moved from slam_toolbox + Isaac ROS Visual SLAM + an estimator-switching supervisor to Point-LIO → RTAB-Map → Nav2. See [Decisions](#decisions).

## Current state (branch `slam-chain`, 2026-11-22)

- Working on the robot: STM32 bridge, mecanum controller, rmw_zenoh runtime, tailnet dashboard ([DASHBOARD.md](DASHBOARD.md)), remote SSH access ([REMOTE_ACCESS.md](REMOTE_ACCESS.md)). Reactive LiDAR exploration and waypoint following run in the dashboard on wheel odometry (see [Exploration](#exploration-scene-capture)).
- Working in the sim and on recorded bags, not yet on the robot: Point-LIO → RTAB-Map → Nav2, including mapping, localization mode and the D435 as a second obstacle source. Evidence in [LOCALIZATION_RESULTS.md](LOCALIZATION_RESULTS.md).
- The front is the sensor side (`forward_is_sensor_side`, see [Decisions](#decisions)). Not yet deployed on the Jetson, where teleop "forward" still drives the other way.
- Wheel odometry is estimated from **commanded** wheel speeds (`set_motor`), not encoders. It publishes `/odom/wheel` with TF off.
- `robot.launch.py` still has localization and navigation commented out; on the robot they run as their own compose services (see [Infrastructure](#infrastructure)).
- Hardware on the Jetson: STM32 at `/dev/rrc` (`ttyACM0`), publishing `/imu_raw`. Unitree L2 at `192.168.1.2`, publishing `/lidar/points_raw` at about 12 Hz and its built-in IMU on `/lidar/imu`. The RealSense D435 (no IMU) was not enumerated when last checked.

## Phases

| # | Phase | Needs hardware | Exit criterion |
|---|---|---|---|
| 1 | Environment: dependencies (incl. `rtabmap_ros`) in the Docker rover image; workspace builds; tests run in the container | none | `scripts/run_tests.sh` passes inside `intel-sys-rover` |
| 2 | Drive: STM32-only bringup; teleop; check whether the STM32 can report encoder speeds | STM32 | Robot drives 1 m out and back; encoder feedback available, or ruled out |
| 3 | Warmup and tests: `preflight.sh`, `warmup.sh`, test tiers, bag recording | STM32 for T4 | Warmup reports PASS/FAIL/SKIP per device |
| 4 | Frames: L2 tilt done (gravity-measured `lidar_joint`, 2026-11-21). Facing settled 2026-11-22 (front = sensor side, `forward_is_sensor_side`; the dashboard flip now follows the controller). Remaining: re-check Point-LIO extrinsics with the mechanical offsets | LiDAR | L2 cloud lines up with `base_footprint` in Foxglove; dashboard sector tests pass without `DASHBOARD_LIDAR_FACES_REAR` |
| 5 | LIO: Point-LIO on the real L2 with its built-in IMU; owns `/odom` and `odom→base_footprint` | LiDAR | Drift bound met on a 10 m loop |
| 6 | SLAM: RTAB-Map, LiDAR-only (ICP), with Point-LIO as its odometry; owns `map→odom`; 3D map + 2D grid; database under `/data` | LiDAR (bags OK for development) | A building loop closes; relaunch in localization mode re-localizes on the saved map |
| 7 | Navigation: Nav2 global costmap on RTAB-Map's `/map`, local costmap on `odom` with STVL; dashboard explorer and waypoints use the real pose and send goals to Nav2 | STM32 + LiDAR | Scripted waypoint run with recovery passes |
| 8 | Camera: D435 running, camera↔LiDAR extrinsics calibrated, RGB-D enabled in RTAB-Map for visual loop closures | D435 | Visual loop closures detected; no regression against LiDAR-only |

Phases 1 through 4 can proceed now. Phases 5–7 already run in the sim and on bags ([results](LOCALIZATION_RESULTS.md)). Later, optional: one `robot_localization` filter fusing Point-LIO + IMU + real wheel encoders as a fallback when the LiDAR degrades.

## Next

- [ ] Rebuild the rover image and run the chain on the robot, following [ROBOT_TEST_PLAN.md](ROBOT_TEST_PLAN.md).
- [ ] Lower Nav2's limits for the first robot run (about 0.2 m/s, no reverse, no BackUp recovery): today's values are sim settings and can reverse into the unseen side.
- [ ] Point-LIO on the Jetson: measure CPU and lag with a bag played on the Orin NX (about 30% of a laptop core at 0.1 m voxels; if it falls behind, switch to 0.4 m voxels).
- [ ] Deploy the sensor-side front on the Jetson: controller and dashboard together; tell the team teleop "forward" has flipped.
- [ ] Record a loop that ends exactly where it started (floor mark), with the camera on: measures drift, including height on level ground, and shows whether the camera's loop closures (`slam_camera:=true`) help.
- [ ] Without a camera there is no global relocalization: the robot must start at the map origin or be given `/rtabmap/initialpose`. Add a "set pose" control to the dashboard, or a fixed home spot.
- [ ] Divergence guard for Point-LIO (stop and restart it when its speed exceeds what the robot can do). Low priority: no divergence since the switch to the output model; revisit if it happens on the robot.

## Decisions

- Dependencies live in the Docker rover image (`Dockerfile.rover`), built with `run_docker.sh --build`.
- **One localization chain, no estimator switching.** Point-LIO owns `odom→base_footprint`, RTAB-Map owns `map→odom`, Nav2 consumes both. Switching between separate estimators makes the pose jump, which Nav2 handles badly; fallbacks go into one fused filter instead. (Replaces the supervisor/LVIO plan.)
- **RTAB-Map instead of slam_toolbox.** It takes Point-LIO as external odometry, does 3D LiDAR loop closure, produces a 2D grid for Nav2, and adds the camera later as a config change instead of a new estimator.
- **The camera goes into RTAB-Map as RGB-D, not as a separate VIO.** (Replaces the Isaac ROS Visual SLAM / OpenVINS plan; Isaac VSLAM is a standalone estimator and does not take external odometry.)
- Point-LIO uses the L2's built-in IMU (`/lidar/imu`), so it does not depend on the STM32 IMU.
- Wheel odometry is not fused into anything until it comes from encoder feedback; today's version is derived from commands.
- Each stack runs in its own container, defined as a compose service (see [Infrastructure](#infrastructure)).
- **The front is the sensor side** (2026-11-22). The L2 and the D435 sit together on one end, facing out; that end is +x everywhere: `/cmd_vel` (teleop, dashboard, Nav2), wheel odometry and the URDF. The STM32 firmware is unchanged; `mecanum_controller` and `odom_publisher` negate `vx`, `vy` (`forward_is_sensor_side`, one launch argument in `stm32.launch.py`). The dashboard reads the controller's setting and only flips autonomy for an older controller. Replaces `DASHBOARD_LIDAR_FACES_REAR`.

## Phase 1 details: dependencies

Both Dockerfiles carry `rtabmap-ros`, `robot-localization`, `imu-filter-madgwick`, `rosbag2-storage-mcap`, `tf2-tools`, `python3-pytest` and `launch-testing-ament-cmake`. The sim image is built and `scripts/run_tests.sh` passes in it; the rover image has not been rebuilt, and the arm64 `rtabmap-ros` is untested on the Orin NX.

Disk: about 27 GB free, and each image is about 20 GB. Pruning old images (`autonomy-jetson:arm64`, `intel-sys:arm64`) needs approval first.

## Infrastructure

- [x] **Stacks as compose services.** Done in `docker-compose.robot.yml` (`scripts/robot.sh`); not yet run on the Jetson.
- [x] **Restart policy.** Drivers, description and dashboard `unless-stopped`; localization and navigation started per session.
- [x] **Least privilege.** Only the STM32 stack is privileged; the rest have no device access and a read-only workspace (the dashboard writes).
- [ ] **Host `ros2` CLI.** A plain SSH shell uses FastRTPS and sees nothing. Use `docker exec` into a stack container, or source `docker/zenoh.env` first.
- [ ] **`hardware.launch.py` STM32 line.** Revert the comment-out, or commit it with a note, so `robot.launch.py` does not silently start without motor control.

## Exploration (scene capture)

Reactive exploration drives the rover around so `record_data.sh` captures the scene for offline processing. It runs inside the dashboard node, which stays the only `/cmd_vel` publisher. States and tuning are in [DASHBOARD.md](DASHBOARD.md#autonomous-exploration).

### Findings (2026-11-16)

- The L2 is mounted tilted and only sees one half-plane (azimuth -90° to +90° in its frame). Measured from a live cloud: no returns at all in the other half.
- That half was the rover's **physical rear** under the STM32 motor convention, so the first exploration version drove blind, and the URDF had the LiDAR facing the wrong way.
  - *Resolved 2026-11-22:* the sensor side is now the front everywhere (`forward_is_sensor_side`, see [Decisions](#decisions)); roll and pitch come from measured gravity. The dashboard follows the running controller; `DASHBOARD_LIDAR_FACES_REAR` remains only as a manual override.

### Limitations

| # | Area | Limitation | Impact |
|---|---|---|---|
| E1 | Sensing | Half the surroundings are blind, including the back half of each side sector. | Turning in place and strafing sweep parts of the body through unseen space. Reverse is disabled. |
| E2 | Sensing | Obstacle band is 0.05–0.60 m above the floor. | Cables, sills, drop-offs (stairs, ledges), and overhangs (tables) are not detected. |
| E3 | Sensing | Sector distances are unfiltered. Front read 2.4 → 0.67 → 2.4 m within a second in a live run; a persistent return near 0.2 m front-left may be the robot itself (unverified). | Speed oscillates; false SCAN/TURN triggers. |
| E4 | Pose | Wheel odometry only, integrated from commanded speeds (see TODO below). No SLAM, no loop closure. | Slip is invisible; strafing drifts. |
| E5 | Coverage | The visit grid (0.5 m cells) marks where the robot drove, not what the LiDAR saw. It resets each run. No frontiers, no planning. | "Unvisited" is a rough hint, not a coverage guarantee. |
| E6 | Speed | 0.10 m/s cap; a full LOOK spin takes about 21 s at 0.3 rad/s. | Slow coverage of large spaces. |
| E7 | Frames | ~~The 180° flip is a dashboard patch, not a URDF fix.~~ Resolved 2026-11-22 (sensor side is the front). | — |
| E8 | Safety | E-stop is software only. Live mode is gated by a `TETHERED` string, not a check. | Relies on a person at the robot. |
| E9 | Safety | Autonomy keeps running if the operator's laptop or the network drops. The 0.3 s deadman covers manual drive only. | No stop on link loss. |
| E10 | Operations | Anyone on the tailnet with the token can drive; no lock or ownership. Two people changed modes during one test session. | Conflicting commands; surprise motion. |
| E11 | Sensing | Camera is at 0 Hz. | Bags carry LiDAR only. |
| E12 | Validation | The coverage-aware explorer has unit tests only. | Behaviour on the real robot is unknown. |

### Exploration TODO (priority order)

- [x] **Fix the URDF LiDAR orientation (E7, Phase 4).** Done 2026-11-22 by redefining the rover's front as the sensor side (option two above): tilt from gravity, the URDF keeps the dome at +x, `mecanum_controller`/`odom_publisher` flip the motor convention (`forward_is_sensor_side`), and the dashboard follows the running controller instead of `DASHBOARD_LIDAR_FACES_REAR`. Re-check the Point-LIO extrinsics once the mechanical offsets arrive.
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
- Confirm the map storage path under `/data` (RTAB-Map database).
- Was the L2 tilt calibration (2026-11-21) measured with the robot on level ground? Gravity-based roll and pitch are only as good as the floor they were measured on.
