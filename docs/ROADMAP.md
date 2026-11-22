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
| 4 | Frames: L2 tilt done (gravity-measured `lidar_joint`, 2026-11-21). Facing settled 2026-11-22 (front = sensor side, `forward_is_sensor_side`; the dashboard flip now follows the controller). Remaining: re-check Point-LIO extrinsics with the mechanical offsets | LiDAR | L2 cloud lines up with `base_footprint` in Foxglove; dashboard sector tests pass without `DASHBOARD_LIDAR_FACES_REAR` |
| 5 | LIO: Point-LIO on the real L2 with its built-in IMU; owns `/odom` and `odom→base_footprint` | LiDAR | Drift bound met on a 10 m loop |
| 6 | SLAM: RTAB-Map, LiDAR-only (ICP), with Point-LIO as its odometry; owns `map→odom`; 3D map + 2D grid; database under `/data` | LiDAR (bags OK for development) | A building loop closes; relaunch in localization mode re-localizes on the saved map |
| 7 | Navigation: Nav2 global costmap on RTAB-Map's `/map`, local costmap on `odom` with STVL; dashboard explorer and waypoints use the real pose and send goals to Nav2 | STM32 + LiDAR | Scripted waypoint run with recovery passes |
| 8 | Camera: D435 running, camera↔LiDAR extrinsics calibrated, RGB-D enabled in RTAB-Map for visual loop closures | D435 | Visual loop closures detected; no regression against LiDAR-only |

Phases 1 through 4 can proceed now. The chain for Phases 5–7 already runs in the Gazebo sim; see [Simulation and bag results](#simulation-and-bag-results-branch-slam-chain-2026-11-22). Phase 6 can be developed on recorded bags once Phase 5 runs. Later, optional: one `robot_localization` filter fusing Point-LIO + IMU + real wheel encoders as a fallback when the LiDAR degrades.

## Simulation and bag results (branch `slam-chain`, 2026-11-22)

The whole chain runs in Gazebo on the laptop (`intel-sys-sim`, amd64, rendering on the NVIDIA GPU under WSL via `docker-compose.wsl.yml`): `ros2 launch intel_sys_bringup sim.launch.py world:=loop.sdf x:=-6.5 y:=0.0` (SLAM is on by default in the sim bringup; off by default in `robot.launch.py`). Tools: `scripts/eval/slam_loop_test.py` (Nav2 lap scored against Gazebo truth), `scripts/eval/lio_drift_test.py` (straight corridor legs, Point-LIO only), `scripts/eval/run_lio_on_bag.sh` (Point-LIO on a recorded bag), `scripts/eval/bag_mount_check.py` (L2 mount from a bag's IMU gravity).

**Phase 6 exit criterion met in simulation:**
- **Mapping lap: PASS.** About 40 m around `loop.sdf`: Point-LIO alone ends 0.8 cm from ground truth; SLAM (map frame) stays within 8 cm; 4 proximity closures.
- **Localization mode: PASS.** Relaunched with `slam_localization:=true database_path:=...` on the saved map: re-localized 103 times, within 4.6 cm for the whole lap.
- **Phase 1 (sim image):** dependencies in both Dockerfiles; `scripts/run_tests.sh` passes in `intel-sys-sim` (23 tests). The rover image has not been rebuilt.

**Real L2 bags** (Hemang's recordings, three mounts plus half of a hallway run; not in git, see `bags/`):
- Mounts from the L2's IMU gravity: orientation 1 ("0°") has the dome 83° from vertical, i.e. facing horizontally, matching the current URDF; orientation 2 ("45°") 39°; orientation 3 ("90°") 0.7°, i.e. dome pointing straight up. Which way the dome faces horizontally is not given by gravity.
- Point-LIO keeps up in real time on all of them (0.02 s behind, ~30% of a core) and tracks plausibly: the 306 s hallway run (orientation 1) gives a clean three-corridor map over 92 m. Its height rises ~1.2 m, but the building has slopes and none of the bags passes the same spot twice, so height drift can't be measured from them (navigation ignores height anyway: planar odom, RTAB-Map 3-DoF).
- The L2's accelerometer reads 9.67–10.02 m/s² at rest depending on which axis faces down (uncalibrated MEMS bias/scale). Setting Point-LIO's `acc_norm` to the reading made tracking much worse (one bag diverged), so keep 9.81 and let Point-LIO's bias estimate absorb it.
- The short orientation-1 bag diverges from its start: a jolt plus ~0.2 s of missing IMU data 2 s in, while Point-LIO's map is still tiny. Started 3 s in, it tracks fine. **Start Point-LIO with the robot still.**
- Real L2 data: ~5,300 points per scan at 12 Hz, genuine per-point times over a ~0.08 s sweep, `ring` always 1.

**Fixed along the way** (each would also have hit the robot):
- Point-LIO adapter applied the LiDAR mount tilt twice (`base_footprint` pitched 84°).
- Point-LIO published odometry + TF after every point (`publish_odometry_without_downsample`): on real L2 data it fell behind real time by ~0.5 s per second at full CPU, and diverged when far behind. Now once per scan. This Point-LIO port ignores `point_filter_num` for the L2.
- Point-LIO `path_en` republished the full path every update; odometry fell 100 s behind within minutes.
- No `velocity_smoother` section: Nav2's defaults clamp `vy` to 0, silently dropping mecanum strafing.
- Static global costmap shrank to the explored map, so goals in unexplored space never planned; it is now a rolling window in `map`.
- Point-LIO's vertical drift lifted the floor into STVL as phantom obstacles; `odom→base_footprint` is now planar (x, y, yaw; REP-120), `planar: false` keeps 6-DoF.
- RTAB-Map localization mode assumed the robot restarted at the last mapped pose; `RGBD/StartAtOrigin` is set.
- Sim only: the simulated L2 fired a fixed 18-ring grid (Point-LIO stuck in corridors) and faked a 0.1 s per-point sweep (Point-LIO ~7% short). Now a dense grid thinned at random to 6,400 points per scan, with a 1 ms spread.

**Odometry and camera comparisons (2026-11-22, overnight):**
- **RTAB-Map's `icp_odometry` vs Point-LIO** (`odometry:=icp` in `localization.launch.py`): on the smooth bags (o1, o2) the two agree at 1 Hz (29.1 vs 28.5 m, 23.8 vs 23.2 m), but ICP jitters frame to frame. On the dome-up bag and the hallway ICP fails (128 and 760 impossible jumps; in long corridors it overshoots, resets and oscillates); Point-LIO has none. In the sim corridor ICP registers no forward motion at all. **Keep Point-LIO.**
- **Point-LIO in the sim, straight 6-12 m legs at 0.3 m/s:** 5-15% short with the IMU-as-input model, yet within centimetres in the Nav2 laps. Solved, see below.
- **Point-LIO diverged intermittently** with the input model: the same bag and settings diverged in 1 of 3 runs (dome-up bag), 2 of 4 from the orientation-1 start, and on o1 and o2 again in the comparison below.

**Point-LIO switched to the output model (`use_imu_as_input: false`, 2026-11-22):**
- Cause, from the Point-LIO source: with IMU as input, velocity is only corrected through its correlation with position (the LiDAR update has no velocity term) and process noise is scaled by dt², so the filter is overconfident in its speed. In a corridor the walls give no constraint along the direction of travel and at constant speed the accelerometer sees only gravity, so an early speed error persists. Turns move the error onto an axis the walls constrain, which is why the laps were accurate. Unitree's own L2 config (`point_lio_unilidar`) uses the output model.
- Sim straight legs (4 x 12 m, clean runs): input model 5.5-8.5% short (two runs); output model 0.2%; input model with `acc_cov_input` 3.0 0.3-0.4%; Unitree's full config (output model, 0.4 m voxels, blind 0.5) 0.1-0.2%; `plane_thr` 0.05 worse (9-13%).
- Real bags: the input model diverged on o1 and o2 (5 m/s speeds) and drew the dome-up bag's walls doubled; the output model gave single walls, never diverged in 16 runs (each short bag 4 times, including the shaky o1 start that diverged before) and ends within 0.3 m of Unitree's config on every bag. Real-time lag unchanged (0.02-0.03 s).
- Nav2 mapping lap: passes (Point-LIO ends 0.5-1.5 cm from truth, SLAM within 8 cm). RTAB-Map's proximity closures vary from lap to lap with either model (0, 3 and 5 seen), so the test's "at least one closure" check can fail by chance.
- Unitree's other L2 settings, each on top of the output model (sim legs; 4 bags; CPU = Point-LIO on the laptop, % of one core):
  - 0.4 m voxels (`filter_size_surf`/`map`, ours 0.1): sim legs 0.1% (vs 0.3%), Nav2 lap passes (5 closures). On the bags the smoothed paths agree with 0.1 m within 2.5% and no divergence, but pose jitter is 2.2-2.4 cm RMS vs 1.5-1.8 cm, and CPU 23% vs 28-35%. **Kept at 0.1**; 0.4 is the fallback if Point-LIO can't keep up on the Jetson.
  - `blind` 0.5 (ours 0.1): no accuracy gain, diverged once from the shaky o1 start. **Kept at 0.1**: the body filter already removes the chassis.
- Test-harness pitfall: `kill -INT` on `ros2 launch` from a script did not stop its nodes; stale `point_lio_frame_adapter`s anchored on an earlier run corrupted `/odom` in later runs. Kill every ROS process and check none are left between runs.
- **Visual-only RTAB-Map** (D435 RGB-D): visual odometry loses track on these hand-carried bags (84-88% of frames with defaults, 37% with reset-on-loss and optical flow) and jumps at every reset; not usable as odometry. It does find visual loop closures (7 on the dome-up bag).
- **Phase 8 wiring done:** `slam_camera:=true` adds the D435 to RTAB-Map (visual loop closures, visual + ICP registration, global relocalization). It runs alongside Point-LIO on the bags, but none of them returns to its start, so no loop closures of any kind happened and the benefit is still unmeasured.
- **Sensor-side front** (see Decisions) is in, and the sim lap passes with the LiDAR and the D435 both in the costmaps (SLAM within 2.5 cm).

Next:
- [ ] Rebuild the rover image and run the chain on the robot (`slam:=true`), robot still at start-up.
- [ ] Point-LIO on the Jetson: measure CPU and lag with a bag played on the Orin NX (about 30% of a laptop core at 0.1 m voxels; if it falls behind, switch to 0.4 m voxels).
- [ ] Without a camera there is no global relocalization: the robot must start at the map origin or be given `/rtabmap/initialpose`. Add a "set pose" control to the dashboard, or a fixed home spot.
- [ ] Guard against Point-LIO divergence (e.g. reset when its speed exceeds what the robot can do). None seen since the switch to the output model, but still wanted before unattended runs.
- [ ] Record a loop that ends exactly where it started (floor mark), with the camera on: measures drift and shows whether the camera's loop closures (`slam_camera:=true`) help.
- [x] Point-LIO short on straight constant-speed legs: the IMU-as-input model; switched to the output model.
- [ ] Deploy the sensor-side front on the Jetson: controller and dashboard together; tell the team teleop "forward" has flipped.
- [ ] Measure height drift properly on level ground (same loop recording as above).
- [x] Which way the sensors face: both the L2 and the D435 sit on what the motor convention called the back, facing out (confirmed by Aahil 2026-11-22). The robot's front is now that side (`forward_is_sensor_side`, see Decisions); the URDF already had the sensors at +x.

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
- Which way does the D435 face? It helps RTAB-Map most if it covers the half the L2 cannot see.
- Confirm the map storage path under `/data` (RTAB-Map database).
- Was the L2 tilt calibration (2026-11-21) measured with the robot on level ground? Gravity-based roll and pitch are only as good as the floor they were measured on.
