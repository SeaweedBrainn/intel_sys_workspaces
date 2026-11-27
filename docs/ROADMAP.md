# Roadmap

Goal: autonomous driving on one localization chain, **Point-LIO** (LiDAR-inertial odometry) → **RTAB-Map** (loop closure, map, 2D grid; LiDAR plus the camera) → **Nav2**, for a robot that is a testbench for the team's University Rover Challenge rover.

## Current state (branch `slam-chain`, 2026-11-26)

- **On the robot (since 2026-11-23):** Point-LIO → RTAB-Map → Nav2. A 21.5 m loop closed within 2 cm, re-localization on the saved map works, Nav2 reached goals at the first-run limits, and with the camera a 19.4 m loop gave 9 visual closures and ended 4 cm from its start ([LOCALIZATION_RESULTS.md](LOCALIZATION_RESULTS.md)). It runs from a `slam-chain` worktree on the Jetson (`~/intel_sys_slam`); every stack runs the image `intel-sys-rover:latest`.
- **Sim only, waiting for the robot** ([ROBOT_TEST_DAY.md](ROBOT_TEST_DAY.md)): `twist_mux` and the e-stop lock, the STM32 link watchdog, frontier exploration ([NAVIGATION.md](NAVIGATION.md)), tag following, relocalization from a Foxglove pose estimate, `SLAM_CONTINUE`, the 60 m global costmap, the STVL LiDAR model. The robot's image predates `twist_mux` and `scipy`: test day starts with a rebuild.
- **UI:** Foxglove ([FOXGLOVE.md](FOXGLOVE.md)). The web dashboard and its wheel-odometry explorer were removed on 2026-11-26.
- **Hardware:** STM32 at `/dev/rrc`; Unitree L2 at `192.168.1.2` (`/lidar/points_raw` about 12 Hz, IMU on `/lidar/imu`); RealSense D435i on USB 3 as the `camera` service (640×480, 15 fps, its IMU unused). The front is the sensor side. Wheel odometry is integrated from **commanded** speeds, not encoders.

## Open items

**Before relying on it:**
- [ ] **Robot test day** ([ROBOT_TEST_DAY.md](ROBOT_TEST_DAY.md)): everything listed above as sim only, one session, fastest checks first.
- [ ] **Zenoh subscriber goes deaf under heavy traffic** (robot, 2026-11-23/24): Nav2's costmaps stopped receiving `/lidar/points` while the D435's full cloud (about 50 MB/s) went to the same processes, and once when a bag recorder restarted; Nav2 then refused to move. Not reproducible on x86 (2026-11-26, shared memory on and off), so it points at the Jetson. On the Jetson every message above a few KB failed shared memory ("Unable to create POSIX shm segment: OS error 12"). `scripts/eval/zenoh/zenoh_check.sh` measures it there (test day step 1); if shared memory is the cause, turn it off in `docker/zenoh/session.json5`, then try a downsampled camera cloud in the costmaps.
- [ ] **Subscribers don't re-match after a publisher restart** under rmw_zenoh (robot, 2026-11-16). Not reproducible on x86 with rmw_zenoh_cpp 0.1.11; same check on the Jetson.
- [ ] **False visual loop closures in look-alike places** (sim, 2026-11-24 and 26): one closure joined two plain grey rooms 10 m apart and turned the map 180°. With Point-LIO's real odometry uncertainty (1e-4) RTAB-Map rejected 147 bad candidates, but one 9.4 m closure still got through later. The sim's identical rooms are worst-case; real rooms are the test (test day step 3). If it happens there: raise `Vis/MinInliers`, check the ICP refinement's acceptance.
- [ ] **Point-LIO heading drift:** 13° once on 2026-11-23, 12° and 78 cm over a 19.4 m loop on 2026-11-24 (RTAB-Map corrected it). Record LiDAR and IMU on every drive so it can be replayed; prepared but not run: replay the hallway bag with every core busy to test a CPU-load cause.
- [ ] **Raise Nav2's first-run limits** (0.2 m/s, no reverse, no BackUp recovery) once navigation is proven on the robot. Without reversing, Nav2 can't back out of a corner it is wedged in; decide on a short, slow, checked reverse.
- [ ] **Autonomy rules:** no untethered Nav2 run without someone at the physical stop until there is a hardware e-stop (a physical or radio stop that cuts the motors independent of software). Low obstacles (cables, sills), drop-offs and overhangs below or above the costmaps' height band are not detected.
- [ ] **Team checkout:** the team's `~/intel_sys_workspaces` on the Jetson is still on `ahmad-zenoh`. Move it to the result of `slam-chain`, and tell the team teleop "forward" now drives toward the sensors. Pick the camera profile (Ahmad's 640×480 colour and 640×360 depth at 30 fps vs our 640×480 at 15 fps) by measurement once the Zenoh question is settled.

**Next:**
- [ ] **Keep mapping on the saved map** (`SLAM_CONTINUE=true`): the URC rover's normal mode. Wired through; not tried in the sim or on the robot (test day step 4). Without the camera a new session can't join the old map.
- [ ] **One configuration for indoors and outdoors (URC)** (plan agreed 2026-11-26). Today everything after Point-LIO is planar: the frame adapter drops z, roll and pitch; RTAB-Map optimizes x, y, yaw (`Reg/Force3DoF`); the grid and the costmaps take obstacles from a fixed height band above the odometry frame, so slopes read as walls. Tie each setting to the robot, its sensors or measured uncertainty rather than the environment; keep a short environment file only for real trade-offs. Each step tested indoors first, then on outdoor recordings:
  1. **Obstacle heights from the local ground**, not the odometry frame. The flattening exists because Point-LIO's height drifts (1.2 m over the hallway bag); ground-relative heights fix that in both environments and let it go. Biggest single step.
  2. **Ground by surface direction** in RTAB-Map's grid (`Grid/NormalsSegmentation`, `Grid/MaxGroundAngle`). Off today for an unrecorded reason; check indoors.
  3. **3D graph with gravity:** `Reg/Force3DoF: false`, `Optimizer/GravitySigma` about 0.3, 6-DoF odometry.
  4. **Speed from the surroundings:** Nav2's collision monitor slowdown zones or speed-limit filters.
  5. **GPS when present:** `robot_localization`'s navsat transform.
  6. **Environment file, only for trade-offs:** global costmap size and cell size together with RTAB-Map's grid cell size (60 m at 5 cm today; outdoors e.g. 200 m at 25 cm, too coarse for a 0.8 m door unless the 5 cm local costmap handles doors), RTAB-Map's grid range, the explorer's `min_new_area`.
  Needs outdoor recordings (L2, IMU, camera, GPS) before steps 2–6 can be judged.
- [ ] **Camera mount check:** the camera cloud against the L2 cloud in Foxglove; the URDF's sensor mounts are estimates until the mechanical team sends measured values.
- [ ] **CPU:** Point-LIO keeps up on the Jetson at 0.1 m voxels (about half a core); the Python body filter and frame adapter cost about as much again. Port to C++ or merge them if CPU gets tight. The 5 cm global costmap is 4× the cells of the earlier 10 cm one: check planner and explorer time on the Jetson.
- [ ] **Encoders:** check whether the STM32 can report wheel speeds; until then wheel odometry is not fused into anything.
- [ ] **Battery units:** the bridge passes the raw board value through (read 4.1–4.2 earlier, 12.1–12.3 on 2026-11-16). Confirm with a multimeter.
- [ ] Divergence guard for Point-LIO (restart it when its speed exceeds what the robot can do). Low priority: no divergence since the switch to the output model.

## Decisions

- **Testbench for the URC rover** (2026-11-26). The URC rover normally maps and localizes at the same time and drives long outdoor traverses. Testing starts indoors. Prefer settings that scale with distance or uncertainty over room-sized constants.
- **One localization chain, no estimator switching** (2026-11-21). Point-LIO owns `odom→base_footprint`, RTAB-Map owns `map→odom`, Nav2 consumes both. Switching between estimators makes the pose jump, which Nav2 handles badly; fallbacks go into one fused filter. Replaced slam_toolbox + Isaac ROS Visual SLAM + an estimator-switching supervisor.
- **RTAB-Map** takes Point-LIO as external odometry, closes loops on 3D LiDAR and the camera, and produces the 2D grid for Nav2. **The camera goes into RTAB-Map as RGB-D**, not as a separate VIO.
- Point-LIO uses the L2's built-in IMU (`/lidar/imu`), not the STM32's.
- **The front is the sensor side** (2026-11-22). The L2 and the D435 sit on one end; that end is +x in `/cmd_vel`, wheel odometry and the URDF. The STM32 firmware is unchanged; `mecanum_controller` and `odom_publisher` negate `vx` and `vy` (`forward_is_sensor_side` in `stm32.launch.py`).
- **Foxglove is the UI** (2026-11-24). New features are ROS topics and services with a Foxglove panel, not a custom web page.
- Each stack runs in its own container, as a compose service in `docker-compose.robot.yml` (`scripts/robot.sh`). Only the STM32 stack is privileged; the others have no device access beyond what they need and a read-only workspace. Drivers, description and health restart on their own; localization and navigation are started per session.
- Dependencies live in the Docker images (`Dockerfile.rover`, `Dockerfile.sim`).

## Open questions

- Was the L2 tilt calibration (2026-11-21) measured on level ground? Gravity-based roll and pitch are only as good as the floor.
