# First robot test: Point-LIO → RTAB-Map → Nav2

A step-by-step plan for the first run of the localization chain on the rover. Each step proves one layer before the next one depends on it. Don't skip ahead when a step fails; its "If it fails" line says where to look. Everything here has passed in the sim and on recorded bags ([LOCALIZATION_RESULTS.md](LOCALIZATION_RESULTS.md)), but nothing has run on the robot yet.

All commands run on the Jetson, from `~/intel_sys_workspaces`, unless they say otherwise.

**Bring:** tape for floor marks, a tape measure, someone at the physical stop for any step where the wheels touch the floor.

## 0. Prepare the Jetson (about 10 minutes)

1. Check for teammates' uncommitted work before switching branches. If there is any, ask them before going further; don't commit or discard it for them.
   ```bash
   git status --short
   ```
2. Get the branch.
   ```bash
   git fetch origin && git checkout slam-chain && git pull
   ```
3. Stop the hand-started containers. The compose services reuse their names.
   ```bash
   docker stop stm32stack lidarstack robot-dashboard; docker rm stm32stack lidarstack robot-dashboard
   ```
4. Check the disk. The image build needs about 20 GB.
   ```bash
   df -h /
   ```

**Pass:** clean (or agreed) working tree on `slam-chain`, at least 25 GB free.
**If it fails:** low disk means pruning old images (`docker images`); ask the team before deleting any.

## 1. Rebuild the image and the workspace (30–90 minutes, mostly waiting)

```bash
./run_docker.sh --rover --build                  # rover image with RTAB-Map and the new dependencies
./run_docker.sh --rover bash scripts/run_tests.sh   # builds the workspace, runs all tests
```

**Pass:** the summary prints `PASS build`, `PASS colcon test`, `PASS script tests`.
**If it fails:** a failed `apt` step is usually the network; a missing arm64 package (most likely `ros-humble-rtabmap-ros`) needs a note in the roadmap and a workaround.

## 2. Drivers, front direction, dashboard (15 minutes, wheels off the floor)

```bash
scripts/preflight.sh                  # host checks: STM32, LiDAR, disk, clock
scripts/robot.sh up                   # Zenoh router, STM32, LiDAR, robot description
scripts/robot.sh ps
scripts/robot.sh ros2 topic hz /lidar/points_raw   # about 12 Hz (Ctrl-C to stop)
scripts/robot.sh ros2 topic hz /lidar/imu          # about 250 Hz
scripts/dashboard.sh
```

Then, **with the wheels lifted clear of the floor**, press forward on the dashboard's drive pad.

**Pass:** all four services are up; rates as above; the dashboard shows LiDAR and odometry; **forward spins the wheels toward the LiDAR and camera end.** This is the first real check of the new front convention.
**If it fails:** if forward drives away from the sensors, the controller isn't running the new code (`scripts/robot.sh logs stm32` prints which front it uses at startup). Don't continue until forward is right: Nav2 and every map depend on it.

## 3. Point-LIO standing still (5 minutes, robot on the floor, not moving)

```bash
scripts/robot.sh up localization      # Point-LIO + RTAB-Map, mapping a new map
scripts/robot.sh ros2 topic hz /odom  # should match the LiDAR, about 12 Hz
docker stats --no-stream localization # CPU of Point-LIO + RTAB-Map
```

Wait 60 seconds without touching the robot, then:

```bash
scripts/robot.sh ros2 topic echo /odom --once --field pose.pose.position
```

**Pass:** `/odom` at the LiDAR rate and steady; the position stays within about 2 cm of zero.
**If it fails:** `/odom` slower than the LiDAR means Point-LIO can't keep up on the Orin NX. Switch to 0.4 m voxels (`filter_size_surf` and `filter_size_map` in `src/intel_sys_localization/config/point_lio.yaml`; equally accurate in the tests, 30% less CPU) and repeat. A position that wanders while still points at the IMU or the startup; check `scripts/robot.sh logs localization`.

## 4. Drift loop with a recording (20 minutes, the key measurement)

1. Put tape on the floor at two corners of the robot so it can be placed back exactly, same spot and heading.
2. Restart localization there so odometry starts at zero, start recording, and open Foxglove.
   ```bash
   scripts/robot.sh stop localization && scripts/robot.sh up localization
   scripts/record_bag.sh first_loop --all     # leave running; Ctrl-C at the end
   scripts/robot.sh up foxglove               # then connect to ws://<robot-tailscale-ip>:8765
   ```
3. Drive slowly with the dashboard pad around a loop of at least 20 m, ideally through a corridor, and back onto the tape marks.
4. Read the end position, stop recording, and stop localization so RTAB-Map saves the map.
   ```bash
   scripts/robot.sh ros2 topic echo /odom --once --field pose.pose.position
   # Ctrl-C the recorder
   scripts/robot.sh stop localization
   python3 -c "import sqlite3; c=sqlite3.connect('data/rtabmap.db'); print('nodes', c.execute('select count(*) from Node').fetchone()[0], 'closures', dict(c.execute('select type, count(*) from Link where type in (1,2) group by type').fetchall()))"
   ```

**Pass:** the end position is within 1% of the loop length from zero (20 cm on 20 m), height within 10 cm on level floor; RTAB-Map stored at least one closure (type 1 = loop, 2 = proximity); the map in Foxglove (`/map`, `/rtabmap/cloud_map`) has single, straight walls.
**If it fails:** keep the bag either way. It is the first recording from the robot itself and can be replayed on the laptop with `scripts/eval/run_lio_on_bag.sh`.

## 5. Re-localize on the saved map (10 minutes)

Without the camera, RTAB-Map assumes the robot starts where mapping began. Put it back on the tape marks.

```bash
SLAM_LOCALIZATION=true scripts/robot.sh up localization
```

Drive part of the loop again and watch Foxglove (fixed frame `map`).

**Pass:** the live LiDAR points land on the saved map's walls for the whole drive; the robot's pose in `map` matches where it physically is.
**If it fails:** check the robot really started on the marks. A start elsewhere needs a pose on `/rtabmap/initialpose` (the "set pose" control on the roadmap).

## 6. Nav2, short goals (20 minutes, someone at the physical stop)

**Before this step:** Nav2's limits are still sim settings: up to 0.5 m/s, and it may reverse (the BackUp recovery included) into the side neither sensor sees. Lower them for the first run (about 0.2 m/s, no reverse, no BackUp) before starting.

```bash
scripts/robot.sh up navigation        # keeps the running localization as it is
```

In Foxglove, publish goals on `/goal_pose` (the 3D panel's pose tool; set its topic to `/goal_pose`). Start with 1 m straight ahead, then 1 m to the side (strafing), then a goal around an obstacle.

**Pass:** each goal is reached within 20 cm without touching anything; the robot strafes for the sideways goal instead of turning.
**If it fails or looks wrong:** `scripts/robot.sh stop navigation`. **The dashboard e-stop doesn't stop Nav2**: Nav2 publishes `/cmd_vel` too, so the two fight. Stop the service, or use the physical stop.

## 7. Wrap up

```bash
scripts/robot.sh stop navigation localization foxglove
```

Copy the bags and the map to a laptop (from the laptop):

```bash
scp -r autonomy-desktop:~/intel_sys_workspaces/bags/first_loop_* autonomy-desktop:~/intel_sys_workspaces/data/rtabmap.db bags/
```

Write what passed and failed into [ROADMAP.md](ROADMAP.md) (Next and Phases 5–7).
