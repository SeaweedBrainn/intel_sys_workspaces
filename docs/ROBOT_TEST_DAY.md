# Robot test day: everything built in the sim since 2026-11-24

A checklist to work through on the rover in one session, fastest and most basic first. Everything here has run in the sim or on recorded bags, not on the robot ([ROADMAP.md](ROADMAP.md)). Each step says what proves it and what to do if it fails. Tick the boxes and write numbers next to them; they go into the roadmap afterwards.

All commands run on the Jetson in the `slam-chain` worktree, `~/intel_sys_slam` (not the team's `~/intel_sys_workspaces`), unless they say otherwise.

**Bring:** tape for floor marks, a tape measure, a box about 30 cm tall, an ArUco marker on a stick (5×5 dictionary, id 10; the follower detects ArUco, not AprilTag), someone at the physical stop whenever the wheels touch the floor.

## 0. Update and rebuild (start first: the image build takes 30–90 minutes)

```bash
cd ~/intel_sys_slam && git status --short && git pull
# One image for everything now: make the slam-chain image the latest, drop the old .env.
docker tag intel-sys-rover:slam intel-sys-rover:latest && rm -f .env
# Build the workspace on the current image, so steps 1-3 run today's code:
docker run --rm -v ~/intel_sys_slam:/workspaces/intel_sys intel-sys-rover:latest bash /workspaces/intel_sys/scripts/run_tests.sh 2>&1 | tail -12
df -h /                                            # the image build needs about 20 GB free
nohup env DOCKER_BUILDKIT=1 docker build --network host --progress=plain -f Dockerfile.rover -t intel-sys-rover:latest . > ~/build_image.log 2>&1 &
```

On the current image the tour planner's tests fail (no `scipy`); everything else should pass. The new image adds `twist_mux` (who drives, e-stop) and `scipy`. **Steps 1–3 work on the current image** while the new one builds. When it is done, rebuild the workspace in it and restart the stacks:

```bash
tail -3 ~/build_image.log
docker run --rm -v ~/intel_sys_slam:/workspaces/intel_sys intel-sys-rover:latest bash /workspaces/intel_sys/scripts/run_tests.sh 2>&1 | tail -12
scripts/robot.sh down && scripts/robot.sh up
```

- [ ] Image built; `PASS build`, `PASS colcon test`, `PASS script tests`.

## 1. Zenoh checks (10 minutes, robot can stay still)

The two open Zenoh bugs (subscribers that stop receiving after a restart; a costmap that goes deaf under heavy traffic) don't happen on x86 with this rmw_zenoh. This runs the same checks on the Jetson, on a private router (port 7457), without touching the running stacks:

```bash
scripts/robot.sh up drivers
docker exec -it stm32stack bash /workspaces/intel_sys/scripts/eval/zenoh/zenoh_check.sh
```

- [ ] rmw_zenoh_cpp version printed: ______
- [ ] Every line reads 10 msg/s, and the heavy-traffic line 10 Hz / 12 Hz, with shared memory **on**.
- [ ] Same with shared memory **off**.

**If shared memory on fails and off passes:** turn shared memory off in `docker/zenoh/session.json5`. If both fail at a restart, note which kind (SIGINT, SIGKILL or router).

## 2. Drivers: link watchdog, who drives, e-stop (15 minutes, wheels off the floor)

```bash
scripts/robot.sh logs stm32 | grep -E "twist_mux|relay|Front"
```

Import `src/intel_sys_bringup/config/foxglove_layout.json` again (it changed: E-STOP/RELEASE buttons, teleop now on `/cmd_vel_teleop`, wider right column).

- [ ] The log shows `twist_mux` subscribed to `cmd_vel`, `cmd_vel_teleop` and `e_stop` (after the rebuild; before it, a WARNING that it relays `/cmd_vel` only).
- [ ] Teleop forward spins the wheels toward the sensor end.
- [ ] Hold teleop, press **E-STOP**: wheels stop at once, teleop does nothing. **RELEASE**: teleop works again.
- [ ] `scripts/robot.sh ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"` spins the wheels; holding teleop backward meanwhile overrides it; releasing teleop goes back to forward after about 0.5 s. Ctrl-C.
- [ ] **USB unplug:** with `scripts/robot.sh logs -f stm32` open, unplug the STM32's USB for 3 s and plug it back. The log shows `STM32 link lost`, then `STM32 link back`, and `/imu_raw` resumes (`scripts/robot.sh ros2 topic hz /imu_raw`). If it doesn't come back, `/dev/rrc` may have moved; check `ls -l /dev/rrc`.

## 3. Point-LIO drift: recorded loops (20 minutes)

The 12° heading drift has never been recorded. Record LiDAR and IMU on every drive today:

```bash
scripts/robot.sh up                  # mapping, with the camera
scripts/record_bag.sh drift_loop     # Ctrl-C to stop; LiDAR, IMU, camera, odometry
```

Drive a loop of 15–20 m that ends on a floor mark, twice: once with the camera running, once without it (`scripts/robot.sh stop localization camera`, then `SLAM_CAMERA=false scripts/robot.sh up`).

- [ ] Loop with camera: Point-LIO end error ____ cm, heading ____°; RTAB-Map closures ____.
- [ ] Loop without camera: ____ cm, ____°.
- [ ] Copy both bags to the laptop (`bags/`): they're the input for finding the drift.
- [ ] **Look-alike places, camera on:** drive between two spots that look the same (two matching classrooms, both ends of a plain corridor) and back. Watch the map in Foxglove: a sudden jump or rotation of the whole map is a false visual loop closure. In the sim, look-alike grey rooms produced two (a 180° flip, and later a 9 m shift); real rooms are usually more distinctive, which is what this checks. Note any jump and record a bag of it.

The bag recorder takes every topic, including the camera's colour and depth images; if Nav2 or RTAB-Map go quiet while it runs, that is the deaf-subscriber bug, so note it.

## 4. Relocalization from a pose estimate (10 minutes)

On a saved map, without the camera, RTAB-Map assumes the robot starts at the map's origin. Foxglove's 3D panel can now tell it otherwise (checked in the sim: 0.6 m from the truth, then refined against the map).

```bash
scripts/robot.sh stop localization
SLAM_CAMERA=false SLAM_LOCALIZATION=true scripts/robot.sh up
```

Start the robot a few metres from where the map was started. In the 3D panel, choose **Publish pose estimate** in the toolbar (top right), click where the robot is on the drawn map and drag toward where it faces.

- [ ] The live LiDAR points jump onto the map's walls and stay there while driving.
- [ ] With the camera on (`scripts/robot.sh stop localization`, then `SLAM_LOCALIZATION=true scripts/robot.sh up`), from somewhere other than the origin: does RTAB-Map place the robot by itself? ____
- [ ] **Keep mapping on the saved map** (the normal mode for the URC rover; new, not tried in the sim): `scripts/robot.sh stop localization`, then `SLAM_CONTINUE=true scripts/robot.sh up`, camera on. Start somewhere already mapped and drive into new space. The new session should join the old map once the camera recognizes a place (the map stops being two separate pieces), and the database keeps both.

## 5. Navigation basics (20 minutes, someone at the physical stop)

```bash
scripts/robot.sh up        # Nav2 and the explorer are already up from step 4; this changes nothing running
```

- [ ] A Nav2 goal from the 3D panel (**Publish pose**) is reached and the robot stops (general goal checker).
- [ ] **E-STOP during a goal** stops the robot at once. RELEASE: Nav2 carries on or reports the goal failed.
- [ ] **STVL clearing:** walk past in front of the robot; your trail in the costmap clears within about a second once you've gone.
- [ ] **The box:** put the box in the robot's path and send a goal behind it (the 2026-11-23 oscillation case). It goes round without switching sides.
- [ ] **Gaps:** two boxes 0.8 m apart, goal on the far side: it goes through.
- [ ] A goal 15–25 m away plans and is reached, including one into space not mapped yet (the global costmap is now a 60 m window at 5 cm, was 30 m). Watch planner and explorer CPU: 4x the cells of 30 m; the explorer's planning took a median 120 ms in the sim.

## 6. Frontier exploration (30 minutes, someone at the physical stop)

Needs the new image (`scipy`). The explorer starts idle with the navigation stack.

```bash
scripts/robot.sh exec explorer python3 -c "import scipy; print(scipy.__version__)"
scripts/robot.sh logs -f explorer
```

Press **Explore: START** in Foxglove in a space with a few rooms or corridors.

- [ ] The log's first line reads `Nav2 limits for travel times: ... (read from velocity_smoother)`.
- [ ] Each goal logs `planned in N ms` (sim: median 120 ms, max 250 ms on the laptop): ____ ms on the Jetson.
- [ ] Goals face unmapped space (the robot arrives looking at what's unknown).
- [ ] It finishes by itself (`no frontiers left`) and drives back to the start.
- [ ] **E-STOP** stops it; **Explore: STOP** cancels the goal.
- [ ] Mapped area and time: ____ m² in ____ min.

- [ ] **Explore: STOP mid-goal:** the robot stops within a second or two (Nav2's goal is cancelled), and nothing drives it afterwards.

If it zig-zags or flips between goals, note where; `min_new_area` (2 m², smallest view worth a trip), `switch_hysteresis` (0.15), `goal_max_cost` (30, how far goals keep from walls) and `chunk_radius` (1.5 m) are the settings to look at.

If the robot wedges against a door jamb or corner and Nav2 reports "Failed to make progress" over and over, note it: with reversing disabled, Nav2's recoveries (clear costmaps, spin, wait) can't back it out. That decides whether a short, slow, checked reverse is worth allowing.

## 7. Tag following (15 minutes, camera on)

```bash
scripts/robot.sh up follow   # stops mapping and exploring, starts following (no RTAB-Map)
```

Foxglove **Follow tag: ON**, walk slowly with the marker.

- [ ] It follows at about 0.5 m and stops when you stop.
- [ ] Nothing else goes quiet while it runs (`robot_health` in the Diagnostics panel all green): the follower takes full-rate colour images, which could starve other subscribers under Zenoh.

## 8. Wrap up

```bash
scripts/robot.sh down
```

Copy today's bags and the RTAB-Map database to the laptop, and write the numbers into [ROADMAP.md](ROADMAP.md).
