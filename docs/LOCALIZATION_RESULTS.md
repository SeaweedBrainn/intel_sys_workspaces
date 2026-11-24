# Localization results

What has been measured on the Point-LIO → RTAB-Map → Nav2 chain, in the Gazebo sim and on recorded L2 bags. The plan and open items are in [ROADMAP.md](ROADMAP.md); this file holds the evidence behind them. All results are from 2026-11-22 (branch `slam-chain`) unless dated otherwise.

## Tools

All in `scripts/eval/`, run inside `intel-sys-sim` with the workspace built.

| Tool | Measures |
|---|---|
| `slam_loop_test.py` | Nav2 lap around `loop.sdf`; SLAM and Point-LIO poses against Gazebo truth at each waypoint |
| `lio_drift_test.py` | Point-LIO alone on straight corridor legs (steered by Gazebo truth); along-track error per leg |
| `run_lio_on_bag.sh` | Point-LIO on a recorded bag; trajectory, map image, lag (`lio_bag_eval.py` records) |
| `run_slam_on_bag.sh` | Point-LIO + RTAB-Map on a bag, optionally with the D435 (`CAMERA=1`) |
| `bag_mount_check.py` | L2 mount roll and pitch from a bag's IMU gravity |

Start the sim for the first two with `ros2 launch intel_sys_bringup sim.launch.py headless:=true use_foxglove:=false world:=loop.sdf x:=-6.5 y:=0.0` (`x:=-6.0 y:=-3.5 slam:=false` for the drift test). `point_lio_params:=<yaml>` swaps the Point-LIO config.

**Between repeated runs, kill every ROS process in the container and check none are left.** `kill -INT` on a scripted `ros2 launch` did not stop its nodes; a stale `point_lio_frame_adapter`, anchored on an earlier run's start, corrupted `/odom` in later runs and made every config look diverged.

## On the robot (2026-11-23)

First run of the chain on the rover, following [ROBOT_TEST_PLAN.md](ROBOT_TEST_PLAN.md), from a separate `slam-chain` worktree on the Jetson (`~/intel_sys_slam`, image `intel-sys-rover:slam`). The bags (about 14 GB) moved off the robot on 2026-11-24 to an external SSD (folder `intel_sys_bags/robot_20261123`); the maps are still on the robot under `data/`. Copying over Wi-Fi ran at about 1 MB/s, so move bags with a USB drive (exFAT needs `exfat-fuse` on the Jetson) or over a wired link.

| Test | Result |
|---|---|
| Tests on the Jetson | build, 25 package tests, 56 script tests pass |
| Front direction | forward drives toward the sensors |
| Point-LIO keeps up | `/odom` 11.9 Hz vs LiDAR 12 Hz; Point-LIO about half a core, the Python body filter and frame adapter about another core together |
| Point-LIO standing still, 60 s | 0.8 cm, 0.1° |
| Loop (run 2), 21.5 m, back onto a taped mark | Point-LIO ends **1.9 cm** from its start (0.09%), 2° heading; RTAB-Map 2.5 cm, 12 proximity closures, its correction never above 2.8 cm |
| Spin test, 720° left then 720° right | Point-LIO back within 1.2 cm and 1.9° |
| Re-localization on run 2's map, short drive back to the tape | ends within about 6 cm and 5° of its start, matching the tape |
| Nav2, 1 m straight ahead | reached; measured 91 cm forward, 7.6 cm right, rotated slightly left vs RTAB-Map 88.4 cm, 6.7 cm right, 5.2° left |
| Nav2, back to the start (turn around, drive back, turn around) | reached; on the tape lengthwise, 4.2 cm right and 8.9° left per RTAB-Map, matching the measurement at the back wheel |
| Nav2, long goal 6 m around the corner on run 2's map | reached 7 cm from the goal, 11° heading, no recoveries; RTAB-Map's correction at most 5.8 cm |
| Nav2 with a box in the corridor | saw it, but oscillated between the long and short way around (obstacle fades from the costmap within 2.5 s out of view); stopped by hand |

Found and fixed on the robot:
- **RTAB-Map's `RGBD/NeighborLinkRefining` dragged the map off.** It replaces every Point-LIO step with RTAB-Map's own ICP; on run 1 its correction grew to almost 1 m (heading swinging ±7°) with one closure. Off since run 2. Run 1's start mark moved, so its Point-LIO number (0.86 m) is not a drift measurement.
- **Nav2's controller missed its 20 Hz rate** on the Jetson. 10 Hz with about 1,000 sampled motions keeps up.
- **The Foxglove bridge showed nothing** when started before localization; restarting it after localization fixed it.

Open:
- Point-LIO's heading drifted about 13° on one short, unrecorded drive mixing driving and spins (RTAB-Map corrected it). The spin test rules out spinning alone. Record every drive until it shows up again.
- Zenoh logs "Unable to create POSIX shm segment" (out of memory for its shared-memory pool) and falls back to normal transport; harmless so far.

## On the robot with the camera (2026-11-24)

D435i on USB 3 as the `camera` service (640×480, 15 fps), RTAB-Map with `SLAM_CAMERA=true` (commit `96ab7fc`), new map, driven by hand from the dashboard. No bag was recorded. Closures were logged from `/rtabmap/info` to `data/closures_20261124.log` on the robot.

| Test | Result |
|---|---|
| Loop, 19.4 m in about 6 min, back to the start lined up by eye | RTAB-Map ends **4 cm** and 1.3° from its start; **9 visual loop closures** (7 back to the first node, the view offset shrinking from 48 cm to 4 cm while driving in; 2 mid-loop) and 1 proximity closure. Several candidates rejected (too few visual inliers, or the scans did not line up). |
| Point-LIO alone over the same loop | ends **78 cm** and 10.9° from its start: 12° of heading drift against the map. Same symptom as the unrecorded 13° drive on 2026-11-23; cause unknown. New since the good 21.5 m loop: the camera driver running, and more CPU load. |
| CPU, everything running (drivers, camera, dashboard, Foxglove, localization) | about 40% of the Jetson; RTAB-Map up to 77% of one core and 600 MB after the loop, the dashboard about 60%, Point-LIO 40–50%, the body filter about 40%, the camera driver 25% |

The camera does what Phase 8 asked: visual closures on the robot, and RTAB-Map's map stays right while the odometry drifts. Between closures the costmaps still see the drifted odometry.

## Simulation

- **Mapping lap (Phase 6): passes.** About 40 m around `loop.sdf`: Point-LIO ends 0.5–1.5 cm from truth, SLAM (map frame) within 8 cm. RTAB-Map's proximity closures vary from lap to lap (0, 3, 4 and 5 seen on identical setups), so the test's "at least one closure" check can fail by chance.
- **Localization mode: passes.** Relaunched with `slam_localization:=true` on the saved map: re-localized 103 times, within 4.6 cm for the whole lap.
- **With the sensor side as the front** and the D435 cloud in the costmaps, the lap passes (SLAM within 2.5 cm).
- **With the camera in RTAB-Map (2026-11-24): passes, but no visual closures.** SLAM within 2.7 cm after the lap, 8 proximity closures. Visual words are stored (2,083 over 188 nodes, about 100 features each), but the plain gray walls of `loop.sdf` give nothing distinctive, so the sim checks the camera plumbing, not visual loop closure. The sim camera renders at about 1.7 Hz on a laptop without a GPU.
- **Straight legs** (4 × 12 m at 0.3 m/s): within 0.3% with the output model; see [Point-LIO configuration](#point-lio-configuration).

## Real L2 bags

Hemang's recordings: three mount orientations plus the first half of a hallway run, hand-carried, not in git (`bags/`).

- **Mounts from IMU gravity:** orientation 1 ("0°") has the dome 83° from vertical, i.e. facing horizontally, like the robot today; orientation 2 ("45°") 39°; orientation 3 ("90°") 0.7°, dome straight up.
- **Data:** about 5,300 points per scan at 12 Hz, genuine per-point times over a ~0.08 s sweep, `ring` always 1. The bags' `/tf_static` puts the sensors at +x facing +x, the robot's current front.
- **Point-LIO** keeps up in real time on the laptop (0.02–0.03 s behind, about 30% of one core) and, with the output model, did not diverge in 20 runs, including the shaky start of the short orientation-1 bag. The 306 s hallway run gives a clean three-corridor map.
- **Height:** rises about 1.2 m over the hallway run, but the building has slopes and no bag passes the same spot twice, so height drift is unmeasured. Navigation ignores height (planar odom, RTAB-Map 3-DoF).
- **Accelerometer:** reads 9.67–10.02 m/s² at rest depending on which axis faces down. Setting `acc_norm` to the reading made tracking worse; keep 9.81 and let the bias estimate absorb it.
- **Camera (Phase 8):** `slam_camera:=true` runs alongside Point-LIO on the bags, but none of them returns to its start, so no loop closures happened and the camera's benefit is unmeasured.

## Point-LIO configuration

**Output model (`use_imu_as_input: false`).** With IMU as input, velocity is corrected only through its correlation with position (the LiDAR update has no velocity term) and process noise is scaled by dt², so the filter is overconfident in its speed. In a corridor the walls give no constraint along the direction of travel and at constant speed the accelerometer sees only gravity, so an early speed error persists. Turns move it onto an axis the walls constrain, which is why the Nav2 laps were accurate. Unitree's own L2 config uses the output model.

Sim straight legs, error per 12 m leg:

| Config | Error |
|---|---|
| Input model (previous default), two runs | 5.5–8.5% short |
| Input model, `acc_cov_input` 3.0 | 0.3–0.4% |
| Input model, `plane_thr` 0.05 | 9–13% short |
| **Output model (current)** | 0.2–0.3% |
| Output model + 0.4 m voxels | 0.1% |
| Unitree's full L2 config | 0.0–0.2% |

On the bags the input model diverged intermittently (same bag and settings: 1 of 3 runs on the dome-up bag, 2 of 4 from the orientation-1 start, then o1 and o2 again) and drew the dome-up bag's walls doubled. The output model gave single walls, never diverged, and ends within 0.3 m of Unitree's config on every bag.

**Unitree's other settings**, each on top of the output model:
- **0.4 m voxels** (`filter_size_surf` / `filter_size_map`; ours 0.1): equally accurate (smoothed paths within 2.5%), CPU 23% vs 28–35% of a core, but pose jitter 2.2–2.4 cm RMS vs 1.5–1.8 cm. Kept 0.1; 0.4 is the fallback if the Jetson can't keep up.
- **`blind` 0.5** (ours 0.1): no gain, and one divergence from the shaky o1 start. Kept 0.1; the body filter already removes the chassis.

**Silent costs found in the code:** `publish_odometry_without_downsample: true` publishes odometry and TF after every point (fell 0.5 s behind per second on real data), and `path_en` republishes the whole path every update (100 s behind within minutes). Both are off. This Point-LIO port ignores `point_filter_num` for the L2.

## Alternatives tried and dropped

- **RTAB-Map `icp_odometry` instead of Point-LIO.** Agreed with Point-LIO on the smooth bags (29.1 vs 28.5 m, 23.8 vs 23.2 m) but jittered frame to frame; failed on the dome-up bag and the hallway (128 and 760 impossible jumps; overshoots and resets in long corridors) and registered no forward motion in the sim corridor. Removed (the launch option lived in commit `87bd022`).
- **Visual-only RTAB-Map (D435).** Visual odometry lost track on 84–88% of frames with defaults and 37% with reset-on-loss and optical flow, jumping at every reset. It did find 7 visual loop closures on the dome-up bag, so place recognition works; the camera stays in as RTAB-Map input (Phase 8), not as odometry.

## Fixed along the way

Each of these would also have hit the robot:
- The Point-LIO frame adapter applied the LiDAR mount tilt twice (`base_footprint` pitched 84°).
- Nav2 had no `velocity_smoother` section; its defaults clamp `vy` to 0, silently dropping mecanum strafing.
- The static global costmap shrank to the explored map, so goals in unexplored space never planned; now a rolling window in `map`.
- Point-LIO's vertical drift lifted the floor into STVL as phantom obstacles; `odom→base_footprint` is now planar (x, y, yaw; REP-120).
- RTAB-Map localization mode assumed the robot restarted at the last mapped pose; `RGBD/StartAtOrigin` is set.
- Sim only: the simulated L2 fired a fixed 18-ring grid (Point-LIO stuck in corridors) and faked a 0.1 s sweep (Point-LIO ~7% short). Now a dense grid thinned at random to 6,400 points, with a 1 ms spread.
