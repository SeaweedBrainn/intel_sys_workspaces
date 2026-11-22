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

## Simulation

- **Mapping lap (Phase 6): passes.** About 40 m around `loop.sdf`: Point-LIO ends 0.5–1.5 cm from truth, SLAM (map frame) within 8 cm. RTAB-Map's proximity closures vary from lap to lap (0, 3, 4 and 5 seen on identical setups), so the test's "at least one closure" check can fail by chance.
- **Localization mode: passes.** Relaunched with `slam_localization:=true` on the saved map: re-localized 103 times, within 4.6 cm for the whole lap.
- **With the sensor side as the front** and the D435 cloud in the costmaps, the lap passes (SLAM within 2.5 cm).
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
