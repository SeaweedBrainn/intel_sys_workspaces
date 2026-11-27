# Navigation and exploration

How Nav2 is set up, how the frontier explorer chooses goals, and what has been measured. Everything here has run in the sim; on the robot only basic Nav2 goals have run so far (see [ROBOT_TEST_DAY.md](ROBOT_TEST_DAY.md)).

## Nav2

- **Frames.** With SLAM, the global costmap is in `map` with RTAB-Map's `/map` as a static layer; the local costmap, controller and behaviours stay in `odom`. Without SLAM (tag following) everything plans in `odom` on the live LiDAR.
- **Global costmap:** a 60 m rolling window around the robot, so goals into unmapped space still plan (the URC rover's main job). Its cell size must equal RTAB-Map's `Grid/CellSize` (5 cm; `test_config.py` checks it): in a rolling window the static layer reads one `/map` cell per costmap cell, so a coarser costmap drops walls one map cell thick. At 10 cm about half the walls were missing and the explorer put goals inside them (`loop.sdf`, 2026-11-26).
- **Obstacles:** STVL with the 3D LiDAR model (`model_type: 1`), so space the LiDAR sees through clears within about a second (`decay_acceleration`), while obstacles out of view are remembered (`voxel_decay` 5 s local, 20 s global). Nothing sees behind the robot, so a box disappearing as soon as the robot turned away made it oscillate around it (robot, 2026-11-23). **Following is the exception:** there the memory is 1.5 s in both costmaps (`follow_overlay` in `navigation.launch.py`). The target is a moving obstacle the robot drives behind, and each spot it stood on stayed lethal, right where the follow goal goes; the LiDAR doesn't sweep the low patch just in front of the robot, so it didn't clear. With the long memory NavFn found a follow plan only some of the time (loop.sdf, 2026-11-27). LiDAR only: the D435's full cloud starved the costmaps' LiDAR subscription under Zenoh on the robot.
- **Inflation:** global 0.55 m with fall-off 5 (paths keep 0.4–0.5 m from walls where there is room; at 0.35 m DWB could not turn at corners), local 0.35 m with fall-off 6 (so 0.8 m gaps are taken; confirmed on the robot 2026-11-24).
- **Controller:** Rotation Shim (turns in place when the path is more than 45° off) in front of DWB. First-run limits: 0.2 m/s and 0.6 rad/s; `mecanum_controller` caps every source at 0.3 m/s and 1.0 rad/s. Normal driving never reverses (DWB `min_vel_x` 0): nothing sees behind the robot.
- **Recoveries** (`behavior_trees/navigate_to_pose.xml`): clear the costmaps, spin, wait, then back up 0.15 m at 0.05 m/s (the velocity smoother lets reverse through down to −0.05 m/s for this alone). Backing up frees the robot from a door jamb or corner it is wedged against, which the others can't; it goes into space the robot usually just drove through, checked against the local costmap's memory.
- **Who drives:** `twist_mux`, see [FOXGLOVE.md](FOXGLOVE.md#who-drives).

## Frontier explorer

`intel_sys_navigation/scripts/frontier_explorer`, with the planning in `tour.py` and `frontiers.py` (no ROS imports, unit-tested). It runs in the map-and-explore mode (`scripts/robot.sh up`, or the sim's default `mode:=explore`) from `launch/explore.launch.py`, idle until **Explore: START** in Foxglove. After FUEL (Zhou et al., RA-L 2021); built after a trial of roadmap-explorer, which mapped `loop.sdf` but never finished and had lifecycle and goal-replacement problems.

Each time `/map` changes (about once a second):

1. **Chunks.** Frontier cells (free next to unknown) are clustered; clusters under `min_frontier_cells` (8) are dropped, larger ones split along their main axis until each fits in `chunk_radius` (1.5 m).
2. **Viewpoints.** 25 samples on rings around each chunk. A sample counts if it is reachable on the global costmap (known cells cheaper than inscribed) and stands on a cheap cell (`goal_max_cost` 30, about 0.45 m from walls: room to stop and turn), at least `min_goal_distance` (0.6 m) from the robot. The best one sees the most of the chunk's frontier cells within a 180° window (`sensor_fov_deg`, the sensors face forward only) and `view_range` (5 m); the window's middle is the goal heading.
3. **Worth a trip.** A viewpoint is only kept if its view holds at least `min_new_area` (2 m²) of unknown space. Exploration ends when none is left; then it drives home.
4. **One stop per view.** A viewpoint covers another chunk when, facing its heading, it sees at least `cover_fraction` (0.8) of what that chunk's own viewpoint would; covered chunks get no stop of their own (greedy set cover). If something hidden blocks the view after all, the chunk's frontier is still there at the next replan.
5. **Order.** An open travelling-salesman path from the robot over all stops, costs in seconds: turn to the path, drive it, turn to the goal heading, at Nav2's speed and turn rate (read from the velocity smoother). Exact up to 9 stops, nearest neighbour plus or-opt beyond. The robot drives to the first stop.
6. **Keep or switch.** The current goal stays in the plan at its own spot and heading while anything it was sent to see is unmapped, and is kept unless another tour is more than 15% cheaper (`switch_hysteresis`).

A goal ends when what it was sent to see is mapped (the next goal preempts it), or when it is reached, fails, or takes 3× its expected travel time (`timeout_factor`, at least 30 s); those three are blacklisted (0.5 m). Every goal sent is tracked until Nav2 reports its end, so Stop and Ctrl-C cancel whatever is running.

`goal_strategy:=greedy` uses the same viewpoints and takes the most unknown area per second, for comparison.

**Lessons that shaped it (sim):**
- Value a view by the unknown area in it, not by frontier cells seen: with cells alone it spent 6 minutes on slivers behind furniture (in 2D a frontier's length says little about the space behind it).
- Keep goals away from walls: a goal 0.22 m from a door jamb wedged the robot for 13 minutes; it can't reverse out.
- Keep goals away from the robot: one 0.46 m away, inside Nav2's 0.5 m progress radius, stalled for 30 s.
- Track the current goal by position, not by this update's chunks: chunks are re-split as the map grows, and matching by chunk skipped the hysteresis and flipped between goals.
- One model of where the robot can drive (Nav2's costmap). An earlier version had four (a flood fill with clearance thresholds, goal placement, a planner pre-check, the costmap) plus patches for the gaps between them.

## Results (sim, camera off, one run each)

Times from the first move.

| World | Version | 90% mapped | 95% mapped | Done | Goals |
|---|---|---|---|---|---|
| `loop.sdf` (113 m², one room) | greedy (2026-11-25) | 160 s | 162 s | 244 s | 27 |
| | first tour version | 126 s | 152 s | 208 s | 17 |
| | current (2026-11-26) | 158 s | 166 s | 210 s | 16 |
| `office.sdf` (279 m², 6 rooms off a corridor) | greedy | 486 s | 838 s | not after 20 min | 47, 5 failed |
| | first tour version | 452 s | 538 s | 606 s, home not reached | 33 |
| | current, 10 cm global costmap | 558 s | 612 s | 715 s, home at 860 s | 35 |

No failed goals in any tour run. Single runs vary by a minute or more. With rooms, the tour finishes in about half greedy's time; in one open room the order matters little. Planning takes a median 120 ms per map update on the laptop; check it on the Jetson.

Open: the office hasn't been rerun with the 5 cm costmap. In `loop.sdf` the last two goals still counted about 10 m² inside a solid block, probably through small unknown gaps in RTAB-Map's own wall; small.
