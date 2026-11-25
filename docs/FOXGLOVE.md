# Foxglove

Foxglove is the robot's UI; the web dashboard (`scripts/dashboard.sh`, [DASHBOARD.md](DASHBOARD.md)) is being retired and no longer starts by default.

Start the bridge with `scripts/robot.sh up foxglove` (Tailscale address, port 8765), open `ws://<robot-tailscale-ip>:8765` in Foxglove, and import `src/intel_sys_bringup/config/foxglove_layout.json`. In the sim the bridge is on `ws://localhost:8765`.

| Panel | Shows / does |
|---|---|
| 3D (display frame `odom`) | Robot, `/lidar/points`, RTAB-Map `/map`, global and local costmaps, `/plan`, `/local_plan`, the followed tag and its goal. The publish-pose tool sends a Nav2 goal on `/goal_pose`. |
| Image | `/camera/image_raw` |
| Diagnostics | `/diagnostics`: `robot_health` (stream rates and freshness, `odom` and `map` TF age) and Nav2's lifecycle state |
| Follow tag ON/OFF | `/tag_follower/enable` (needs `scripts/robot.sh up follow`) |
| Explore START/STOP | `/frontier_explorer/start`, `/stop` (needs `scripts/robot.sh up navigation`); its state is shown below the buttons |
| Teleop | `/cmd_vel`, 0.15 m/s and 0.5 rad/s. Check the speeds in the panel settings after importing. `mecanum_controller` caps every source at 0.3 m/s and 1.0 rad/s and stops after 0.5 s without commands. |
| Plot, Log | `/battery` (raw board value), `/rosout` warnings and errors |

Not covered: the dashboard's e-stop (dropped for now; while Nav2 drives, stop it with Ctrl-C on the explorer or follower, or `scripts/robot.sh stop navigation`) and its view-only tokens: anyone on the tailnet who can reach port 8765 can drive.
