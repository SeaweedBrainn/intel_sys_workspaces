# Foxglove

Foxglove is the robot's UI: views, teleop, the e-stop, and the explorer and follower controls.

Start the bridge with `scripts/robot.sh up foxglove` (Tailscale address, port 8765), open `ws://<robot-tailscale-ip>:8765` in Foxglove, and import `src/intel_sys_bringup/config/foxglove_layout.json`. In the sim the bridge is on `ws://localhost:8765`.

| Panel | Shows / does |
|---|---|
| 3D (display frame `odom`) | Robot, `/lidar/points`, RTAB-Map `/map`, global and local costmaps, `/plan`, `/local_plan`, the followed tag and its goal. Its publish tool (toolbar, top right) has two modes: **Publish pose** sends a Nav2 goal on `/goal_pose`; **Publish pose estimate** tells RTAB-Map where the robot is (`/rtabmap/initialpose`), for when it was started away from the map's origin without the camera: click where the robot stands on the drawn map and drag toward where it faces. |
| Image | `/camera/image_raw` |
| Diagnostics | `/diagnostics`: `robot_health` (stream rates and freshness, `odom` and `map` TF age) and Nav2's lifecycle state |
| Follow tag ON/OFF | `/tag_follower/enable` (needs `scripts/robot.sh up follow`) |
| Explore START/STOP | `/frontier_explorer/start`, `/stop` (needs `scripts/robot.sh up navigation`); its state is shown below the buttons |
| E-STOP / RELEASE | `/e_stop` (`std_msgs/Bool`): E-STOP stops the wheels at once and blocks every drive input, Nav2 and teleop alike, until RELEASE. Nav2 doesn't know it was stopped: stop exploration or following too, or its goal fails after the progress timeout. |
| Teleop | `/cmd_vel_teleop`, 0.15 m/s and 0.5 rad/s. Check the speeds in the panel settings after importing. Teleop overrides Nav2 while held (`twist_mux`, see below). `mecanum_controller` caps every source at 0.3 m/s and 1.0 rad/s and stops after 0.5 s without commands. |
| Plot, Log | `/battery` (raw board value), `/rosout` warnings and errors |

## Who drives

`twist_mux` (started with the STM32 stack, `intel_sys_hardware/launch/cmd_vel_mux.launch.py`, config `config/twist_mux.yaml`) picks one input and sends it to `/cmd_vel_mux`, which the wheels follow:

| Input | Topic | Priority |
|---|---|---|
| Nav2, scripts, anything older | `/cmd_vel` | 10 |
| A person (Foxglove teleop) | `/cmd_vel_teleop` | 100: overrides Nav2 while held, plus 0.5 s |
| E-stop lock | `/e_stop` | 255: blocks everything until false |

`mecanum_controller` also listens to `/e_stop` itself and zeroes the wheels on the spot. On a rover image without `twist_mux` (built before 2026-11-26), the launch relays `/cmd_vel` straight through and logs a warning: teleop on `/cmd_vel_teleop` is then ignored, but the controller's own e-stop still works. In the sim Gazebo holds the last velocity it was sent, so the simulated robot keeps moving under E-STOP; only the real controller zeroes the wheels.

Not covered: view-only access. Anyone on the tailnet who can reach port 8765 can drive.
