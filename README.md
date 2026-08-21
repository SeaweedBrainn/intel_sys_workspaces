# Intel Sys Mobile Robot Platform

Modular ROS 2 Humble workspace and multi-stage Docker environment for the Intel Sys mecanum mobile robot.

---

## Architecture Overview

The codebase is organized into **7 core decoupled ROS 2 packages** under `src/` with pre-compiled third-party sensor dependencies isolated in the Docker image:

```text
intel_sys_workspaces/
├── docker/                     # Container entrypoints and environment scripts
├── third_party/                # Pinned third-party upstream repositories (robot.repos)
├── src/
│   ├── intel_sys_interfaces/   # Custom STM32 message and service definitions
│   ├── intel_sys_hardware/     # STM32 serial bridge, mecanum velocity controller & sensor drivers
│   ├── intel_sys_description/  # Unified robot URDF/Xacro, 3D meshes & state publisher
│   ├── intel_sys_localization/ # Mecanum odometry, decoupled EKF fusion & Point-LIO 3D LiDAR odometry
│   ├── intel_sys_navigation/   # Nav2 stack configuration, costmaps & autonomous path planning
│   ├── intel_sys_sim/          # Gazebo Harmonic simulation, worlds, and ros_gz_bridge
│   └── intel_sys_bringup/      # Top-level system coordinators and Foxglove Studio dashboard
├── run_docker.sh               # Universal runner (auto-detects Desktop x86_64 vs Jetson ARM64)
├── Dockerfile                  # Multi-stage Docker build with Gazebo Harmonic & Foxglove bridge
└── docker-compose.yml          # Container service definition with USB/GPU pass-through
```

---

## Package Summary

| Package | Type | Description |
| :--- | :--- | :--- |
| **`intel_sys_interfaces`** | `ament_cmake` | 15 custom ROS 2 messages (`MotorsState`, `MotorState`, `LedState`, `BuzzerState`, `ButtonState`, `Sbus`, `OLEDState`, `BusServoState`, `PWMServoState`) and 2 services. |
| **`intel_sys_hardware`** | `ament_python` | STM32 serial driver bridge (`stm32_bridge` with MultiThreadedExecutor), inverse mecanum velocity controller with `/joint_states` publishing (`mecanum_controller`), sensor launchers, and udev rules. |
| **`intel_sys_description`** | `ament_cmake` | Unified `robot.urdf.xacro`, chassis and mecanum wheel STL meshes, Gazebo `MecanumDrive` and sensor plugins, and one-click `view_robot.launch.py`. |
| **`intel_sys_localization`** | `ament_python` | Dead-reckoning `odom_publisher`, decoupled `robot_localization` EKF configuration, Unitree L2 Point-LIO 3D LiDAR odometry, and unit tests. |
| **`intel_sys_navigation`** | `ament_cmake` | Nav2 omnidirectional navigation stack (DWB controller, NavFn planner, global/local costmaps, behavior trees). |
| **`intel_sys_sim`** | `ament_cmake` | Gazebo Harmonic simulation environment (`default.sdf`), model spawner, and `ros_gz_bridge` configuration. |
| **`intel_sys_bringup`** | `ament_cmake` | Top-level system coordinators: `robot.launch.py`, `sim.launch.py`, `teleop.launch.py`, and `foxglove_layout.json`. |

---

## Quick Start with `run_docker.sh`

[`run_docker.sh`](file:///c:/Users/aahil/Documents/Coding/Robotics/intel_sys_workspaces/run_docker.sh) auto-detects whether you are running on an **x86_64 PC/WSL** or an **ARM64 NVIDIA Jetson Orin NX** and manages a shared multi-terminal container:

```bash
# 1. Start or attach to the shared container shell (auto-detects NVIDIA RTX GPU on Desktop / Tegra on Jetson):
./run_docker.sh

# 2. Force specific GPU/CPU modes (optional):
./run_docker.sh --nvidia     # Force NVIDIA GPU pass-through
./run_docker.sh --cpu        # Force standard CPU mode

# 3. Rebuild the image before entering:
./run_docker.sh --build

# 4. Stop the container when finished:
./run_docker.sh --down

# 5. Run any ROS 2 command directly inside the container:
./run_docker.sh ros2 launch intel_sys_bringup robot.launch.py
./run_docker.sh colcon test
```

---

## Hardware Setup (One-Time Setup on Host)

To ensure the STM32 microcontroller binds consistently to `/dev/rrc` with read/write permissions:

```bash
cd src/intel_sys_hardware/udev
sudo ./install_udev_rules.sh
```

---

## Launching the System

### 1. Launch the Physical Robot (Master Command)

Starts the entire stack (robot description + STM32 bridge + sensors + localization EKF + Nav2 + Foxglove WebSocket Bridge):

```bash
./run_docker.sh ros2 launch intel_sys_bringup robot.launch.py
```

#### Common Arguments:
```bash
# Enable Point-LIO 3D LiDAR odometry in EKF fusion:
./run_docker.sh ros2 launch intel_sys_bringup robot.launch.py use_point_lio:=true
```

### 2. Launch 3D Simulation (Gazebo Harmonic + Foxglove Bridge)

Spins up the Gazebo obstacle world, spawns the robot model with Mecanum drive physics, and starts the Foxglove WebSocket bridge:

```bash
# Simulation with Gazebo GUI + Foxglove WebSocket bridge:
./run_docker.sh ros2 launch intel_sys_bringup sim.launch.py

# Launch simulation with autonomous Nav2 planning enabled:
./run_docker.sh ros2 launch intel_sys_bringup sim.launch.py use_nav:=true

# Headless Simulation (for CI / automated testing):
./run_docker.sh ros2 launch intel_sys_bringup sim.launch.py headless:=true
```

### 3. Preview 3D Robot Description

To inspect the URDF, meshes, coordinate frames, and joint sliders:

```bash
./run_docker.sh ros2 launch intel_sys_description view_robot.launch.py
```

### 4. Teleoperation

Control the mobile base using keyboard or joystick:

```bash
# Keyboard teleoperation:
./run_docker.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Gamepad joystick:
./run_docker.sh ros2 launch intel_sys_bringup teleop.launch.py use_joy:=true joy_dev:=/dev/input/js0
```

---

## Visualization with Foxglove Studio

The workspace uses **Foxglove Studio** for high-performance, cross-platform telemetry and 3D visualization without X11/GUI forwarding latency:

1. Download and open [Foxglove Studio](https://foxglove.dev/download) (or open `https://app.foxglove.dev` in your browser).
2. Click **Open connection** and connect to:
   ```text
   ws://localhost:8765
   ```
   *(or `ws://<jetson-ip>:8765` when running remotely on real robot)*
3. Import the pre-configured layout:
   * In Foxglove Studio, go to **Layout** $\to$ **Import from file...**
   * Select [`src/intel_sys_bringup/config/foxglove_layout.json`](file:///c:/Users/aahil/Documents/Coding/Robotics/intel_sys_workspaces/src/intel_sys_bringup/config/foxglove_layout.json).
   * This dashboard opens a 3D scene (`/lidar/points`, `/robot_description`, `/odom`, TF tree), real-time camera stream (`/camera/image_raw`), live IMU acceleration/gyro plots, and teleoperation controls.

---

## Testing & Verification

Run the full automated test suite inside the container:

```bash
# Build all packages:
./run_docker.sh colcon build --symlink-install

# Run all unit tests:
./run_docker.sh colcon test

# View test summary:
./run_docker.sh colcon test-result --all --verbose
```