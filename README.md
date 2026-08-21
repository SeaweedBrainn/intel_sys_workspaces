# Intel Sys Mobile Robot Platform

Modular ROS 2 Humble workspace and multi-stage Docker environment for the Intel Sys mecanum mobile robot.

---

## 🏗️ Architecture Overview

The codebase is organized into **6 core ROS 2 packages** under `src/` with pre-compiled third-party sensor dependencies isolated in the Docker image:

```text
intel_sys_workspaces/
├── docker/                     # Container entrypoints and environment scripts
├── third_party/                # Pinned third-party upstream repositories (robot.repos)
├── src/
│   ├── intel_sys_interfaces/   # Custom STM32 message and service definitions
│   ├── intel_sys_hardware/     # STM32 serial bridge, mecanum kinematics & sensor drivers
│   ├── intel_sys_description/  # Unified robot URDF/Xacro, 3D meshes & state publisher
│   ├── intel_sys_navigation/   # Mecanum odometry, decoupled EKF fusion, Point-LIO & Nav2
│   ├── intel_sys_sim/          # Gazebo Harmonic simulation, worlds, and ros_gz_bridge
│   └── intel_sys_bringup/      # Top-level one-command launch files and RViz profiles
├── run_docker.sh               # Universal runner (auto-detects Desktop x86_64 vs Jetson ARM64)
├── Dockerfile                  # Multi-stage Docker build with Gazebo Harmonic & dependencies
└── docker-compose.yml          # Container service definition with USB/GPU pass-through
```

---

## 📦 Package Summary

| Package | Type | Description |
| :--- | :--- | :--- |
| **`intel_sys_interfaces`** | `ament_cmake` | 15 custom ROS 2 messages (`MotorsState`, `MotorState`, `LedState`, `BuzzerState`, `ButtonState`, `Sbus`, `OLEDState`, `BusServoState`, `PWMServoState`) and 2 services. |
| **`intel_sys_hardware`** | `ament_python` | STM32 serial driver bridge (`stm32_bridge`), inverse mecanum velocity controller (`mecanum_controller`), LiDAR/camera launch files, and udev rules. |
| **`intel_sys_description`** | `ament_cmake` | Unified `robot.urdf.xacro`, chassis and mecanum wheel STL meshes, Gazebo `MecanumDrive` and sensor plugins, and one-click `view_robot.launch.py`. |
| **`intel_sys_navigation`** | `ament_python` | Dead-reckoning `odom_publisher`, decoupled `robot_localization` EKF configuration, Unitree L2 Point-LIO 3D LiDAR odometry, and Nav2 omnidirectional navigation stack. |
| **`intel_sys_sim`** | `ament_cmake` | Gazebo Harmonic simulation environment (`default.sdf`), model spawner, and `ros_gz_bridge` configuration. |
| **`intel_sys_bringup`** | `ament_cmake` | Top-level system coordinators: `robot.launch.py`, `sim.launch.py`, and `teleop.launch.py`. |

---

## 🚀 Quick Start with `run_docker.sh`

[`run_docker.sh`](file:///c:/Users/aahil/Documents/Coding/Robotics/intel_sys_workspaces/run_docker.sh) auto-detects whether you are running on an **x86_64 PC/WSL** or an **ARM64 NVIDIA Jetson Orin NX** and configures GPU runtimes automatically:

```bash
# 1. Start or attach to an interactive container shell:
./run_docker.sh

# 2. Rebuild the image before entering:
./run_docker.sh --build

# 3. Run any ROS 2 command directly inside the container:
./run_docker.sh ros2 launch intel_sys_bringup robot.launch.py
./run_docker.sh colcon test
```

---

## 🔧 Hardware Setup (One-Time Setup on Host)

To ensure the STM32 microcontroller binds consistently to `/dev/rrc` with read/write permissions:

```bash
cd src/intel_sys_hardware/udev
sudo ./install_udev_rules.sh
```

---

## 🎮 Launching the System

### 1. Launch the Physical Robot (Master Command)

Starts the entire stack (robot description + STM32 bridge + sensors + odometry + EKF + Nav2):

```bash
./run_docker.sh ros2 launch intel_sys_bringup robot.launch.py
```

#### Common Arguments:
```bash
# Enable Point-LIO 3D LiDAR odometry in EKF fusion:
./run_docker.sh ros2 launch intel_sys_bringup robot.launch.py use_point_lio:=true

# Open RViz2 navigation interface automatically:
./run_docker.sh ros2 launch intel_sys_bringup robot.launch.py use_rviz:=true
```

### 2. Launch 3D Simulation (Gazebo Harmonic + Nav2 + RViz)

Spins up the Gazebo obstacle world, spawns the robot model with Mecanum drive physics, bridges sensors, and launches Nav2:

```bash
# Full Simulation with GUI and RViz2:
./run_docker.sh ros2 launch intel_sys_bringup sim.launch.py

# Headless Simulation (e.g. for CI / automated testing):
./run_docker.sh ros2 launch intel_sys_bringup sim.launch.py headless:=true
```

### 3. Preview 3D Robot Description in RViz

To inspect the URDF, meshes, and coordinate frames:

```bash
./run_docker.sh ros2 launch intel_sys_description view_robot.launch.py
```

### 4. Teleoperation

Control the mobile base using keyboard or joystick:

```bash
# Keyboard teleoperation:
./run_docker.sh ros2 launch intel_sys_bringup teleop.launch.py use_joy:=false

# Gamepad joystick:
./run_docker.sh ros2 launch intel_sys_bringup teleop.launch.py use_joy:=true joy_dev:=/dev/input/js0
```

---

## 🧪 Testing & Verification

Run the full automated test suite inside the container:

```bash
# Build all packages:
./run_docker.sh colcon build --symlink-install

# Run all unit tests:
./run_docker.sh colcon test

# View test summary:
./run_docker.sh colcon test-result --all --verbose
```