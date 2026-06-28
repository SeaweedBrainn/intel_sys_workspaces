# INTEL_SYS_WORKSPACES
This repository contains the ROS2 workspaces and docker info for the intel sys team from SJSU Robotics.

## Setup Instructions and Pre-requisities
You need to have Docker Engine installed on your computer. The following system works for Ubuntu 22.04 Jammy and NVIDIA Jetson Orin NX 16GB running Jetpack 6.2.

## 1. Docker Setup
### 1.1 Installing Docker Engine
Follow the instructions here: https://docs.docker.com/engine/install/ubuntu/ OR follow the instructions below:

- Uninstall conflicting packages
    ```bash
    sudo apt remove $(dpkg --get-selections docker.io docker-compose docker-compose-v2 docker-doc podman-docker containerd runc | cut -f1)
    ```
- Set up Docker's apt repository
    ```bash
    # Add Docker's official GPG key:
    sudo apt update
    sudo apt install ca-certificates curl
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

    # Add the repository to Apt sources:
    sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
    Types: deb
    URIs: https://download.docker.com/linux/ubuntu
    Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
    Components: stable
    Architectures: $(dpkg --print-architecture)
    Signed-By: /etc/apt/keyrings/docker.asc
    EOF

    sudo apt update
    ```
- Install Docker packages
    ```bash
    sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    ```
- After installation ensure docker is running
    ```bash
    sudo systemctl status docker
    ```
- If Docker is not running then manually turn it on
    ```bash
    sudo systemctl start docker
    ```
### 1.2 Using docker commands without sudo
Currently, you can only use docker commands with sudo. To be able to use docker commands without sudo either follow the steps in here: https://docs.docker.com/engine/install/linux-postinstall/ OR follow the steps below:
- Create docker group
    ```bash
    sudo groupadd docker
    ```
- Add User to docker group
    ```bash
    sudo usermod -aG docker $USER
    ```
- Restart your computer or if you are using a virtual machine then restart the virtual machine

## 2. Building Docker Image and Running Docker Container for the First Time
- Clone the repository into your root folder
    ```bash
    cd ~
    git clone https://github.com/SeaweedBrainn/intel_sys_workspaces.git
    cd intel_sys_workspaces
    ```
- Set the build setup files as executables
    ```bash
    chmod +x build_desktop.sh build_jetson.sh
    ```
- Run the executable depending on your platform
    - For x86_64 systems (or normal Ubuntu 22.04), run:
        ```bash
        ./build_desktop.sh
        ```
    - For aarch64 systems (NVIDIA Jetson Orin NX), run:
        ```bash
        ./build_jetson.sh
        ```

## 3. Setting Up Workspaces Inside Docker Container
Make sure you are inside the docker container
### 3.1 ros2_ws
This workspace contains our custom code within the robotics_URC_package as well as the source code from the test bench. To build this code run the following:
```bash
cd /root/ros2_ws
sourceros2
colcon build
```

### 3.2 unilidar_sdk2-2.0.4
This is the ros2 source code and the sdk for the unitiree unilidar L2. To build this code run the following:
```bash
cd /root/unilidar_sdk2-2.0.4/unitree_lidar_ros2
sourceros2
colcon build
```

### 3.3 ws_livox
This repository only appears after you have completed Section 2. It contains the source code for the Livox Driver for the Lidar which is needed by point lio (Our SLAM framework) to run. You can find the documentation here: https://github.com/Livox-SDK/livox_ros_driver2. To build this code run the following:
```bash
cd /root/ws_livox/src/livox_ros_driver2
sourceros2
./build.sh humble
```

### 3.4 catkin_point_lio_unilidar
This repository only appears after you have completed Section 2. It contains the source code for point lio (Our SLAM framework). You can find the documentation here: https://github.com/dfloreaa/point_lio_ros2. Make sure Section 3.3 is completed before this. To build this code run the following:
```bash
sourcelivox
cd /root/catkin_point_lio_unilidar
colcon build
```