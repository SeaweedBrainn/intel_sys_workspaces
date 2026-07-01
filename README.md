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
    git clone --recurse-submodules https://github.com/SeaweedBrainn/intel_sys_workspaces.git
    cd intel_sys_workspaces
    git submodule update --recursive
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
This repository contains the source code for point lio (Our SLAM framework). You can find the documentation here: https://github.com/dfloreaa/point_lio_ros2. Make sure Section 3.3 is completed before this. To build this code run the following:
```bash
sourceros2
sourcelivox
cd /root/catkin_point_lio_unilidar
colcon build
```
## 4. Using Different Components
Make sure you are inside the docker container. There are a number of bash functions and aliases which you can find in this repo and in the docker container. To view all the bash functions run:
```bash
gedit /root/ros_aliases.sh
```
### 4.1 Launching Lidar
In the terminal run:
```bash
launchlidar
```
- In the left panel of the rviz window that opened up, change fixed frame to world for a upright view.
- Make sure the lidar is connected via ethernet, and the IPv4 configuration is as follows:
    - Address: 192.168.1.2
    - Netmask: 255.255.255.0

### 4.2 Launching Slam
In the terminal run:
```bash
launchslam
```
Make sure you completed Section 4.1 prior to doing this section

### 4.2 Launching RealSense Viewer
In the terminal run:
```bash
realsense-viewer
```
Make sure the realsense camera is connected

### 4.3 Making the TestBench Teleoperate
Make sure you pressed on the button on the test bench to turn on the stm motor driver, as well as make sure you have connected the USB-A from the testbench to your device prior to doing these steps. The testbench here is the HiWonder JetAuto.
#### 4.3.1 Running the ROS2 Communication Node
Open a terminal session and run:
```bash
sourceros2
cd /root/ros2_ws && source install/setup.sh
ros2 launch ros_robot_controller ros_robot_controller.launch.py
```
You may get a udev-rules error here. The problem is that the USB device probably shows itself as /dev/ttyACM0 by default, however, the HiWonder library marks the device to /dev/rrc in the python code. To fix the error.
Open a terminal on your host (not the container) and run:
```bash
lsusb
```
In the output, locate the USB Device (In this case the Testbench HiWonder). For instance here it was:
```bash
Bus 001 Device 012: ID 1a86:55d4 QinHeng Electronics USB Single Serial
```
After that run:
```bash
# Replace 1a86 and 55d4 with your correct info from the above field
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", SYMLINK+="rrc", MODE="0666"' | sudo tee /etc/udev/rules.d/99-jetauto-rrc.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```


#### 4.3.2 Starting the Odometery Publisher Node
Open another terminal session and run:
```bash
sourceros2
cd /root/ros2_ws && source install/setup.sh
ros2 run controller odom_publisher
```

#### 4.3.3 Starting the Teleoperation Node
Open another terminal session and run:
```bash
sourceros2
cd /root/ros2_ws && source install/setup.sh
ros2 run robotics_URC_package testbench_teleop
```

### 4.4 Using py-trees-ros
We are using py-trees-ros for our behavior trees. You can find the documentation here: https://github.com/splintered-reality/py_trees_ros.

- To run the behavior trees open a terminal session and run:
    ```bash
    sourceros2
    cd /root/ros2_ws && source install/setup.sh
    ros2 run robotics_URC_package robot
    ```
- To visualize the tree while its running, open another terminal session and run:
    ```bash
    sourceros2
    py-trees-tree-viewer
    ```

## 5. Working With git
There are a number of commands associated when working with git with this repository.
- While in the base folder running `git add .` will add all the changes to the next commit. Here `.` is means the current working repository, you could change it to specific changes too.
- Running `git commit -m "Your Message"` will commit your changes to remote.
- Running `git push` will push your changes online to github.
- Running `git pull` will all changes from remote onto your local system
- Running `git switch -c <branch-name>` will create a new branch and switch to it.
- Running `git branch <branch-name>` will pull an existing branch from remote and switch to it.
- Running `git branch` will show all the branches locally on your computer.
- Running `git switch <branch-name>` will switch to one of those branches locally on your computer.

## 6. Working With Docker Containers
There are a number of commands associated when working with docker cli with this repository.
- To build a docker container with the DockerFile simply go to the base folder and run `docker compose build`
- To open a new container associated with the repo simply go to the base folder and run `docker compose up -d`.
- To exit a container just run `exit` while in the container.
- To stop a container, first exit the container and then run `docker compose stop` in the base folder of this repo
- To start a stopped container, go to the base folder of this repo and run `docker compose start`
- To delete a container, first exit the container and then run `docker compose down`. Be careful if you delete a container, all data not in the three folders aka catkin_point_lio_unilidar, ros2_ws, and unilidar_sdk2-2.0.4 will be lost.
- To enter the terminal shell of an already running docker container go to the base folder and run `docker compose exec <container-name> bash`
- To view all current running containers run `docker container list`
- To view all current running and pas stopped containers run `docker container list -a`
- To delete a docker container run `docker rm <container-name>`
- To delete all docker containers run `docker container prune`
- To view all docker images run `docker images list`
- To delete a docker image run `docker rmi <image-name>`
- To delete all build cache for a fresh build from the dockerfile run `docker builder prune`

## 7. Working With ROS2
There are a number of commands associated when working with ROS2 cli with this repository.
- You need to source ROS2 before you can use any ros2 commands, you can do it via `source /opt/ros/humble/setup.bash` or via `sourceros2`
- After you make changes to a ROS2 package you need to build it again and then source the installation of that package.
    - To build a ros2 package, go into the workspace with that package, this folder should have just src and maybe build, log, and install folders. Run `colcon build`. If you want to build only specific packages in that workspace run `colcon build --packages-select <package-name>`.
    - To source the installation of that package, again go into the workspace and run `source install/setup.sh`. This sourcing is different from sourcing ros2, as this places the workspace packages for us to use. You need to do this to work with any package in the workspace. 
- To view all ros2 packages run `ros2 pkg list`.
- To run a ros2 node run `ros2 run <package-name> <node-name>`.
- To run a ros2 launch file run `ros2 launch <package-name> <launch-file-name>`.
- To view all current nodes run `ros2 node list`.
- To view all current topics run `ros2 topic list`.

## 8. Using ROS2 Bags
You might have a rosbag or a ros2 bag with you that you can run test data. In this repository one such example can be extracted to `ros2_ws/src/robotics_URC_package/robotics_URC_package/rosbags`. 

### 8.1 Getting The ROS1 Bag File for This Project
The rosbag file is too big to be commited to github. Get the file for this repository from here: https://drive.google.com/file/d/1w7TD9ZcxOy_qwZ4VXqc-komV-KagjOmv. It was found in this repository: https://github.com/unitreerobotics/point_lio_unilidar.

After downloading the file from the google drive link. Run the following (on your host NOT the container):
```bash
sudo apt install unzip
unzip ~/Downloads/unilidar_l2.zip -d ~/intel_sys_workspaces/ros2_ws/src/robotics_URC_package/robotics_URC_package/rosbags/
# Replace the download path if your file was downloaded somewhere else
```

### 8.2 Converting ROS1 bag files to ROS2
The `.bag` files are the files supported by ROS1 but we are using ROS2 so we need to convert it to its respective ROS2 sqlite database. We are using the rosbags python library for that.

Simply run (inside the container):
```bash
sourceros2
rosbags-convert --src <source-bag-file> --dst <name-of-bag-file-with-a-`_0`-appended-at-end>
rm -rf <destination-from-above>/metadata.yaml
ros2 bag reindex <desination-from-above> -s sqlite3
# Here change sqlite3 to mcap if your data base file inside the destination is .mcap instead of .db3
```
Make sure your destination ends with `_0` (or especially the .db3 file inside), otherwise the reindexing in the last line will fail.

For example, here the rosbag file is `unilidar_l2.bag`. Make sure you completed Section 8.1 before this. So we would run:
```bash
sourceros2
rosbags-convert --src /root/ros2_ws/src/robotics_URC_package/robotics_URC_package/rosbags/unilidar_l2.bag --dst /root/ros2_ws/src/robotics_URC_package/robotics_URC_package/rosbags/unilidar_l2_0
rm -rf /root/ros2_ws/src/robotics_URC_package/robotics_URC_package/rosbags/unilidar_l2_0/metadata.yaml
ros2 bag reindex /root/ros2_ws/src/robotics_URC_package/robotics_URC_package/rosbags/unilidar_l2_0 -s sqlite3
```
This made the ros2 bag folder  `unilidar_l2_0` and then reindexed it for us to run later.

### 8.3 Running ROS2 bag files
Simply run the generated ROS2 bag folder via
```bash
sourceros2
ros2 bag play <ros2-bag-folder>
```
For example, here the ros2 bag folder is `unilidar_l2_0`. Make sure the folder (or especially the .db3 file inside the folder) ends with a `_0`. This ros2 bag plays the data provided by unitree unilidar l2. Make sure you completed Section 8.2 before doing this. You can run it by:
```bash
sourceros2
ros2 bag play /root/ros2_ws/src/robotics_URC_package/robotics_URC_package/rosbags/unilidar_l2_0
```
If you get an error try reindexing the bag folder via Section 8.1. After running this you can open another terminal session and run SLAM (Look at Section 4.2).