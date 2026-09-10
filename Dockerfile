ARG TARGETARCH

FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0 AS base-arm64
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg software-properties-common locales \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && add-apt-repository universe \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list \
 && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
 && rm -rf /var/lib/apt/lists/*

FROM osrf/ros:humble-desktop AS base-amd64
FROM base-${TARGETARCH} AS base

ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake curl git python3-colcon-common-extensions \
    python3-rosdep python3-vcstool python3-serial python3-pip python3-tqdm \
    ros-dev-tools gnupg software-properties-common \
    ros-humble-pcl-conversions ros-humble-pcl-ros \
    ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-xacro \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-spatio-temporal-voxel-layer \
    ros-humble-teleop-twist-keyboard ros-humble-teleop-twist-joy ros-humble-joy \
    ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-ros-gz ros-humble-pointcloud-to-laserscan \
    ros-humble-foxglove-bridge \
    libeigen3-dev libpcl-dev mesa-utils pciutils \
 && rm -rf /var/lib/apt/lists/*

RUN install -d -m 0755 /etc/apt/keyrings \
 && curl -fsSL --output /tmp/librealsenseai.asc https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
 && gpg --batch --yes --dearmor --output /etc/apt/keyrings/librealsenseai.gpg /tmp/librealsenseai.asc \
 && rm -f /tmp/librealsenseai.asc \
 && echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo jammy main" > /etc/apt/sources.list.d/librealsense.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends librealsense2-dev librealsense2-utils \
 && rm -rf /var/lib/apt/lists/*

FROM base AS third-party-builder
COPY third_party/ /opt/intel_sys/third_party-manifest/
RUN apt-get update \
 && mkdir -p /opt/intel_sys/deps/src \
 && vcs import /opt/intel_sys/deps/src < /opt/intel_sys/third_party-manifest/robot.repos \
 && (rosdep init 2>/dev/null || true) \
 && rosdep update \
 && rosdep install --from-paths /opt/intel_sys/deps/src/point_lio_ros2 \
      /opt/intel_sys/deps/src/unilidar_sdk2/unitree_lidar_ros2/src/unitree_lidar_ros2 \
      /opt/intel_sys/deps/src/realsense-ros --ignore-src --rosdistro ${ROS_DISTRO} \
      --skip-keys pcl --skip-keys librealsense2 --skip-keys launch_pytest --skip-keys launch-pytest -y \
 && . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon build --merge-install --base-paths /opt/intel_sys/deps/src/point_lio_ros2 \
      /opt/intel_sys/deps/src/unilidar_sdk2/unitree_lidar_ros2/src/unitree_lidar_ros2 \
      /opt/intel_sys/deps/src/realsense-ros --install-base /opt/intel_sys/third_party \
 && rm -rf /var/lib/apt/lists/*

FROM base AS development
COPY --from=third-party-builder /opt/intel_sys/third_party /opt/intel_sys/third_party
COPY docker/entrypoint.sh /ros_entrypoint_intel_sys.sh
RUN chmod +x /ros_entrypoint_intel_sys.sh \
 && echo "source /ros_entrypoint_intel_sys.sh" >> /etc/bash.bashrc \
 && echo "source /ros_entrypoint_intel_sys.sh" >> /root/.bashrc
WORKDIR /workspaces/intel_sys
ENTRYPOINT ["/ros_entrypoint_intel_sys.sh"]
CMD ["bash"]
