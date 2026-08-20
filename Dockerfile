ARG TARGETARCH

FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0 AS base-arm64
FROM osrf/ros:humble-desktop AS base-amd64
FROM base-${TARGETARCH} AS base

ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake curl git python3-colcon-common-extensions \
    python3-rosdep python3-vcstool python3-serial python3-pip python3-tqdm \
    ros-dev-tools gnupg software-properties-common \
    ros-humble-pcl-conversions ros-humble-pcl-ros \
    ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-xacro \
    ros-humble-robot-localization ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-teleop-twist-keyboard ros-humble-teleop-twist-joy ros-humble-joy \
    ros-humble-cv-bridge ros-humble-image-transport \
    libeigen3-dev libpcl-dev \
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
RUN chmod +x /ros_entrypoint_intel_sys.sh
WORKDIR /workspaces/intel_sys
ENTRYPOINT ["/ros_entrypoint_intel_sys.sh"]
CMD ["bash"]
