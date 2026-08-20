ARG TARGETARCH

FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0 AS base-arm64
FROM osrf/ros:humble-desktop AS base-amd64
FROM base-${TARGETARCH} AS base

ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake curl git python3-colcon-common-extensions \
    python3-rosdep python3-vcstool python3-serial ros-dev-tools \
    ros-humble-pcl-conversions ros-humble-pcl-ros \
    ros-humble-robot-state-publisher ros-humble-xacro libeigen3-dev libpcl-dev \
 && rm -rf /var/lib/apt/lists/*

FROM base AS third-party-builder
COPY third_party/ /opt/intel_sys/third_party-manifest/
RUN mkdir -p /opt/intel_sys/deps/src \
 && vcs import /opt/intel_sys/deps/src < /opt/intel_sys/third_party-manifest/robot.repos \
 && rosdep update \
 && rosdep install --from-paths /opt/intel_sys/deps/src/point_lio_ros2 \
      /opt/intel_sys/deps/src/unilidar_sdk2/unitree_lidar_ros2/src/unitree_lidar_ros2 \
      /opt/intel_sys/deps/src/realsense-ros --ignore-src --rosdistro ${ROS_DISTRO} --yes \
 && . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon build --merge-install --base-paths /opt/intel_sys/deps/src/point_lio_ros2 \
      /opt/intel_sys/deps/src/unilidar_sdk2/unitree_lidar_ros2/src/unitree_lidar_ros2 \
      /opt/intel_sys/deps/src/realsense-ros --install-base /opt/intel_sys/third_party

FROM base AS development
COPY --from=third-party-builder /opt/intel_sys/third_party /opt/intel_sys/third_party
COPY docker/entrypoint.sh /ros_entrypoint_intel_sys.sh
RUN chmod +x /ros_entrypoint_intel_sys.sh
WORKDIR /workspaces/intel_sys
ENTRYPOINT ["/ros_entrypoint_intel_sys.sh"]
