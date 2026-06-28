ARG TARGETARCH
ARG TARGETPLATFORM

FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0 AS base-arm64
FROM osrf/ros:humble-desktop AS base-amd64
FROM base-${TARGETARCH}

ENV TERM=xterm-256color
ENV COLORTERM=truecolor
ENV FORCE_COLOR=1l
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# base tools — both architectures
RUN apt-get update && apt-get install -y \
    git \
    vim \
    curl \
    gedit \
    nano \
    gh \
    gpg \
    make \
    cmake \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble — only needed on Jetson (amd64 base already has it)
RUN if [ "$(uname -m)" = "aarch64" ]; then \
    add-apt-repository universe && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    apt-get update && apt-get install -y \
        ros-humble-desktop \
        ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*; \
fi

# ROS dev tools — both architectures
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Livox SDK2 — both architectures
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/livox-sdk2 && \
    cd /tmp/livox-sdk2 && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    ldconfig && \
    rm -rf /tmp/livox-sdk2

# RealSense — x86 (desktop testing)
RUN if [ "$(uname -m)" = "x86_64" ]; then \
    mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
        gpg --dearmor | tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] \
        https://librealsense.realsenseai.com/Debian/apt-repo \
        $(lsb_release -cs) main" | \
        tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && apt-get install -y \
        librealsense2-dkms \
        librealsense2-utils \
        librealsense2-dev \
        librealsense2-dbg \
    && rm -rf /var/lib/apt/lists/*; \
fi

# RealSense — Jetson
RUN if [ "$(uname -m)" = "aarch64" ]; then \
    mkdir -p /etc/apt/keyrings && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
        --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    echo "deb https://librealsense.intel.com/Debian/apt-repo \
        $(lsb_release -cs) main" | \
        tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && apt-get install -y \
        librealsense2-utils \
        librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*; \
fi

# Dependencies for point lio
RUN apt-get update && apt-get install -y \ 
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-visualization-msgs \
    libeigen3-dev \
    pcl-tools && \
    rm -rf /var/lib/apt/lists/*

# Setup aliases
COPY ros_aliases.sh /root/ros_aliases.sh
RUN echo "source /root/ros_aliases.sh" >> /root/.bashrc

# Setup directories for later to use volumes with
RUN mkdir -p /root/ros2_ws/src
RUN mkdir -p /root/unilidar_sdk2-2.0.4
RUN mkdir -p /root/unilidar_sdk2-2.0.4/unitree_lidar_ros2/src
RUN mkdir -p /root/catkin_point_lio_unilidar/src
RUN mkdir -p /root/ws_livox/src

# Build the unilidar sdk
COPY unilidar_sdk2-2.0.4/unitree_lidar_sdk /root/unilidar_sdk2-2.0.4/unitree_lidar_sdk
RUN cd /root/unilidar_sdk2-2.0.4/unitree_lidar_sdk && \
    mkdir build && \
    cd build && \
    cmake .. && make -j$(nproc)

# git clone livox driver and point lio
RUN cd /root/ws_livox/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git

WORKDIR /root
CMD ["/bin/bash"]