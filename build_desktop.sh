#!/bin/bash
set -e

export TARGETARCH=amd64
export DOCKER_RUNTIME=runc

echo "================================================"
echo " Building autonomy-jetson for x86 desktop"
echo "================================================"

# create workspace folders if they don't exist
echo "Ensuring workspace directories exist..."
mkdir -p ros2_ws/src
mkdir -p unilidar_sdk2-2.0.4/unitree_lidar_ros2/src
mkdir -p catkin_point_lio_unilidar/src

xhost +local:docker

echo "[1/3] Building image..."
docker compose build

echo "[2/3] Starting container..."
docker compose up -d

echo "[3/3] Opening shell — type 'exit' to leave"
echo "(container keeps running in background)"
echo ""
docker compose exec autonomy-jetson bash