#!/usr/bin/env bash
# Run docker compose on the robot's services (docker-compose.robot.yml) from the repo root.
#
#   scripts/robot.sh up                # drivers (STM32, LiDAR, camera), robot description, health
#   scripts/robot.sh up localization   # + Point-LIO and RTAB-Map (with the camera; SLAM_CAMERA=false for LiDAR only)
#   scripts/robot.sh up navigation     # + Point-LIO, RTAB-Map, Nav2 and the explorer (idle until started)
#   scripts/robot.sh up follow         # tag following: Point-LIO + Nav2 in odom (no RTAB-Map) + follower
#   scripts/robot.sh up foxglove       # Foxglove bridge on the Tailscale address, port 8765
#   scripts/robot.sh ros2 topic hz /odom   # ros2 CLI on the robot's middleware
#   scripts/robot.sh <compose args>    # anything else, e.g. ps, logs -f lidar, stop navigation, down
#
# The image comes from ROBOT_IMAGE (default intel-sys-rover:latest), usually set in an
# untracked .env in the repo root, e.g. ROBOT_IMAGE=intel-sys-rover:slam. The Zenoh
# router runs the same image unless ZENOH_IMAGE says otherwise.
set -euo pipefail
cd "$(dirname "$0")/.."
export COMPOSE_FILE=docker-compose.yml:docker-compose.zenoh.yml:docker-compose.robot.yml
if [[ -f .env ]]; then set -a; . ./.env; set +a; fi
export ROBOT_IMAGE="${ROBOT_IMAGE:-intel-sys-rover:latest}"
export ZENOH_IMAGE="${ZENOH_IMAGE:-$ROBOT_IMAGE}"

# With the camera on, RTAB-Map waits for camera frames and stops updating without
# them, so refuse to start localization unless the camera is publishing.
require_camera() {
  [[ "${SLAM_CAMERA:-true}" == false ]] && return 0
  running localization && return 0
  if ! running camerastack \
     || ! docker exec camerastack /ros_entrypoint_intel_sys.sh \
          timeout 15 ros2 topic echo --once --no-daemon /camera/camera_info >/dev/null 2>&1; then
    echo "The camera is not publishing /camera/camera_info. Start it with 'scripts/robot.sh up'" >&2
    echo "and check 'scripts/robot.sh logs camera', or run SLAM_CAMERA=false scripts/robot.sh up ${1}." >&2
    exit 1
  fi
}

running() { docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null | grep -q true; }

# A fresh mapping session (the default) makes RTAB-Map delete its database, so move the
# previous map aside first. Not when re-localizing or continuing on it, and not when
# localization is already running (it isn't restarted).
keep_previous_map() {
  DATA="${ROBOT_DATA_DIR:-data}"; mkdir -p "$DATA"
  [[ "${SLAM_LOCALIZATION:-false}" == true || "${SLAM_CONTINUE:-false}" == true ]] && return 0
  if [[ -f "$DATA/rtabmap.db" ]] && ! running localization; then
    mv "$DATA/rtabmap.db" "$DATA/rtabmap_$(date +%Y%m%d_%H%M%S).db"
    echo "previous map kept as $(ls -t "$DATA"/rtabmap_*.db | head -1)"
  fi
}

if [[ "${1:-}" == "up" ]]; then
  echo "image: $ROBOT_IMAGE"
  case "${2:-}" in
    "")                       exec docker compose up -d zenohd stm32 lidar camera description health ;;
    localization)             require_camera localization
                              keep_previous_map
                              exec docker compose --profile localization up -d localization ;;
    # Starts localization too if it isn't running. --no-recreate: never restart a running
    # localization (that would reset odometry, and in mapping mode wipe the map).
    navigation)               require_camera navigation
                              keep_previous_map
                              exec docker compose --profile navigation up -d --no-recreate navigation explorer ;;
    # Following runs without RTAB-Map (SLAM=false), so a running mapping session must stop first.
    follow)                   require_camera follow
                              if running localization \
                                 && docker inspect -f '{{join .Config.Cmd " "}}' localization | grep -q 'slam:=true'; then
                                echo "Localization is running with RTAB-Map. Stop it first: scripts/robot.sh stop navigation localization" >&2
                                exit 1
                              fi
                              export SLAM=false
                              exec docker compose --profile follow up -d --no-recreate follow ;;
    foxglove)                 export FOXGLOVE_ADDRESS="${FOXGLOVE_ADDRESS:-$(tailscale ip -4 2>/dev/null | head -1 || true)}"
                              exec docker compose --profile foxglove up -d foxglove ;;
  esac
fi
# ros2 inside the LiDAR container (Zenoh settings, workspace sourced), e.g.
#   scripts/robot.sh ros2 topic hz /lidar/points_raw
if [[ "${1:-}" == "ros2" ]]; then
  exec docker compose exec lidar /ros_entrypoint_intel_sys.sh "$@"
fi
exec docker compose "$@"
