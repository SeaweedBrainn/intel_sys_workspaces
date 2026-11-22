#!/usr/bin/env bash
# Run docker compose on the robot's services (docker-compose.robot.yml) from the repo root.
#
#   scripts/robot.sh up                # drivers and robot description
#   scripts/robot.sh up localization   # + Point-LIO and RTAB-Map
#   scripts/robot.sh up navigation     # + Point-LIO, RTAB-Map and Nav2
#   scripts/robot.sh up foxglove       # Foxglove bridge on the Tailscale address, port 8765
#   scripts/robot.sh ros2 topic hz /odom   # ros2 CLI on the robot's middleware
#   scripts/robot.sh <compose args>    # anything else, e.g. ps, logs -f lidar, stop navigation, down
#
# Start the dashboard with scripts/dashboard.sh (it supplies the tokens), after the drivers.
set -euo pipefail
cd "$(dirname "$0")/.."
export COMPOSE_FILE=docker-compose.yml:docker-compose.zenoh.yml:docker-compose.robot.yml

if [[ "${1:-}" == "up" ]]; then
  case "${2:-}" in
    "")                       exec docker compose up -d zenohd stm32 lidar description ;;
    localization)             DATA="${ROBOT_DATA_DIR:-data}"; mkdir -p "$DATA"
                              # Mapping starts a new database; keep the previous map instead of
                              # letting RTAB-Map delete it (only when localization is not running).
                              if [[ "${SLAM_LOCALIZATION:-false}" != true && -f "$DATA/rtabmap.db" ]] \
                                 && ! docker inspect -f '{{.State.Running}}' localization 2>/dev/null | grep -q true; then
                                mv "$DATA/rtabmap.db" "$DATA/rtabmap_$(date +%Y%m%d_%H%M%S).db"
                                echo "previous map kept as $(ls -t "$DATA"/rtabmap_*.db | head -1)"
                              fi
                              exec docker compose --profile localization up -d localization ;;
    # --no-recreate: never restart a running localization (that would reset odometry,
    # and in mapping mode wipe the map); start it only if it is not running yet.
    navigation)               mkdir -p "${ROBOT_DATA_DIR:-data}"
                              exec docker compose --profile navigation up -d --no-recreate navigation ;;
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
