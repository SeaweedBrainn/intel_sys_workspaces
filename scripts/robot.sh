#!/usr/bin/env bash
# Run docker compose on the robot's services (docker-compose.robot.yml) from the repo root.
#
#   scripts/robot.sh up                # drivers, robot description, dashboard
#   scripts/robot.sh up localization   # + Point-LIO and RTAB-Map
#   scripts/robot.sh up navigation     # + Point-LIO, RTAB-Map and Nav2
#   scripts/robot.sh <compose args>    # anything else, e.g. ps, logs -f lidar, stop navigation, down
#
# Start the dashboard with scripts/dashboard.sh (it supplies the tokens), after the drivers.
set -euo pipefail
cd "$(dirname "$0")/.."
export COMPOSE_FILE=docker-compose.yml:docker-compose.zenoh.yml:docker-compose.robot.yml

if [[ "${1:-}" == "up" ]]; then
  case "${2:-}" in
    "")                       exec docker compose up -d zenohd stm32 lidar description ;;
    localization|navigation)  mkdir -p "${ROBOT_DATA_DIR:-data}"
                              exec docker compose --profile "$2" up -d "$2" ;;
  esac
fi
exec docker compose "$@"
