#!/usr/bin/env bash
set -e

# Intel Sys Universal Docker Runner Script
# Guarantees that multiple terminal sessions connect to the same shared container.
#
# Usage:
#   ./run_docker.sh              # Start or attach to the container (auto-detects sim or rover)
#   ./run_docker.sh --sim        # Force simulation container (intel-sys-sim)
#   ./run_docker.sh --rover      # Force rover hardware container (intel-sys-rover)
#   ./run_docker.sh --build      # Rebuild image and restart shared container
#   ./run_docker.sh --down       # Stop and remove running containers
#   ./run_docker.sh <cmd>        # Run a command inside the container (e.g. ./run_docker.sh colcon test)

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

export DOCKER_BUILDKIT=1

REBUILD=false
STOP_CONTAINER=false
PASSTHROUGH_ARGS=()
ENABLE_NVIDIA=auto
TARGET_MODE=""

# 1. Parse arguments
for arg in "$@"; do
    case "$arg" in
        --sim)
            TARGET_MODE="sim"
            ;;
        --rover)
            TARGET_MODE="rover"
            ;;
        --build|-b)
            REBUILD=true
            ;;
        --down|--stop)
            STOP_CONTAINER=true
            ;;
        --nvidia)
            ENABLE_NVIDIA=true
            ;;
        --cpu)
            ENABLE_NVIDIA=false
            ;;
        *)
            PASSTHROUGH_ARGS+=("$arg")
            ;;
    esac
done

# 2. Hardware Architecture & Default Target Auto-Detection
HOST_ARCH=$(uname -m)
if [ -z "$TARGET_MODE" ]; then
    if [ "$HOST_ARCH" = "aarch64" ] || [ "$HOST_ARCH" = "arm64" ]; then
        TARGET_MODE="rover"
    else
        TARGET_MODE="sim"
    fi
fi

if [ "$TARGET_MODE" = "rover" ]; then
    SERVICE="intel-sys-rover"
    export DOCKER_RUNTIME=${DOCKER_RUNTIME:-nvidia}
    export COMPOSE_FILE="docker-compose.yml:docker-compose.nvidia.yml"
    PLATFORM_NAME="Rover Hardware [Jetson ARM64]"
else
    SERVICE="intel-sys-sim"
    export DOCKER_RUNTIME=${DOCKER_RUNTIME:-runc}

    # Auto-detect NVIDIA GPU availability on x86_64 Desktop/WSL for Gazebo
    HAS_NVIDIA=false
    if [ "$ENABLE_NVIDIA" = "true" ]; then
        HAS_NVIDIA=true
    elif [ "$ENABLE_NVIDIA" = "auto" ]; then
        if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
            HAS_NVIDIA=true
        fi
    fi

    if [ "$HAS_NVIDIA" = "true" ]; then
        export COMPOSE_FILE="docker-compose.yml:docker-compose.nvidia.yml"
        PLATFORM_NAME="Simulation [x86_64 with NVIDIA GPU acceleration]"
    else
        export COMPOSE_FILE="docker-compose.yml"
        PLATFORM_NAME="Simulation [x86_64 standard CPU]"
    fi
fi

# 3. Stop containers if requested
if [ "$STOP_CONTAINER" = true ]; then
    echo "Stopping all intel-sys containers..."
    docker compose down
    exit 0
fi

# 4. Allow X11 GUI forwarding for Gazebo/RViz if display is available
if [ -n "$DISPLAY" ]; then
    xhost +local:docker 2>/dev/null || true
fi

# 5. Rebuild if requested
if [ "$REBUILD" = true ]; then
    echo "Building Docker image (${SERVICE}) for ${PLATFORM_NAME}..."
    docker compose down 2>/dev/null || true
    docker compose build "$SERVICE"
fi

# 6. Ensure the shared background container is running
RUNNING=$(docker inspect -f '{{.State.Running}}' "$SERVICE" 2>/dev/null || echo "false")

if [ "$RUNNING" != "true" ]; then
    echo "Starting ${SERVICE} container on ${PLATFORM_NAME}..."
    docker compose up -d "$SERVICE"
fi

# 7. Execute inside the shared container with interactive TTY support
if [ ${#PASSTHROUGH_ARGS[@]} -eq 0 ]; then
    echo "Connected to ${SERVICE} (${PLATFORM_NAME}). Type 'exit' to leave shell."
    if [ -t 0 ]; then
        docker exec -it "$SERVICE" bash
    else
        docker exec -i "$SERVICE" bash
    fi
else
    if [ -t 0 ]; then
        docker exec -it "$SERVICE" /ros_entrypoint_intel_sys.sh "${PASSTHROUGH_ARGS[@]}"
    else
        docker exec -i "$SERVICE" /ros_entrypoint_intel_sys.sh "${PASSTHROUGH_ARGS[@]}"
    fi
fi
