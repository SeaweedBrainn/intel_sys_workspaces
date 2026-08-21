#!/usr/bin/env bash
set -e

# Intel Sys Universal Docker Runner Script
# Guarantees that multiple terminal sessions connect to the same shared container.
#
# Usage:
#   ./run_docker.sh              # Start or attach to the shared container shell
#   ./run_docker.sh --build      # Rebuild image and restart shared container
#   ./run_docker.sh --down       # Stop and remove the shared container
#   ./run_docker.sh <cmd>        # Run a command inside the shared container (e.g. ./run_docker.sh colcon test)

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

REBUILD=false
STOP_CONTAINER=false
PASSTHROUGH_ARGS=()
ENABLE_NVIDIA=auto
FORCED_ARCH=""

# 1. Parse arguments
for arg in "$@"; do
    case "$arg" in
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
        --jetson)
            FORCED_ARCH="arm64"
            ;;
        --desktop)
            FORCED_ARCH="amd64"
            ;;
        *)
            PASSTHROUGH_ARGS+=("$arg")
            ;;
    esac
done

# 2. Hardware Architecture & GPU Auto-Detection
ARCH=${FORCED_ARCH:-$(uname -m)}
if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    export TARGETARCH=arm64
    export DOCKER_RUNTIME=${DOCKER_RUNTIME:-nvidia}
    export COMPOSE_FILE="docker-compose.yml:docker-compose.nvidia.yml"
    PLATFORM_NAME="NVIDIA Jetson (ARM64 with GPU runtime)"
else
    export TARGETARCH=amd64
    export DOCKER_RUNTIME=${DOCKER_RUNTIME:-runc}

    # Auto-detect NVIDIA GPU availability on x86_64 Desktop/WSL
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
        PLATFORM_NAME="Desktop/WSL (x86_64 with NVIDIA GPU acceleration)"
    else
        export COMPOSE_FILE="docker-compose.yml"
        PLATFORM_NAME="Desktop/WSL (x86_64 standard CPU)"
    fi
fi

# 3. Stop container if requested
if [ "$STOP_CONTAINER" = true ]; then
    echo "Stopping intel-sys container..."
    docker compose down
    exit 0
fi

# 3. Allow X11 GUI forwarding for Gazebo if display is available
if [ -n "$DISPLAY" ]; then
    xhost +local:docker 2>/dev/null || true
fi

# 4. Rebuild if requested
if [ "$REBUILD" = true ]; then
    echo "Building Docker image (intel-sys) for ${PLATFORM_NAME}..."
    docker compose down 2>/dev/null || true
    docker compose build intel-sys
fi

# 5. Ensure the shared background container is running
RUNNING=$(docker inspect -f '{{.State.Running}}' intel-sys 2>/dev/null || echo "false")

if [ "$RUNNING" != "true" ]; then
    echo "Starting shared intel-sys container on ${PLATFORM_NAME}..."
    docker compose up -d intel-sys
fi

# 6. Execute inside the shared container with interactive TTY support
if [ ${#PASSTHROUGH_ARGS[@]} -eq 0 ]; then
    echo "Connected to shared container (intel-sys). Type 'exit' to leave shell."
    if [ -t 0 ]; then
        docker exec -it intel-sys bash
    else
        docker exec -i intel-sys bash
    fi
else
    if [ -t 0 ]; then
        docker exec -it intel-sys /ros_entrypoint_intel_sys.sh "${PASSTHROUGH_ARGS[@]}"
    else
        docker exec -i intel-sys /ros_entrypoint_intel_sys.sh "${PASSTHROUGH_ARGS[@]}"
    fi
fi
