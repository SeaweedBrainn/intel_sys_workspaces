#!/usr/bin/env bash
set -e

# Intel Sys Universal Docker Runner Script
# Auto-detects x86_64 Desktop vs ARM64 NVIDIA Jetson.
#
# Usage:
#   ./run_docker.sh              # Start or attach to container with interactive bash
#   ./run_docker.sh --build      # Rebuild image before entering
#   ./run_docker.sh <cmd>        # Run a specific command inside the container (e.g. ./run_docker.sh colcon test)

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

REBUILD=false
PASSTHROUGH_ARGS=()

# 1. Hardware Architecture & Runtime Auto-Detection
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    export TARGETARCH=arm64
    export DOCKER_RUNTIME=${DOCKER_RUNTIME:-nvidia}
    PLATFORM_NAME="NVIDIA Jetson (ARM64 with GPU runtime)"
else
    export TARGETARCH=amd64
    export DOCKER_RUNTIME=${DOCKER_RUNTIME:-runc}
    PLATFORM_NAME="Desktop/WSL (x86_64)"
fi

for arg in "$@"; do
    case "$arg" in
        --build|-b)
            REBUILD=true
            ;;
        --jetson)
            export TARGETARCH=arm64
            export DOCKER_RUNTIME=nvidia
            PLATFORM_NAME="NVIDIA Jetson (Forced ARM64)"
            ;;
        --desktop)
            export TARGETARCH=amd64
            export DOCKER_RUNTIME=runc
            PLATFORM_NAME="Desktop (Forced x86_64)"
            ;;
        *)
            PASSTHROUGH_ARGS+=("$arg")
            ;;
    esac
done

# 2. Allow X11 GUI forwarding for RViz / GUI tools if display is available
if [ -n "$DISPLAY" ]; then
    xhost +local:docker 2>/dev/null || true
fi

# 3. Rebuild if requested
if [ "$REBUILD" = true ]; then
    echo "🔨 Building Docker image (intel-sys) for ${PLATFORM_NAME}..."
    docker compose build intel-sys
fi

# 4. Check if container is already running
CONTAINER_ID=$(docker compose ps -q intel-sys 2>/dev/null || true)

if [ -n "$CONTAINER_ID" ] && [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_ID" 2>/dev/null)" = "true" ]; then
    echo "🔌 Connecting to running container ($CONTAINER_ID on ${PLATFORM_NAME})..."
    if [ ${#PASSTHROUGH_ARGS[@]} -eq 0 ]; then
        docker compose exec intel-sys bash
    else
        docker compose exec intel-sys "${PASSTHROUGH_ARGS[@]}"
    fi
else
    echo "🚀 Starting interactive container on ${PLATFORM_NAME}..."
    if [ ${#PASSTHROUGH_ARGS[@]} -eq 0 ]; then
        docker compose run --rm intel-sys bash
    else
        docker compose run --rm intel-sys "${PASSTHROUGH_ARGS[@]}"
    fi
fi
