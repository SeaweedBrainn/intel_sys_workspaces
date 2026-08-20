#!/bin/bash
set -e

export TARGETARCH=amd64
export DOCKER_RUNTIME=runc

echo "================================================"
echo " Building intel-sys for x86 desktop"
echo "================================================"

# Allow local docker GUI access
xhost +local:docker 2>/dev/null || true

echo "[1/3] Building image..."
docker compose build

echo "[2/3] Starting container..."
docker compose up -d

echo "[3/3] Opening shell — type 'exit' to leave"
echo "(container keeps running in background)"
echo ""
docker compose exec intel-sys bash