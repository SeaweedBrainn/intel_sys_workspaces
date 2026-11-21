#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
echo "Installing Intel Sys udev rules to /etc/udev/rules.d/..."
sudo cp "$SCRIPT_DIR"/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "Udev rules installed successfully."
