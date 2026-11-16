#!/usr/bin/env bash
# Host-side preflight. Runs on the Jetson, NOT in the container.
# Prints PASS / FAIL / SKIP per check. Exits nonzero only if a check FAILs.
# SKIP means the device is offline (expected for the LiDAR right now), not broken.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-intel-sys-rover:latest}"
LIDAR_IP="${LIDAR_IP:-192.168.1.2}"
MIN_FREE_GB="${MIN_FREE_GB:-10}"
SERIAL_LINK="${SERIAL_LINK:-/dev/rrc}"

FAILS=0
pass() { echo "[PASS] $1"; }
fail() { echo "[FAIL] $1"; FAILS=$((FAILS + 1)); }
skip() { echo "[SKIP] $1"; }

# 1. STM32 serial link
if [[ -e "$SERIAL_LINK" ]]; then
  target="$(readlink -f "$SERIAL_LINK")"
  if [[ -c "$target" ]]; then
    pass "serial $SERIAL_LINK -> $target"
  else
    fail "serial $SERIAL_LINK resolves to $target which is not a char device"
  fi
else
  fail "serial $SERIAL_LINK missing (check udev rules: src/intel_sys_hardware/udev)"
fi

# 2. Docker image
if command -v docker >/dev/null 2>&1 && docker image inspect "$IMAGE" >/dev/null 2>&1; then
  pass "docker image $IMAGE present"
else
  fail "docker image $IMAGE missing (run ./run_docker.sh --build)"
fi

# 3. Disk space on the repo's filesystem
free_gb="$(df -BG --output=avail "$REPO_ROOT" | tail -1 | tr -dc '0-9')"
if [[ -n "$free_gb" && "$free_gb" -ge "$MIN_FREE_GB" ]]; then
  pass "disk free ${free_gb}G (min ${MIN_FREE_GB}G)"
else
  fail "disk free ${free_gb:-?}G below ${MIN_FREE_GB}G"
fi

# 4. LiDAR reachability (offline is SKIP, not FAIL)
if ping -c1 -W1 "$LIDAR_IP" >/dev/null 2>&1; then
  pass "LiDAR $LIDAR_IP reachable"
else
  skip "LiDAR $LIDAR_IP unreachable (expected while LiDAR is offline)"
fi

# 5. RealSense on USB (Intel vendor id 8086)
if command -v lsusb >/dev/null 2>&1 && lsusb -d 8086: 2>/dev/null | grep -q .; then
  pass "Intel USB device(s): $(lsusb -d 8086: | sed 's/^.*ID //' | paste -sd, -)"
else
  skip "no Intel (8086) USB device found; RealSense not enumerated"
fi

# 6. Video nodes
if ls /dev/video* >/dev/null 2>&1; then
  pass "video nodes: $(ls /dev/video* | paste -sd' ' -)"
else
  skip "no /dev/video* nodes"
fi

# 7. Clock sanity (NTP sync is informative; skew is a FAIL only if wildly wrong)
if command -v timedatectl >/dev/null 2>&1; then
  sync="$(timedatectl show -p NTPSynchronized --value 2>/dev/null)"
  if [[ "$sync" == "yes" ]]; then
    pass "clock NTP-synchronized"
  else
    skip "clock not NTP-synchronized (timestamps may drift; OK for offline bags)"
  fi
else
  skip "timedatectl unavailable; clock not checked"
fi
year="$(date +%Y)"
if [[ "$year" -lt 2025 ]]; then
  fail "system clock year $year looks wrong"
fi

echo
if [[ "$FAILS" -gt 0 ]]; then
  echo "preflight: $FAILS FAIL"
  exit 1
fi
echo "preflight: no FAILs"
exit 0
