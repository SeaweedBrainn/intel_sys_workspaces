#!/usr/bin/env bash
# Build and test the workspace inside the rover container.
#
# Usage (from the host):
#   ./run_docker.sh bash scripts/run_tests.sh            # build + test
#   ./run_docker.sh bash scripts/run_tests.sh --no-build # test only
#
# Note: the hardware unit tests import rclpy and intel_sys_interfaces, so the
# workspace overlay must be built AND sourced. This script builds and then
# sources install/setup.bash before running colcon test.
set -uo pipefail

WS="${WS:-/workspaces/intel_sys}"
DO_BUILD=1
[[ "${1:-}" == "--no-build" ]] && DO_BUILD=0

cd "$WS" || { echo "[FAIL] workspace $WS not found"; exit 1; }

# Source ROS base and third-party overlay. Container entrypoint does this too,
# but be explicit so the script also works when run via bash -c.
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
[[ -f /opt/intel_sys/third_party/local_setup.bash ]] && source /opt/intel_sys/third_party/local_setup.bash
set -u

STAGE_RESULTS=()
record() { STAGE_RESULTS+=("$1 $2"); }

if [[ "$DO_BUILD" -eq 1 ]]; then
  if colcon build --symlink-install --parallel-workers 6 --event-handlers console_direct+; then
    record PASS "build"
  else
    record FAIL "build"
  fi
fi

# Overlay must be sourced before tests run (see note above).
if [[ -f install/setup.bash ]]; then
  set +u; source install/setup.bash; set -u
else
  record FAIL "overlay (install/setup.bash missing; build first)"
fi

if colcon test --event-handlers console_direct+; then
  record PASS "colcon test"
else
  record FAIL "colcon test"
fi

colcon test-result --verbose > /tmp/_colcon_result.txt 2>&1 || true
cat /tmp/_colcon_result.txt
rm -f /tmp/_colcon_result.txt

echo
echo "=== run_tests summary ==="
FAILED=0
for line in "${STAGE_RESULTS[@]}"; do
  echo "$line"
  [[ "$line" == FAIL* ]] && FAILED=1
done

# Per-package failure counts from colcon's result summary.
if colcon test-result --all >/dev/null 2>&1; then
  :
else
  echo "FAIL colcon test-result reports failing tests"
  FAILED=1
fi

exit "$FAILED"
