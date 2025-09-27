#!/usr/bin/env bash
# Read-only verification for ROS 2 environment and workspace.
# - Sources ROS 2 Jazzy if available
# - Runs ros2 doctor --report
# - Lists workspace packages with colcon
# - Runs rosdep check (no installation)
# Usage:
#   bash scripts/verify.sh [--ros-distro jazzy]

set -Eeuo pipefail
IFS=$'\n\t'

ROS_DISTRO="jazzy"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro) ROS_DISTRO="${2:-jazzy}"; shift 2;;
    *) echo "[WARN] Unknown arg: $1" >&2; shift;;
  esac
done

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"

log(){ echo "[INFO] $*"; }
warn(){ echo "[WARN] $*" >&2; }

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # Temporarily disable nounset to avoid unbound var issues in ROS setup scripts
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  log "Sourced /opt/ros/${ROS_DISTRO}/setup.bash"
else
  warn "/opt/ros/${ROS_DISTRO}/setup.bash not found. Ensure ROS ${ROS_DISTRO} is installed."
fi

log "ros2 doctor --report"
if ! ros2 doctor --report; then
  warn "ros2 doctor reported issues."
fi

log "colcon list (from ros2_ws)"
if [[ -d ros2_ws ]]; then
  (cd ros2_ws && colcon list || warn "colcon not found or list failed")
else
  warn "ros2_ws not found."
fi

log "rosdep check (no installation)"
if command -v rosdep >/dev/null 2>&1; then
  rosdep check -i --from-paths ros2_ws/src --rosdistro "${ROS_DISTRO}" || true
else
  warn "rosdep not found. Install with: sudo apt install python3-rosdep -y && sudo rosdep init && rosdep update"
fi

log "Verify complete."
