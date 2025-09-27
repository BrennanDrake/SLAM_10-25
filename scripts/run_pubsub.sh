#!/usr/bin/env bash
# Launch the template_pkg pub/sub demo after building.
# Usage:
#   bash scripts/run_pubsub.sh [--ros-distro jazzy]

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

if [[ -f "ros2_ws/install/setup.bash" ]]; then
  # Temporarily disable nounset to avoid unbound var issues in overlay setup scripts
  set +u
  # shellcheck disable=SC1091
  source "ros2_ws/install/setup.bash"
  set -u
  log "Sourced workspace overlay: ros2_ws/install/setup.bash"
else
  warn "Workspace overlay not found. Build first: bash scripts/build.sh"
fi

log "Launching template_pkg pub_sub.launch.py"
ros2 launch template_pkg pub_sub.launch.py
