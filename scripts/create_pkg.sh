#!/usr/bin/env bash
# Helper to create a new ROS 2 package in ros2_ws/src.
# Usage examples:
#   bash scripts/create_pkg.sh --name my_pkg --type ament_cmake --deps "rclcpp std_msgs"
#   bash scripts/create_pkg.sh --name my_py_pkg --type ament_python --deps "rclpy std_msgs"

set -Eeuo pipefail
IFS=$'\n\t'

NAME=""
TYPE="ament_cmake"  # or ament_python
DEPS=""
ROS_DISTRO="jazzy"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --name) NAME="$2"; shift 2;;
    --type) TYPE="$2"; shift 2;;
    --deps) DEPS="$2"; shift 2;;
    --ros-distro) ROS_DISTRO="${2:-jazzy}"; shift 2;;
    -h|--help)
      echo "Usage: $0 --name <pkg_name> [--type ament_cmake|ament_python] [--deps \"rclcpp std_msgs\"] [--ros-distro jazzy]"; exit 0;;
    *) echo "[WARN] Unknown arg: $1" >&2; shift;;
  esac
done

if [[ -z "$NAME" ]]; then
  echo "[ERR ] --name is required" >&2
  exit 1
fi

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"

log(){ echo "[INFO] $*"; }
warn(){ echo "[WARN] $*" >&2; }

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  warn "/opt/ros/${ROS_DISTRO}/setup.bash not found. Ensure ROS ${ROS_DISTRO} is installed."
fi

mkdir -p ros2_ws/src
cd ros2_ws/src

case "$TYPE" in
  ament_cmake)
    CMD=(ros2 pkg create "$NAME" --build-type ament_cmake)
    ;;
  ament_python)
    CMD=(ros2 pkg create "$NAME" --build-type ament_python --dependencies rclpy)
    ;;
  *)
    echo "[ERR ] Unknown --type: $TYPE (use ament_cmake|ament_python)" >&2
    exit 1
    ;;
end

if [[ -n "$DEPS" ]]; then
  # shellcheck disable=SC2206
  DEPS_ARR=($DEPS)
  for d in "${DEPS_ARR[@]}"; do
    CMD+=(--dependencies "$d")
  done
fi

log "Running: ${CMD[*]}"
"${CMD[@]}"

log "Package created: ros2_ws/src/$NAME"
log "Next: bash scripts/build.sh && sjw"
