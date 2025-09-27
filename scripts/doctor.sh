#!/usr/bin/env bash
# Read-only environment and workspace checks for this template.
# - Verifies ROS distro availability
# - Prints ros2 doctor report
# - Lists packages with colcon
# - Detects stale CMake caches and suggests fixes
# - Confirms direnv trust and .envrc presence
# Usage:
#   bash scripts/doctor.sh [--ros-distro jazzy]

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

# Check direnv trust
if command -v direnv >/dev/null 2>&1; then
  if [[ -f .envrc ]]; then
    log "direnv installed and .envrc present. Run: direnv allow (if not already trusted)."
  else
    warn ".envrc not found in project root."
  fi
else
  warn "direnv not found. Optional but recommended: sudo apt install direnv -y"
fi

# Source ROS distro if available
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  log "Sourced /opt/ros/${ROS_DISTRO}/setup.bash"
else
  warn "/opt/ros/${ROS_DISTRO}/setup.bash not found. Ensure ROS ${ROS_DISTRO} is installed."
fi

# ros2 doctor (read-only)
if command -v ros2 >/dev/null 2>&1; then
  log "ros2 doctor --report"
  if ! ros2 doctor --report; then
    warn "ros2 doctor reported issues."
  fi
else
  warn "ros2 command not found on PATH."
fi

# colcon list (read-only)
if [[ -d ros2_ws ]]; then
  if command -v colcon >/dev/null 2>&1; then
    log "colcon list (from ros2_ws)"
    (cd ros2_ws && colcon list || warn "colcon list failed")
  else
    warn "colcon not found. Install colcon (e.g., python3-colcon-common-extensions)."
  fi
else
  warn "ros2_ws directory not found."
fi

# Detect stale caches (no mutation)
detect_stale_cache() {
  local rootdir="$1"
  [[ -d "$rootdir" ]] || return 0
  local stale=false
  while IFS= read -r -d '' cache; do
    local cache_dir
    cache_dir=$(dirname "$cache")
    if ! grep -q "^CMAKE_CACHEFILE_DIR:INTERNAL=${cache_dir}$" "$cache"; then
      stale=true
      warn "Stale CMake cache detected: ${cache_dir}. Suggest: bash scripts/clean.sh -y or make build-auto-clean"
    fi
  done < <(find "$rootdir" -maxdepth 2 -type f -name CMakeCache.txt -print0)
  $stale && return 1 || return 0
}

detect_stale_cache "build" || true
detect_stale_cache "ros2_ws/build" || true

log "Doctor checks complete."
