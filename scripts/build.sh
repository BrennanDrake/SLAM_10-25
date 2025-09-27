#!/usr/bin/env bash
# Build the ROS 2 workspace with sensible defaults.
# - Sources ROS Jazzy
# - Builds from ros2_ws directory
# - Tries --mixin release first, then falls back to CMAKE_BUILD_TYPE=Release
# Usage:
#   bash scripts/build.sh [--ros-distro jazzy] [--mixin release] [--auto-clean]

set -Eeuo pipefail
IFS=$'\n\t'

ROS_DISTRO="jazzy"
MIXIN="release"
AUTO_CLEAN=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro) ROS_DISTRO="${2:-jazzy}"; shift 2;;
    --mixin) MIXIN="${2:-release}"; shift 2;;
    --auto-clean) AUTO_CLEAN=true; shift;;
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

if [[ ! -d ros2_ws/src ]]; then
  warn "ros2_ws/src not found; creating it"
  mkdir -p ros2_ws/src
fi

# Detect stale caches (CMakeCache created in a different directory)
detect_stale_cache() {
  local rootdir="$1"
  [[ -d "$rootdir" ]] || return 0
  local stale=false
  while IFS= read -r -d '' cache; do
    local cache_dir
    cache_dir=$(dirname "$cache")
    # Expect CMAKE_CACHEFILE_DIR to match the actual cache directory
    if ! grep -q "^CMAKE_CACHEFILE_DIR:INTERNAL=${cache_dir}$" "$cache"; then
      stale=true
      warn "Stale CMake cache detected: ${cache_dir}. Recommend: bash scripts/clean.sh -y"
    fi
  done < <(find "$rootdir" -maxdepth 2 -type f -name CMakeCache.txt -print0)
  $stale && return 1 || return 0
}

# Check both top-level and workspace build directories
if ! detect_stale_cache "build" || ! detect_stale_cache "ros2_ws/build"; then
  if $AUTO_CLEAN; then
    log "Auto-clean enabled. Running scripts/clean.sh -y due to stale caches."
    bash scripts/clean.sh -y
  else
    if [[ -t 0 && -t 1 ]]; then
      read -r -p "Stale caches detected. Clean now? [y/N] " reply || reply=""
      if [[ "$reply" =~ ^[Yy]$ ]]; then
        bash scripts/clean.sh -y
      else
        warn "Continuing without cleaning. Build may fail."
      fi
    else
      warn "Stale caches detected in non-interactive mode. Consider: bash scripts/clean.sh -y or use --auto-clean."
    fi
  fi
fi

cd ros2_ws

if command -v colcon >/dev/null 2>&1; then
  log "Building workspace with mixin: ${MIXIN}"
  if colcon build --mixin "${MIXIN}"; then
    log "Build succeeded with mixin ${MIXIN}."
  else
    warn "Build with mixin ${MIXIN} failed. Falling back to CMAKE_BUILD_TYPE=Release."
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  fi
  log "To use this overlay now: source install/setup.bash"
else
  warn "colcon not found. Install colcon (e.g., python3-colcon-common-extensions)."
fi
