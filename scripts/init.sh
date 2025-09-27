#!/usr/bin/env bash
# Bootstrap this template after cloning.
# - Sets up colcon mixins (release/debug profiles)
# - Optionally runs `direnv allow` to auto-source ROS and this workspace overlay
# - Optionally runs rosdep checks or installs dependencies
# - Builds the workspace
#
# Usage:
#   scripts/init.sh [--ros-distro jazzy] [--install-deps] [--no-mixins] [--no-direnv-allow] [--no-build] [--force-reinit] [--verbose]
#
# Defaults:
#   ROS_DISTRO=jazzy
#   install deps: false (use --install-deps to run rosdep install)
#   mixins: enabled
#   direnv allow: enabled
#   build: enabled
#
# Notes:
#   - This script avoids sudo by default. If you pass --install-deps, rosdep may prompt for sudo.
#   - If colcon mixins are missing, we will add/update the default mixin repo.
#   - If direnv is installed, we will run `direnv allow` in the project root.

set -Eeuo pipefail
IFS=$'\n\t'

# --- args ---
ROS_DISTRO="jazzy"
DO_INSTALL_DEPS=false
DO_MIXINS=true
DO_DIRENV_ALLOW=true
DO_BUILD=true
VERBOSE=false
DO_FORCE=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro)
      ROS_DISTRO="${2:-jazzy}"; shift 2 ;;
    --install-deps)
      DO_INSTALL_DEPS=true; shift ;;
    --no-mixins)
      DO_MIXINS=false; shift ;;
    --no-direnv-allow)
      DO_DIRENV_ALLOW=false; shift ;;
    --no-build)
      DO_BUILD=false; shift ;;
    --force-reinit)
      DO_FORCE=true; shift ;;
    --verbose)
      VERBOSE=true; shift ;;
    -h|--help)
      grep '^# ' "$0" | sed 's/^# \{0,1\}//' ; exit 0 ;;
    *)
      echo "[WARN] Unknown arg: $1" >&2; shift ;;
  esac
done

# --- helpers ---
log()   { echo "[INFO] $*"; }
warn()  { echo "[WARN] $*" >&2; }
error() { echo "[ERR ] $*" >&2; exit 1; }
run()   { if $VERBOSE; then set -x; fi; eval "$*"; if $VERBOSE; then set +x; fi; }

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
cd "$PROJECT_ROOT"

log "Project root: $PROJECT_ROOT"

# --- init stamp: skip only if stamp matches current project root (unless forced) ---
STATE_DIR="$PROJECT_ROOT/.windsurf/state"
STAMP_PATH="$STATE_DIR/init_done.json"
if [[ -f "$STAMP_PATH" && "$DO_FORCE" != "true" ]]; then
  if grep -q "\"project_root\": \"$PROJECT_ROOT\"" "$STAMP_PATH"; then
    log "Initialization stamp matches current project root. Skipping init. Use --force-reinit to run again."
    exit 0
  else
    warn "Initialization stamp exists but project_root differs. Re-initializing and updating stamp."
  fi
fi

# --- step 1: check for expected layout ---
if [[ ! -d ros2_ws/src ]]; then
  warn "Expected ros2_ws/src to exist. Creating it."
  mkdir -p ros2_ws/src
fi

# --- step 2: source ROS if present (non-fatal if missing) ---
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # Temporarily disable nounset; some ROS setup scripts reference unset vars when AMENT tracing is on
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  log "Sourced /opt/ros/${ROS_DISTRO}/setup.bash"
else
  warn "/opt/ros/${ROS_DISTRO}/setup.bash not found. Ensure ROS ${ROS_DISTRO} is installed and sourced."
fi

# --- step 3: setup colcon mixins (optional) ---
if $DO_MIXINS; then
  if command -v colcon >/dev/null 2>&1; then
    if ! colcon mixin list 2>/dev/null | grep -q '^default\b'; then
      log "Adding default colcon mixin repository..."
      run "colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml"
    fi
    log "Updating colcon mixins..."
    run "colcon mixin update"
  else
    warn "colcon not found. Install colcon to continue (e.g., sudo apt install python3-colcon-common-extensions)."
  fi
fi

# --- step 4: direnv allow (optional) ---
if $DO_DIRENV_ALLOW; then
  if command -v direnv >/dev/null 2>&1; then
    if [[ -f .envrc ]]; then
      log "Running: direnv allow (to trust .envrc)"
      run "direnv allow"
    else
      warn ".envrc not found in project root (expected). Skipping direnv allow."
    fi
  else
    warn "direnv not found. Install direnv for per-project environment activation."
  fi
fi

# --- step 5: rosdep (check or install) ---
if command -v rosdep >/dev/null 2>&1; then
  log "Updating rosdep database..."
  run "rosdep update"
  if $DO_INSTALL_DEPS; then
    log "Installing dependencies with rosdep (may prompt for sudo)..."
    run "rosdep install -i --from-paths ros2_ws/src --rosdistro ${ROS_DISTRO} -y"
  else
    log "Checking dependencies with rosdep (no installation). Use --install-deps to install."
    run "rosdep check -i --from-paths ros2_ws/src --rosdistro ${ROS_DISTRO}" || true
  fi
else
  warn "rosdep not found. Install with: sudo apt install python3-rosdep -y && sudo rosdep init && rosdep update"
fi

# --- step 6: build workspace (optional) ---
if $DO_BUILD; then
  if [[ -x "scripts/build.sh" ]]; then
    log "Building workspace via scripts/build.sh (keeps artifacts under ros2_ws/)..."
    run "bash scripts/build.sh --ros-distro ${ROS_DISTRO} --mixin release --auto-clean"
  elif command -v colcon >/dev/null 2>&1; then
    log "scripts/build.sh not found; falling back to colcon build in ros2_ws/..."
    if [[ -d ros2_ws ]]; then
      if (cd ros2_ws && colcon build --mixin release); then
        log "Build succeeded with release mixin."
      else
        warn "Release mixin not available or build failed. Falling back to CMAKE_BUILD_TYPE=Release."
        run "cd ros2_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
      fi
    else
      warn "ros2_ws directory not found; skipping build."
    fi
  else
    warn "colcon not found. Skipping build."
  fi
  log "To use this overlay in current shell: source ros2_ws/install/setup.bash"
fi

# --- step 7: write init stamp ---
mkdir -p "$STATE_DIR"
timestamp=$(date -u +%Y-%m-%dT%H:%M:%SZ || echo "unknown")
cat > "$STAMP_PATH" <<EOF
{
  "initialized": true,
  "ros_distro": "$ROS_DISTRO",
  "project_root": "$PROJECT_ROOT",
  "created_utc": "$timestamp",
  "steps_run": {
    "mixins": $DO_MIXINS,
    "direnv_allow": $DO_DIRENV_ALLOW,
    "build": $DO_BUILD
  }
}
EOF
log "Wrote init stamp to $STAMP_PATH"

# --- final tips ---
log "Done. If using direnv, re-enter the project directory to auto-activate."
log "Try: ros2 launch template_pkg pub_sub.launch.py (after build)."
