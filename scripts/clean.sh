#!/usr/bin/env bash
# Safe cleanup utility for this project.
# Removes colcon/CMake artifacts to avoid stale cache issues after duplicating the template.
# Usage:
#   bash scripts/clean.sh [-y] [--dry-run]
# Options:
#   -y         Proceed without interactive confirmation
#   --dry-run  Only show what would be removed

set -Eeuo pipefail
IFS=$'\n\t'

CONFIRM=false
DRY_RUN=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    -y|--yes) CONFIRM=true; shift;;
    --dry-run) DRY_RUN=true; shift;;
    *) echo "[WARN] Unknown arg: $1" >&2; shift;;
  esac
done

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"

TARGETS=(
  "build"
  "install"
  "log"
  "ros2_ws/build"
  "ros2_ws/install"
  "ros2_ws/log"
)

# Filter only existing paths
EXISTING=()
for p in "${TARGETS[@]}"; do
  if [[ -e "$p" ]]; then
    EXISTING+=("$p")
  fi
done

if [[ ${#EXISTING[@]} -eq 0 ]]; then
  echo "[INFO] Nothing to clean."
  exit 0
fi

echo "[INFO] The following paths will be removed:"
for p in "${EXISTING[@]}"; do echo "  - $p"; done

if $DRY_RUN; then
  echo "[INFO] Dry-run complete. No changes made."
  exit 0
fi

if ! $CONFIRM; then
  read -r -p "Proceed? [y/N] " ans
  case "$ans" in
    [yY][eE][sS]|[yY]) ;;
    *) echo "[INFO] Aborted."; exit 0;;
  esac
fi

for p in "${EXISTING[@]}"; do
  echo "[INFO] Removing $p"
  rm -rf "$p"
done

echo "[INFO] Cleanup complete."
