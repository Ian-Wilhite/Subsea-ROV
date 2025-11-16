#!/usr/bin/env bash
set -euo pipefail

# Load the ROS environment for CLI tools + nodes.
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Optionally source a workspace overlay.
if [[ -n "${WORKSPACE_SETUP:-}" && -f "${WORKSPACE_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE_SETUP}"
fi

# Forward Xauthority if DISPLAY is forwarded
if [[ -n "${DISPLAY:-}" && -f "/tmp/.X11-unix/X0" ]]; then
  echo "[entrypoint] DISPLAY=${DISPLAY}" >&2
fi

exec "$@"
