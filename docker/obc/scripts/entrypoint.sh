#!/usr/bin/env bash
set -euo pipefail

# Always source the ROS 2 environment if available so ros2 CLI works out-of-the-box.
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Optionally source a workspace setup script if provided via env
if [[ -n "${WORKSPACE_SETUP:-}" && -f "${WORKSPACE_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE_SETUP}"
fi

exec "$@"
