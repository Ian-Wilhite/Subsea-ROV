#!/usr/bin/env bash
set -euo pipefail

# Forward Xauthority if DISPLAY is forwarded
if [[ -n "${DISPLAY:-}" && -f "/tmp/.X11-unix/X0" ]]; then
  echo "[entrypoint] DISPLAY=${DISPLAY}" >&2
fi

exec "$@"
