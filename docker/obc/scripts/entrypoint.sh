#!/usr/bin/env bash
set -euo pipefail

# Optionally source a workspace setup script if provided via env
if [[ -n "${WORKSPACE_SETUP:-}" && -f "${WORKSPACE_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE_SETUP}"
fi

exec "$@"
