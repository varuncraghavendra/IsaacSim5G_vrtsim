#!/usr/bin/env bash
# Convenience wrapper: run the dashboard for the Iris quadcopter platform.
# Equivalent to `bash scripts/run_teleop.sh iris_drone`.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec bash "${REPO_ROOT}/scripts/run_teleop.sh" iris_drone "$@"
