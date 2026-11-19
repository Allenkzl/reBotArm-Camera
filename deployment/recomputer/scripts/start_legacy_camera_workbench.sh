#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/scripts/rs_env.sh"
export ROS_DOMAIN_ID=77
cd "${ROOT}"
exec python3 -m camera_workbench.server --port 8012 \
  --data "${ROOT}/deployment/recomputer/workbench-data" \
  --home "${ROOT}/deployment/recomputer/home.json" \
  --static "${ROOT}/camera_workbench/static" --rs-root "${ROOT}"
