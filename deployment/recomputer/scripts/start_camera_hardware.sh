#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${REBOTARM_RS_HARDWARE_CONFIRM:-}" != "I_UNDERSTAND_RS_WILL_MOVE" ]]; then
  echo "Hardware launch blocked: explicit on-site authorization required." >&2
  exit 2
fi
# The legacy workbench can enable its own controller; stop it before takeover.
if curl --silent --fail --max-time 2 http://127.0.0.1:8012/api/status >/dev/null; then
  if ! curl --silent --fail --max-time 2 http://127.0.0.1:8012/api/status | python3 -c 'import json,sys; assert json.load(sys.stdin).get("backend_kind") == "new-rs-ros"'; then
    echo "Legacy motor controller detected; refusing dual control." >&2
    exit 3
  fi
fi
export REBOTARM_HARDWARE_CONFIG="${ROOT_DIR}/deployment/recomputer/hardware-camera-j1-j6.yaml"
exec "${ROOT_DIR}/scripts/start_rs_hardware.sh"
