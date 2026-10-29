#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${REPO_ROOT}/install/setup.bash"

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "Error: ${SETUP_FILE} not found. Build the workspace first." >&2
  exit 1
fi

set +u
source "${SETUP_FILE}"
set -u
exec ros2 service call /lc/set_command_state jetson_lc_interfaces/srv/SetLcCommandState \
  "{destination_floor: 0, hall_call_disable: false, manual_motion: 0, maintenance_request: 0}"
