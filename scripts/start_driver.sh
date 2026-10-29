#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${REPO_ROOT}/install/setup.bash"

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "Error: ${SETUP_FILE} not found. Build the workspace first." >&2
  exit 1
fi
if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <serial-device>" >&2
  exit 1
fi

set +u
source "${SETUP_FILE}"
set -u
echo "Starting LC driver with serial device: $1"
exec ros2 run jetson_lc_driver lc_driver_node --ros-args -p "serial_device:=$1"
