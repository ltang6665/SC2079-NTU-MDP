#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK1_PATH="${SCRIPT_DIR}/task1.py"

run_as_root() {
  if (( EUID == 0 )); then
    "$@"
  else
    sudo "$@"
  fi
}

if [[ ! -f "${TASK1_PATH}" ]]; then
  echo "[startup] Cannot find ${TASK1_PATH}" >&2
  exit 1
fi

echo "[startup] Checking for stale rfcomm processes..."
mapfile -t RFCOMM_PIDS < <(ps -eo pid=,args= | awk '/[r]fcomm/ {print $1}')

if (( ${#RFCOMM_PIDS[@]} > 0 )); then
  echo "[startup] Killing stale rfcomm PID(s): ${RFCOMM_PIDS[*]}"
  run_as_root kill -9 "${RFCOMM_PIDS[@]}" || true
  sleep 1
else
  echo "[startup] No stale rfcomm process found."
fi

echo "[startup] Launching task1.py..."
if (( EUID == 0 )); then
  exec python3 "${TASK1_PATH}" "$@"
else
  exec sudo -E python3 "${TASK1_PATH}" "$@"
fi
