#!/usr/bin/env bash
set -euo pipefail

# The dashboard is read-only and binds only to localhost.  The collector keeps
# running normally if the dashboard is closed in the browser.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/home/ubuntu/miniconda3/envs/lerobot/bin/python}"
CONFIG_PATH="${1:-teleoperator/config/config_d3_right_live_test.yaml}"

cd "$ROOT_DIR"
"$PYTHON_BIN" -m teleoperator.dashboard.server --host 127.0.0.1 --port 8765 &
DASHBOARD_PID=$!
COLLECTOR_PID=""
cleanup() {
  kill "$DASHBOARD_PID" 2>/dev/null || true
  if [[ -n "$COLLECTOR_PID" ]]; then
    kill "$COLLECTOR_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Dashboard: http://127.0.0.1:8765"
"$PYTHON_BIN" teleoperator/main.py --config "$CONFIG_PATH" &
COLLECTOR_PID=$!
wait "$COLLECTOR_PID"
