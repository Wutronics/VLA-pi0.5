#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [[ ! -x "${ROOT_DIR}/.venv-sim/bin/python" ]]; then
  bash "${ROOT_DIR}/scripts/simulation/setup_simulation.sh"
fi

cd "${ROOT_DIR}"
exec "${ROOT_DIR}/.venv-sim/bin/python" \
  "${ROOT_DIR}/scripts/simulation/d3_right_arm_demo.py" "$@"
