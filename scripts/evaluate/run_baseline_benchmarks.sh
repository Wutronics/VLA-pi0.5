#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON_BIN="${ROOT_DIR}/.venv-sim/bin/python"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  bash "${ROOT_DIR}/scripts/simulation/setup_simulation.sh"
fi

cd "${ROOT_DIR}"
bash scripts/simulation/run_d3_sim.sh --demo all
"${PYTHON_BIN}" -m unittest discover -s tests/unit -v
"${PYTHON_BIN}" scripts/evaluate/d3_hardware_readonly_benchmark.py
"${PYTHON_BIN}" scripts/evaluate/d3_right_joint_step_benchmark.py

echo "PASS: software baseline benchmarks completed; hardware connection was not attempted."
