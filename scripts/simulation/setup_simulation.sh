#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VENV_DIR="${ROOT_DIR}/.venv-sim"
REQUIREMENTS="${ROOT_DIR}/requirements-simulation.txt"

if [[ ! -d "${ROOT_DIR}/third_party/carm-mujoco/.git" ]]; then
  bash "${ROOT_DIR}/scripts/bootstrap_dependencies.sh"
fi

if command -v python3.10 >/dev/null 2>&1; then
  SIM_PYTHON="${SIM_PYTHON:-python3.10}"
elif command -v python3.11 >/dev/null 2>&1; then
  SIM_PYTHON="${SIM_PYTHON:-python3.11}"
else
  SIM_PYTHON="${SIM_PYTHON:-python3}"
fi

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  if command -v uv >/dev/null 2>&1; then
    uv venv --python "${SIM_PYTHON}" "${VENV_DIR}"
  else
    "${SIM_PYTHON}" -m venv "${VENV_DIR}"
  fi
fi

if command -v uv >/dev/null 2>&1; then
  uv pip install --python "${VENV_DIR}/bin/python" -r "${REQUIREMENTS}"
else
  "${VENV_DIR}/bin/python" -m pip install -r "${REQUIREMENTS}"
fi

"${VENV_DIR}/bin/python" - <<'PY'
import mujoco
import numpy

assert mujoco.__version__ == "3.2.3", mujoco.__version__
assert numpy.__version__ == "1.24.4", numpy.__version__
print(f"Simulation environment ready: MuJoCo {mujoco.__version__}, NumPy {numpy.__version__}")
PY
