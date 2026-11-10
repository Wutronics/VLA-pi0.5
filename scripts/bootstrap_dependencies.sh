#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY_DIR="${ROOT_DIR}/third_party"
WITH_OPENPI=false

if [[ "${1:-}" == "--with-openpi" ]]; then
  WITH_OPENPI=true
elif [[ $# -gt 0 ]]; then
  echo "Usage: $0 [--with-openpi]" >&2
  exit 2
fi

mkdir -p "${THIRD_PARTY_DIR}"

retry() {
  local attempt=1
  local max_attempts=3

  until "$@"; do
    if [[ ${attempt} -ge ${max_attempts} ]]; then
      echo "Command failed after ${max_attempts} attempts: $*" >&2
      return 1
    fi
    attempt=$((attempt + 1))
    echo "Retrying (${attempt}/${max_attempts}): $*" >&2
    sleep 2
  done
}

checkout_revision() {
  local name="$1"
  local url="$2"
  local revision="$3"
  shift 3
  local destination="${THIRD_PARTY_DIR}/${name}"

  if [[ -d "${destination}/.git" ]]; then
    local configured_url
    configured_url="$(git -C "${destination}" remote get-url origin)"
    if [[ "${configured_url}" != "${url}" ]]; then
      echo "Refusing to reuse ${destination}: origin is ${configured_url}, expected ${url}." >&2
      exit 1
    fi
  elif [[ -e "${destination}" ]]; then
    echo "Refusing to overwrite non-Git path: ${destination}" >&2
    exit 1
  else
    mkdir -p "${destination}"
    git -C "${destination}" init -q
    git -C "${destination}" remote add origin "${url}"
  fi

  if [[ $# -gt 0 ]]; then
    git -C "${destination}" sparse-checkout init --cone
    git -C "${destination}" sparse-checkout set "$@"
  else
    git -C "${destination}" sparse-checkout disable 2>/dev/null || true
  fi

  echo "Fetching ${name} at ${revision}"
  retry git -C "${destination}" fetch --depth 1 --filter=blob:none origin "${revision}"
  retry git -C "${destination}" checkout --detach --force FETCH_HEAD

  local actual_revision
  actual_revision="$(git -C "${destination}" rev-parse HEAD)"
  if [[ "${actual_revision}" != "${revision}" ]]; then
    echo "Revision mismatch for ${name}: got ${actual_revision}, expected ${revision}." >&2
    exit 1
  fi
}

checkout_revision \
  "carm-lerobot" \
  "https://github.com/cvte-robotics/carm-lerobot.git" \
  "31b1349bc1906765dd522324659a07bd7d281c10" \
  "src"

checkout_revision \
  "carm_demo" \
  "https://github.com/cvte-robotics/carm_demo.git" \
  "5e6e4a5bee16222194392c76a12836ba15cf5136" \
  "python" "carm_ros" "carm_ros2" "cpp_test_demo"

checkout_revision \
  "carm_grasp" \
  "https://github.com/cvte-robotics/carm_grasp.git" \
  "870ee61747b20d70732673cf4ae932a6c5e822e1"

checkout_revision \
  "carm-mujoco" \
  "https://github.com/cvte-robotics/carm-mujoco.git" \
  "0ea54f12ffe17b2f8f65eebbb51e6eab2b6af643"

if [[ "${WITH_OPENPI}" == true ]]; then
  checkout_revision \
    "openpi" \
    "https://github.com/Physical-Intelligence/openpi.git" \
    "215abfb217dbac7d5f1273282331b9b1866c0479" \
    "src" "packages" "scripts"
fi

echo "Dependencies are ready in ${THIRD_PARTY_DIR}."
echo "Do not connect to the robot before completing docs/operations/hardware-safety-checklist.md."
