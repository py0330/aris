#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <cmake_target> [build_dir]"
  exit 2
fi

TARGET="$1"
BUILD_DIR="${2:-$(cd "$(dirname "$0")/.." && pwd)/build}"
LOG_FILE="/tmp/aris_build_${TARGET}.log"

printf "Building target: %s\n" "${TARGET}"
printf "Build dir: %s\n" "${BUILD_DIR}"

if ! cmake --build "${BUILD_DIR}" --target "${TARGET}" -j8 >"${LOG_FILE}" 2>&1; then
  printf "Build failed. Full log: %s\n" "${LOG_FILE}"
  printf "\nTop error matches:\n"
  grep -n "error:" "${LOG_FILE}" | head -n 30 || true
  printf "\nTail of build log:\n"
  tail -n 80 "${LOG_FILE}"
  exit 1
fi

printf "Build succeeded. Log: %s\n" "${LOG_FILE}"
