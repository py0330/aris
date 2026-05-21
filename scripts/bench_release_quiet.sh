#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
TARGET="${1:-bench_bench_model_solver}"

CONFIG_LOG="/tmp/aris_bench_configure.log"
BUILD_LOG="/tmp/aris_bench_build.log"
RUN_LOG="/tmp/aris_${TARGET}_latest.out"

printf "[1/3] Configure (Release + BUILD_BENCHMARKS=ON)\n"
if ! cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release -DBUILD_BENCHMARKS=ON >"${CONFIG_LOG}" 2>&1; then
  printf "Configure failed. Full log: %s\n" "${CONFIG_LOG}"
  tail -n 80 "${CONFIG_LOG}"
  exit 1
fi

printf "[2/3] Build target: %s\n" "${TARGET}"
if ! cmake --build "${BUILD_DIR}" --target "${TARGET}" -j8 >"${BUILD_LOG}" 2>&1; then
  printf "Build failed. Full log: %s\n" "${BUILD_LOG}"
  grep -n "error:" "${BUILD_LOG}" | head -n 30 || true
  tail -n 80 "${BUILD_LOG}"
  exit 1
fi

printf "[3/3] Run benchmark\n"
if ! "${BUILD_DIR}/bin/${TARGET}" >"${RUN_LOG}" 2>&1; then
  printf "Run failed. Full log: %s\n" "${RUN_LOG}"
  tail -n 80 "${RUN_LOG}"
  exit 1
fi

printf "Run complete. Output log: %s\n" "${RUN_LOG}"
printf "\n===== Benchmark Summary =====\n"
grep -E "^(-----------------|bench |.*computational .* time:)" "${RUN_LOG}" || tail -n 120 "${RUN_LOG}"
