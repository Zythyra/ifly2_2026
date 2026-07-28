#!/usr/bin/env bash

set -euo pipefail

PATCH_VERSION="sherpa-sentence-clips-v7.1-20260728"
EXPECTED_SERVER_VERSION="${PATCH_VERSION} item-cache-0-to-2-v1"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
RUNTIME_DIR="${PACKAGE_DIR}/third_party/sherpa_onnx_batch_runtime"
SERVER_BIN="${RUNTIME_DIR}/bin/sherpa-onnx-preload-tts"
LIB_DIR="${RUNTIME_DIR}/lib"
SOCKET_PATH="${TTS_SOCKET_PATH:-/tmp/ucarmain2026_tts.sock}"
CPU_SET="${TTS_CPU_SET:-4-7}"

if [[ ! -x "${SERVER_BIN}" ]]; then
    echo "[TTS] 错误：预加载程序不存在或不可执行：${SERVER_BIN}" >&2
    echo "[TTS] 请先运行 scripts/install_sherpa_preload_once.sh" >&2
    exit 1
fi

export LD_LIBRARY_PATH="${LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export TTS_NUM_THREADS="${TTS_NUM_THREADS:-4}"

ACTUAL_SERVER_VERSION="$("${SERVER_BIN}" --version 2>/dev/null || true)"
if [[ "${ACTUAL_SERVER_VERSION}" != "${EXPECTED_SERVER_VERSION}" ]]; then
    echo "[TTS] 错误：已安装的预加载程序仍是旧版或损坏。" >&2
    echo "[TTS] 期望：${EXPECTED_SERVER_VERSION}" >&2
    echo "[TTS] 实际：${ACTUAL_SERVER_VERSION:-无法读取版本}" >&2
    echo "[TTS] 请重新运行 scripts/install_sherpa_preload_once.sh" >&2
    exit 1
fi

echo "[TTS] 随小车启动预加载模型，版本 ${PATCH_VERSION}"

if [[ -n "${CPU_SET}" ]] && command -v taskset >/dev/null 2>&1; then
    exec taskset -c "${CPU_SET}" \
        "${SERVER_BIN}" "${PACKAGE_DIR}" "${SOCKET_PATH}"
fi

exec "${SERVER_BIN}" "${PACKAGE_DIR}" "${SOCKET_PATH}"
