#!/usr/bin/env bash

set -euo pipefail

PATCH_VERSION="sherpa-sentence-clips-v7.1-20260728"
EXPECTED_SERVER_VERSION="${PATCH_VERSION} item-cache-0-to-2-v1"
EXPECTED_SOURCE_MARKER='#define SERVER_PROTOCOL_ID "item-cache-0-to-2-v1"'
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SOURCE_FILE="${PACKAGE_DIR}/src/sherpa_preload_tts.c"
RUNTIME_DIR="${PACKAGE_DIR}/third_party/sherpa_onnx_batch_runtime"
BIN_DIR="${RUNTIME_DIR}/bin"
LIB_DIR="${RUNTIME_DIR}/lib"
TARGET_BIN="${BIN_DIR}/sherpa-onnx-preload-tts"
MODEL_DIR="${PACKAGE_DIR}/third_party/sherpa_onnx/models/matcha-icefall-zh-baker"
VOCODER="${PACKAGE_DIR}/third_party/sherpa_onnx/models/hifigan_v1.onnx"

ARCH="$(uname -m)"
if [[ "${ARCH}" != "aarch64" && "${ARCH}" != "arm64" ]]; then
    echo "错误：该安装脚本只适用于 RK3588/AArch64，当前架构为 ${ARCH}" >&2
    exit 1
fi

for required_file in \
    "${SOURCE_FILE}" \
    "${LIB_DIR}/libsherpa-onnx-c-api.so" \
    "${LIB_DIR}/libonnxruntime.so" \
    "${MODEL_DIR}/model-steps-3.onnx" \
    "${MODEL_DIR}/lexicon.txt" \
    "${MODEL_DIR}/tokens.txt" \
    "${MODEL_DIR}/dict/jieba.dict.utf8" \
    "${MODEL_DIR}/phone.fst" \
    "${MODEL_DIR}/date.fst" \
    "${MODEL_DIR}/number.fst" \
    "${VOCODER}"; do
    if [[ ! -f "${required_file}" ]]; then
        echo "错误：缺少 v4 运行文件：${required_file}" >&2
        echo "请先确认 v4/v5 的共享运行库与模型已安装成功。" >&2
        exit 1
    fi
done

if ! command -v cc >/dev/null 2>&1; then
    echo "错误：缺少 C 编译器，请先执行：sudo apt-get install build-essential" >&2
    exit 1
fi

TEMP_DIR="$(mktemp -d)"
STAGED_BIN="${TEMP_DIR}/sherpa-onnx-preload-tts"
TEST_SOCKET="${TEMP_DIR}/tts.sock"
TEST_OUTPUT_DIR="${TEMP_DIR}/audios"
SERVER_PID=""
INSTALL_SUCCEEDED=0

cleanup() {
    if [[ -n "${SERVER_PID}" ]] && kill -0 "${SERVER_PID}" 2>/dev/null; then
        kill "${SERVER_PID}" 2>/dev/null || true
        wait "${SERVER_PID}" 2>/dev/null || true
    fi
    rm -rf -- "${TEMP_DIR}"
}
trap cleanup EXIT

echo "正在安装 ${PATCH_VERSION}"
echo "复用现有 Sherpa-ONNX 1.10.45 共享库，不重新下载模型..."

if ! grep -Fqx "${EXPECTED_SOURCE_MARKER}" "${SOURCE_FILE}"; then
    echo "错误：当前 src/sherpa_preload_tts.c 仍是旧版或文件未完整覆盖。" >&2
    echo "期望源码标记：${EXPECTED_SOURCE_MARKER}" >&2
    echo "请重新覆盖修正版中的 src/sherpa_preload_tts.c 后再安装。" >&2
    exit 1
fi

cc \
    -std=c99 \
    -O3 \
    -Wall \
    -Wextra \
    "${SOURCE_FILE}" \
    -L"${LIB_DIR}" \
    -Wl,-rpath,'$ORIGIN/../lib' \
    -Wl,-rpath-link,"${LIB_DIR}" \
    -lsherpa-onnx-c-api \
    -o "${STAGED_BIN}"

chmod +x "${STAGED_BIN}"

ACTUAL_SERVER_VERSION="$(
    LD_LIBRARY_PATH="${LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
        "${STAGED_BIN}" --version
)"
if [[ "${ACTUAL_SERVER_VERSION}" != "${EXPECTED_SERVER_VERSION}" ]]; then
    echo "错误：刚刚编译出的服务端版本不正确。" >&2
    echo "期望：${EXPECTED_SERVER_VERSION}" >&2
    echo "实际：${ACTUAL_SERVER_VERSION}" >&2
    exit 1
fi
echo "已确认新服务端：${ACTUAL_SERVER_VERSION}"

if LD_LIBRARY_PATH="${LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
    ldd "${STAGED_BIN}" | grep -q "not found"; then
    echo "错误：预加载程序仍有未找到的共享库：" >&2
    LD_LIBRARY_PATH="${LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
        ldd "${STAGED_BIN}" >&2
    exit 1
fi

mkdir -p "${TEST_OUTPUT_DIR}"

echo "执行真实的“预加载—等待—两个物品名生成—释放退出”自检..."
LD_LIBRARY_PATH="${LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
TTS_NUM_THREADS="${TTS_NUM_THREADS:-4}" \
"${STAGED_BIN}" "${PACKAGE_DIR}" "${TEST_SOCKET}" &
SERVER_PID="$!"

TTS_SOCKET_PATH="${TEST_SOCKET}" \
TTS_OUTPUT_DIR="${TEST_OUTPUT_DIR}" \
TTS_PRELOAD_CONNECT_TIMEOUT_SECONDS=60 \
python3 "${SCRIPT_DIR}/generate_task_audios.py" \
    "实体测试物品" \
    "仿真测试物品"

wait "${SERVER_PID}"
SERVER_PID=""

for output_file in \
    "${TEST_OUTPUT_DIR}/实体测试物品.wav" \
    "${TEST_OUTPUT_DIR}/仿真测试物品.wav"; do
    if [[ ! -s "${output_file}" ]]; then
        echo "错误：自检没有生成有效 WAV：${output_file}" >&2
        exit 1
    fi
done

mkdir -p "${BIN_DIR}"

mapfile -t OLD_SERVER_PIDS < <(
    ps -eo pid=,args= |
        awk -v executable="${TARGET_BIN}" '$2 == executable {print $1}'
)
if (( ${#OLD_SERVER_PIDS[@]} > 0 )); then
    echo "停止仍在运行的旧预加载服务：${OLD_SERVER_PIDS[*]}"
    kill -TERM "${OLD_SERVER_PIDS[@]}" 2>/dev/null || true
    for _ in {1..30}; do
        RUNNING_COUNT=0
        for old_pid in "${OLD_SERVER_PIDS[@]}"; do
            if kill -0 "${old_pid}" 2>/dev/null; then
                RUNNING_COUNT=$((RUNNING_COUNT + 1))
            fi
        done
        if (( RUNNING_COUNT == 0 )); then
            break
        fi
        sleep 0.1
    done
    for old_pid in "${OLD_SERVER_PIDS[@]}"; do
        if kill -0 "${old_pid}" 2>/dev/null; then
            echo "错误：旧预加载服务 PID ${old_pid} 未能退出，请先关闭对应 launch。" >&2
            exit 1
        fi
    done
fi

rm -f -- /tmp/ucarmain2026_tts.sock
install -m 0755 "${STAGED_BIN}" "${TARGET_BIN}"
chmod +x \
    "${SCRIPT_DIR}/start_tts_preload.sh" \
    "${SCRIPT_DIR}/generate_task_audios.py"
INSTALL_SUCCEEDED=1

echo "安装成功：${TARGET_BIN}"
echo "已安装服务端版本：${ACTUAL_SERVER_VERSION}"
echo "请在 competition.launch 内加入："
echo '  <include file="$(find ucarmain2026)/launch/tts_preload.launch" />'
echo "无需重新编译 ROS。"
