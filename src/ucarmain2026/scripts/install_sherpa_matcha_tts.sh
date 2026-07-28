#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
THIRD_PARTY_DIR="${PACKAGE_DIR}/third_party"
SHERPA_DIR="${THIRD_PARTY_DIR}/sherpa_onnx"

# 1.10.45 已包含 Matcha TTS，静态 ARM64 包不依赖小车的旧版
# libstdc++/glibc。固定版本可避免上游更新后文件名或参数突然变化。
SHERPA_VERSION="1.10.45"
RUNTIME_ARCHIVE="sherpa-onnx-v${SHERPA_VERSION}-linux-aarch64-static.tar.bz2"
RUNTIME_URL_PRIMARY="https://huggingface.co/csukuangfj/sherpa-onnx-libs/resolve/main/aarch64/${SHERPA_VERSION}/${RUNTIME_ARCHIVE}"
RUNTIME_URL_FALLBACK="https://github.com/k2-fsa/sherpa-onnx/releases/download/v${SHERPA_VERSION}/${RUNTIME_ARCHIVE}"

MODEL_ARCHIVE="matcha-icefall-zh-baker.tar.bz2"
MODEL_URL="https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/${MODEL_ARCHIVE}"
VOCODER_FILE="vocos-22khz-univ.onnx"
VOCODER_URL="https://github.com/k2-fsa/sherpa-onnx/releases/download/vocoder-models/${VOCODER_FILE}"

ARCH="$(uname -m)"
if [[ "${ARCH}" != "aarch64" && "${ARCH}" != "arm64" ]]; then
    echo "错误：该安装包只适用于 RK3588/AArch64，当前架构为 ${ARCH}" >&2
    exit 1
fi

SUDO=()
if [[ "${EUID}" -ne 0 ]]; then
    if ! command -v sudo >/dev/null 2>&1; then
        echo "错误：缺少 sudo，无法安装必要的下载/解压工具" >&2
        exit 1
    fi
    SUDO=(sudo)
fi

MISSING_TOOLS=()
command -v wget >/dev/null 2>&1 || MISSING_TOOLS+=(wget)
command -v bzip2 >/dev/null 2>&1 || MISSING_TOOLS+=(bzip2)
command -v tar >/dev/null 2>&1 || MISSING_TOOLS+=(tar)

if (( ${#MISSING_TOOLS[@]} > 0 )); then
    echo "安装必要工具：wget、bzip2、tar、ca-certificates..."
    "${SUDO[@]}" apt-get update
    "${SUDO[@]}" apt-get install -y \
        wget \
        bzip2 \
        tar \
        ca-certificates
fi

TEMP_DIR="$(mktemp -d)"
STAGING_DIR="${THIRD_PARTY_DIR}/.sherpa_install_${$}"
BACKUP_DIR="${THIRD_PARTY_DIR}/.sherpa_backup_${$}"
INSTALL_SUCCEEDED=0

cleanup() {
    rm -rf -- "${TEMP_DIR}" "${STAGING_DIR}"

    if [[ "${INSTALL_SUCCEEDED}" -ne 1 && -e "${BACKUP_DIR}" ]]; then
        if [[ ! -e "${SHERPA_DIR}" ]]; then
            mv "${BACKUP_DIR}" "${SHERPA_DIR}"
        fi
    fi

    if [[ "${INSTALL_SUCCEEDED}" -eq 1 ]]; then
        rm -rf -- "${BACKUP_DIR}"
    fi
}
trap cleanup EXIT

mkdir -p \
    "${THIRD_PARTY_DIR}" \
    "${PACKAGE_DIR}/audios" \
    "${TEMP_DIR}/runtime" \
    "${STAGING_DIR}/bin" \
    "${STAGING_DIR}/models"

download_file() {
    local output_file="$1"
    local primary_url="$2"
    local fallback_url="${3:-}"

    if wget \
        --tries=3 \
        --timeout=30 \
        --max-redirect=20 \
        --show-progress \
        -O "${output_file}" \
        "${primary_url}"; then
        return 0
    fi

    if [[ -z "${fallback_url}" ]]; then
        return 1
    fi

    echo "主下载地址失败，尝试官方备用地址..."
    wget \
        --tries=3 \
        --timeout=30 \
        --max-redirect=20 \
        --show-progress \
        -O "${output_file}" \
        "${fallback_url}"
}

echo "下载 Sherpa-ONNX ${SHERPA_VERSION} ARM64 静态程序..."
download_file \
    "${TEMP_DIR}/${RUNTIME_ARCHIVE}" \
    "${RUNTIME_URL_PRIMARY}" \
    "${RUNTIME_URL_FALLBACK}"

echo "下载 Matcha 中文 Baker 模型..."
download_file \
    "${TEMP_DIR}/${MODEL_ARCHIVE}" \
    "${MODEL_URL}"

echo "下载 Vocos 22.05 kHz 声码器..."
download_file \
    "${STAGING_DIR}/models/${VOCODER_FILE}" \
    "${VOCODER_URL}"

echo "检查两个压缩包..."
bzip2 -t "${TEMP_DIR}/${RUNTIME_ARCHIVE}"
bzip2 -t "${TEMP_DIR}/${MODEL_ARCHIVE}"

tar -xjf \
    "${TEMP_DIR}/${RUNTIME_ARCHIVE}" \
    -C "${TEMP_DIR}/runtime"
tar -xjf \
    "${TEMP_DIR}/${MODEL_ARCHIVE}" \
    -C "${STAGING_DIR}/models"

RUNTIME_BIN="$(find \
    "${TEMP_DIR}/runtime" \
    -type f \
    -path '*/bin/sherpa-onnx-offline-tts' \
    -print \
    -quit)"

if [[ -z "${RUNTIME_BIN}" ]]; then
    echo "错误：静态运行包中没有 sherpa-onnx-offline-tts" >&2
    exit 1
fi

cp -p \
    "${RUNTIME_BIN}" \
    "${STAGING_DIR}/bin/sherpa-onnx-offline-tts"
chmod +x "${STAGING_DIR}/bin/sherpa-onnx-offline-tts"

MODEL_DIR="${STAGING_DIR}/models/matcha-icefall-zh-baker"
ACOUSTIC_MODEL="${MODEL_DIR}/model-steps-3.onnx"
LEXICON="${MODEL_DIR}/lexicon.txt"
TOKENS="${MODEL_DIR}/tokens.txt"
DICT_DIR="${MODEL_DIR}/dict"
JIEBA_DICT="${DICT_DIR}/jieba.dict.utf8"
VOCODER="${STAGING_DIR}/models/${VOCODER_FILE}"
STAGING_BIN="${STAGING_DIR}/bin/sherpa-onnx-offline-tts"

check_minimum_size() {
    local file_path="$1"
    local minimum_bytes="$2"
    local description="$3"

    if [[ ! -f "${file_path}" ]]; then
        echo "错误：缺少${description}：${file_path}" >&2
        exit 1
    fi

    local actual_bytes
    actual_bytes="$(wc -c < "${file_path}")"
    if (( actual_bytes < minimum_bytes )); then
        echo "错误：${description}文件过小，下载可能不完整：${file_path}" >&2
        exit 1
    fi
}

check_minimum_size "${STAGING_BIN}" 1000000 "Sherpa-ONNX TTS 程序"
check_minimum_size "${ACOUSTIC_MODEL}" 50000000 "Matcha 声学模型"
check_minimum_size "${VOCODER}" 40000000 "Vocos 声码器"
check_minimum_size "${LEXICON}" 100000 "Matcha 词典"
check_minimum_size "${TOKENS}" 1000 "Matcha tokens"
check_minimum_size "${JIEBA_DICT}" 1000000 "Jieba 中文分词词典"

RULE_FSTS=()
for fst_name in phone.fst date.fst number.fst; do
    if [[ -f "${MODEL_DIR}/${fst_name}" ]]; then
        RULE_FSTS+=("${MODEL_DIR}/${fst_name}")
    fi
done

COMMAND=(
    "${STAGING_BIN}"
    "--matcha-acoustic-model=${ACOUSTIC_MODEL}"
    "--matcha-vocoder=${VOCODER}"
    "--matcha-lexicon=${LEXICON}"
    "--matcha-tokens=${TOKENS}"
    "--matcha-dict-dir=${DICT_DIR}"
    "--num-threads=4"
    "--output-filename=${TEMP_DIR}/matcha_test.wav"
    "--debug=0"
)

if (( ${#RULE_FSTS[@]} > 0 )); then
    IFS=,
    COMMAND+=("--tts-rule-fsts=${RULE_FSTS[*]}")
    unset IFS
fi

COMMAND+=("离线中文语音合成测试成功。")

echo "运行 ARM64 静态程序并合成中文测试 WAV..."
"${COMMAND[@]}"

if [[ ! -s "${TEMP_DIR}/matcha_test.wav" ]]; then
    echo "错误：Matcha 测试没有生成有效 WAV" >&2
    exit 1
fi

if [[ -e "${SHERPA_DIR}" ]]; then
    mv "${SHERPA_DIR}" "${BACKUP_DIR}"
fi
mv "${STAGING_DIR}" "${SHERPA_DIR}"
INSTALL_SUCCEEDED=1

echo "Sherpa-ONNX + Matcha Baker 安装并测试成功。"
echo "程序：${SHERPA_DIR}/bin/sherpa-onnx-offline-tts"
echo "模型：${SHERPA_DIR}/models/matcha-icefall-zh-baker"
echo "现在可执行：python3 ${SCRIPT_DIR}/generate_error_audio.py"
