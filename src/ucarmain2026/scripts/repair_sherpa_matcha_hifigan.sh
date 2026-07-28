#!/usr/bin/env bash

set -euo pipefail

PATCH_VERSION="hifigan-fix-v2-20260727"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SHERPA_DIR="${PACKAGE_DIR}/third_party/sherpa_onnx"
BIN="${SHERPA_DIR}/bin/sherpa-onnx-offline-tts"
MODEL_DIR="${SHERPA_DIR}/models/matcha-icefall-zh-baker"
VOCODER_NAME="hifigan_v1.onnx"
VOCODER_URL="https://github.com/k2-fsa/sherpa-onnx/releases/download/vocoder-models/${VOCODER_NAME}"
VOCODER_TARGET="${SHERPA_DIR}/models/${VOCODER_NAME}"

ARCH="$(uname -m)"
if [[ "${ARCH}" != "aarch64" && "${ARCH}" != "arm64" ]]; then
    echo "错误：该修复脚本只适用于 RK3588/AArch64，当前架构为 ${ARCH}" >&2
    exit 1
fi

for required_file in \
    "${BIN}" \
    "${MODEL_DIR}/model-steps-3.onnx" \
    "${MODEL_DIR}/lexicon.txt" \
    "${MODEL_DIR}/tokens.txt" \
    "${MODEL_DIR}/dict/jieba.dict.utf8"; do
    if [[ ! -f "${required_file}" ]]; then
        echo "错误：现有 Sherpa-ONNX 安装不完整：${required_file}" >&2
        echo "请改为运行 scripts/install_sherpa_matcha_tts.sh 完整安装。" >&2
        exit 1
    fi
done

if ! command -v wget >/dev/null 2>&1; then
    echo "错误：缺少 wget，请先执行 sudo apt install -y wget ca-certificates" >&2
    exit 1
fi

TEMP_DIR="$(mktemp -d)"
cleanup() {
    rm -rf -- "${TEMP_DIR}"
}
trap cleanup EXIT

echo "正在运行 ${PATCH_VERSION}"
echo "只下载兼容 Sherpa-ONNX 1.10.45 的 HiFi-GAN v1 声码器..."
wget \
    --tries=3 \
    --timeout=30 \
    --max-redirect=20 \
    --show-progress \
    -O "${TEMP_DIR}/${VOCODER_NAME}" \
    "${VOCODER_URL}"

VOCODER_BYTES="$(wc -c < "${TEMP_DIR}/${VOCODER_NAME}")"
if (( VOCODER_BYTES < 10000000 )); then
    echo "错误：HiFi-GAN 文件过小，下载可能不完整。" >&2
    exit 1
fi

RULE_FSTS="${MODEL_DIR}/phone.fst,${MODEL_DIR}/date.fst,${MODEL_DIR}/number.fst"
TEST_WAV="${TEMP_DIR}/hifigan_test.wav"

echo "使用现有 Matcha 模型和新声码器生成测试语音..."
"${BIN}" \
    "--matcha-acoustic-model=${MODEL_DIR}/model-steps-3.onnx" \
    "--matcha-vocoder=${TEMP_DIR}/${VOCODER_NAME}" \
    "--matcha-lexicon=${MODEL_DIR}/lexicon.txt" \
    "--matcha-tokens=${MODEL_DIR}/tokens.txt" \
    "--matcha-dict-dir=${MODEL_DIR}/dict" \
    "--tts-rule-fsts=${RULE_FSTS}" \
    "--num-threads=4" \
    "--output-filename=${TEST_WAV}" \
    "--debug=0" \
    "未能正确识别，请重新唤醒。"

python3 - "${TEST_WAV}" <<'PY'
import array
import math
import sys
import wave

path = sys.argv[1]
with wave.open(path, "rb") as wav_file:
    if (
        wav_file.getnchannels() != 1
        or wav_file.getsampwidth() != 2
        or wav_file.getnframes() <= 0
    ):
        raise SystemExit("错误：测试 WAV 格式异常")
    raw_samples = wav_file.readframes(wav_file.getnframes())

samples = array.array("h")
samples.frombytes(raw_samples)
if sys.byteorder != "little":
    samples.byteswap()
if not samples:
    raise SystemExit("错误：测试 WAV 没有采样数据")

count = float(len(samples))
mean = sum(samples) / count / 32768.0
rms = math.sqrt(sum(x * x for x in samples) / count) / 32768.0
clipping_ratio = sum(abs(x) >= 32700 for x in samples) / count

print(
    "测试波形：RMS={:.4f}，直流偏置={:.4f}，削波比例={:.4%}".format(
        rms, mean, clipping_ratio
    )
)
if rms < 0.001 or abs(mean) > 0.03 or clipping_ratio > 0.002:
    raise SystemExit(
        "错误：HiFi-GAN 测试波形仍异常，未修改正式安装。"
    )
PY

install -m 0644 "${TEMP_DIR}/${VOCODER_NAME}" "${VOCODER_TARGET}"

echo "${PATCH_VERSION} 修复完成：${VOCODER_TARGET}"
echo "旧 vocos-22khz-univ.onnx 可保留，但新脚本不会再调用它。"
echo "现在执行：python3 ${SCRIPT_DIR}/generate_error_audio.py"
