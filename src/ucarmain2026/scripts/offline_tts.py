#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Sherpa-ONNX Matcha 一次加载、批量生成、立即退出的公共模块。"""

import array
import math
import os
import subprocess
import sys
import wave


PATCH_VERSION = "sherpa-batch-once-v4-20260727"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)

DEFAULT_SHERPA_DIR = os.path.join(
    PACKAGE_DIR, "third_party", "sherpa_onnx"
)
DEFAULT_BATCH_RUNTIME_DIR = os.path.join(
    PACKAGE_DIR, "third_party", "sherpa_onnx_batch_runtime"
)

MATCHA_MODEL_DIR = os.environ.get(
    "MATCHA_MODEL_DIR",
    os.path.join(
        DEFAULT_SHERPA_DIR, "models", "matcha-icefall-zh-baker"
    )
)
MATCHA_VOCODER = os.environ.get(
    "MATCHA_VOCODER",
    os.path.join(DEFAULT_SHERPA_DIR, "models", "hifigan_v1.onnx")
)
BATCH_TTS_BIN = os.environ.get(
    "SHERPA_BATCH_TTS_BIN",
    os.path.join(
        DEFAULT_BATCH_RUNTIME_DIR, "bin", "sherpa-onnx-batch-tts"
    )
)
BATCH_TTS_LIB_DIR = os.environ.get(
    "SHERPA_BATCH_TTS_LIB_DIR",
    os.path.join(DEFAULT_BATCH_RUNTIME_DIR, "lib")
)

MATCHA_ACOUSTIC_MODEL = os.path.join(
    MATCHA_MODEL_DIR, "model-steps-3.onnx"
)
MATCHA_LEXICON = os.path.join(MATCHA_MODEL_DIR, "lexicon.txt")
MATCHA_TOKENS = os.path.join(MATCHA_MODEL_DIR, "tokens.txt")
MATCHA_JIEBA_DICT = os.path.join(
    MATCHA_MODEL_DIR, "dict", "jieba.dict.utf8"
)
BATCH_C_API_LIB = os.path.join(
    BATCH_TTS_LIB_DIR, "libsherpa-onnx-c-api.so"
)
BATCH_ONNX_RUNTIME_LIB = os.path.join(
    BATCH_TTS_LIB_DIR, "libonnxruntime.so"
)

try:
    BATCH_TIMEOUT_SECONDS = int(
        os.environ.get("TTS_BATCH_TIMEOUT_SECONDS", "600")
    )
except ValueError:
    BATCH_TIMEOUT_SECONDS = 600

if BATCH_TIMEOUT_SECONDS < 30:
    BATCH_TIMEOUT_SECONDS = 30
elif BATCH_TIMEOUT_SECONDS > 1800:
    BATCH_TIMEOUT_SECONDS = 1800


class OfflineTtsError(RuntimeError):
    """离线语音合成失败。"""


def _required_files():
    return [
        ("一次性批量 TTS 程序", BATCH_TTS_BIN),
        ("Sherpa-ONNX C API 运行库", BATCH_C_API_LIB),
        ("ONNX Runtime 运行库", BATCH_ONNX_RUNTIME_LIB),
        ("Matcha 声学模型", MATCHA_ACOUSTIC_MODEL),
        ("Matcha 词典", MATCHA_LEXICON),
        ("Matcha tokens", MATCHA_TOKENS),
        ("Jieba 中文分词词典", MATCHA_JIEBA_DICT),
        ("HiFi-GAN v1 声码器", MATCHA_VOCODER),
    ]


def check_installation():
    """检查一次性批量运行程序、共享库和模型是否完整。"""
    missing = [
        "{}：{}".format(description, path)
        for description, path in _required_files()
        if not os.path.isfile(path)
    ]
    if missing:
        raise OfflineTtsError(
            "Sherpa-ONNX 批量 TTS 未安装完整：\n{}\n"
            "请运行 scripts/install_sherpa_batch_once.sh".format(
                "\n".join(missing)
            )
        )

    if not os.access(BATCH_TTS_BIN, os.X_OK):
        raise OfflineTtsError(
            "批量 TTS 程序没有执行权限：{}\n"
            "请执行：chmod +x \"{}\"".format(
                BATCH_TTS_BIN, BATCH_TTS_BIN
            )
        )


def _wav_looks_valid(path):
    try:
        if os.path.getsize(path) <= 44:
            return False

        with wave.open(path, "rb") as wav_file:
            if (
                wav_file.getnchannels() != 1
                or wav_file.getsampwidth() != 2
                or wav_file.getframerate() <= 0
                or wav_file.getnframes() <= 0
            ):
                return False
            raw_samples = wav_file.readframes(wav_file.getnframes())

        samples = array.array("h")
        samples.frombytes(raw_samples)
        if sys.byteorder != "little":
            samples.byteswap()
        if not samples:
            return False

        sample_count = len(samples)
        mean = sum(samples) / float(sample_count) / 32768.0
        rms = math.sqrt(
            sum(sample * sample for sample in samples)
            / float(sample_count)
        ) / 32768.0
        clipping_ratio = (
            sum(1 for sample in samples if abs(sample) >= 32700)
            / float(sample_count)
        )

        return (
            rms >= 0.001
            and abs(mean) <= 0.03
            and clipping_ratio <= 0.002
        )
    except (OSError, EOFError, wave.Error):
        return False


def _validate_pair(pair, index):
    if len(pair) != 2:
        raise OfflineTtsError(
            "第 {} 个任务不是“文本、输出文件”二元组".format(index + 1)
        )

    text, output_file = pair
    text = str(text).strip()
    output_file = os.path.abspath(str(output_file))

    if not text:
        raise OfflineTtsError(
            "第 {} 段待合成文本为空".format(index + 1)
        )
    if "\x00" in text:
        raise OfflineTtsError(
            "第 {} 段待合成文本含有非法空字符".format(index + 1)
        )

    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    return text, output_file


def _batch_environment():
    environment = os.environ.copy()
    old_library_path = environment.get("LD_LIBRARY_PATH", "")
    if old_library_path:
        environment["LD_LIBRARY_PATH"] = (
            BATCH_TTS_LIB_DIR + os.pathsep + old_library_path
        )
    else:
        environment["LD_LIBRARY_PATH"] = BATCH_TTS_LIB_DIR
    return environment


def synthesize_many(text_output_pairs):
    """一个子进程加载一次模型，连续生成全部 WAV，随后退出。"""
    check_installation()

    pairs = list(text_output_pairs)
    if not pairs:
        raise OfflineTtsError("没有需要合成的文本")

    tasks = [
        _validate_pair(pair, index)
        for index, pair in enumerate(pairs)
    ]

    # 这里只启动一次真正持有模型的原生子进程。所有文本和输出文件
    # 都在同一条命令中传入，因此模型只创建一次，最后由 C API
    # 显式销毁；subprocess.run 返回时进程已经完全退出。
    command = [BATCH_TTS_BIN, PACKAGE_DIR]
    for text, output_file in tasks:
        command.extend([text, output_file])

    try:
        result = subprocess.run(
            command,
            env=_batch_environment(),
            timeout=BATCH_TIMEOUT_SECONDS,
            check=False
        )
    except subprocess.TimeoutExpired:
        raise OfflineTtsError(
            "批量合成超过 {} 秒，子进程已终止，模型内存已由系统回收".format(
                BATCH_TIMEOUT_SECONDS
            )
        )
    except OSError as error:
        raise OfflineTtsError(
            "无法启动一次性批量 TTS 程序：{}".format(error)
        )

    if result.returncode != 0:
        raise OfflineTtsError(
            "一次性批量 TTS 失败，返回状态 {}；"
            "详细原因见上方 [TTS] 日志".format(
                result.returncode
            )
        )

    invalid_files = [
        output_file
        for _, output_file in tasks
        if not _wav_looks_valid(output_file)
    ]
    if invalid_files:
        raise OfflineTtsError(
            "以下 WAV 波形异常（静音、直流偏置或严重削波）：\n{}".format(
                "\n".join(invalid_files)
            )
        )


def synthesize(text, output_file):
    """生成单段 WAV；同样在完成后立即退出原生进程。"""
    synthesize_many([(text, output_file)])
