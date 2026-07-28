#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""检查两个物品名音频，只让预加载 TTS 合成未命中的 0～2 个名称。"""

import array
import math
import os
import socket
import struct
import sys
import time
import wave


PATCH_VERSION = "sherpa-sentence-clips-v7.1-20260728"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
OUTPUT_DIR = os.environ.get(
    "TTS_OUTPUT_DIR",
    os.path.join(PACKAGE_DIR, "audios")
)
SOCKET_PATH = os.environ.get(
    "TTS_SOCKET_PATH",
    "/tmp/ucarmain2026_tts.sock"
)

REQUEST_MAGIC = 0x54545331  # "TTS1"
RESPONSE_MAGIC = 0x54545352  # "TTSR"
MAX_FIELD_BYTES = 1024 * 1024


class PreloadedTtsError(RuntimeError):
    """预加载 TTS 服务或合成任务失败。"""


def _env_float(name, default_value, minimum, maximum):
    try:
        value = float(os.environ.get(name, str(default_value)))
    except ValueError:
        value = default_value
    return max(minimum, min(maximum, value))


CONNECT_TIMEOUT_SECONDS = _env_float(
    "TTS_PRELOAD_CONNECT_TIMEOUT_SECONDS", 60.0, 1.0, 300.0
)
GENERATE_TIMEOUT_SECONDS = _env_float(
    "TTS_BATCH_TIMEOUT_SECONDS", 600.0, 30.0, 1800.0
)


def _recv_exact(connection, byte_count):
    chunks = []
    remaining = byte_count
    while remaining:
        chunk = connection.recv(remaining)
        if not chunk:
            raise PreloadedTtsError(
                "TTS 预加载进程在返回完整结果前已经退出"
            )
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _encode_field(value, description):
    data = str(value).encode("utf-8")
    if not data:
        raise PreloadedTtsError("{}为空".format(description))
    if len(data) > MAX_FIELD_BYTES:
        raise PreloadedTtsError(
            "{}超过 {} 字节限制".format(description, MAX_FIELD_BYTES)
        )
    return struct.pack("!I", len(data)) + data


def _connect_to_preloaded_server():
    deadline = time.monotonic() + CONNECT_TIMEOUT_SECONDS
    last_error = None

    while time.monotonic() < deadline:
        connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            connection.settimeout(1.0)
            connection.connect(SOCKET_PATH)
            connection.settimeout(GENERATE_TIMEOUT_SECONDS)
            return connection
        except (FileNotFoundError, ConnectionRefusedError, socket.timeout) as error:
            last_error = error
            connection.close()
            time.sleep(0.1)
        except OSError as error:
            last_error = error
            connection.close()
            time.sleep(0.1)

    raise PreloadedTtsError(
        "等待开机预加载 TTS 进程超时（{:.1f} 秒）：{}；"
        "请确认 competition.launch 已包含 launch/tts_preload.launch。"
        .format(CONNECT_TIMEOUT_SECONDS, last_error)
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

        sample_count = float(len(samples))
        mean = sum(samples) / sample_count / 32768.0
        rms = math.sqrt(
            sum(sample * sample for sample in samples) / sample_count
        ) / 32768.0
        clipping_ratio = (
            sum(abs(sample) >= 32700 for sample in samples) / sample_count
        )
        return (
            rms >= 0.001
            and abs(mean) <= 0.03
            and clipping_ratio <= 0.002
        )
    except (OSError, EOFError, wave.Error):
        return False


def _item_audio_path(item_name):
    clean_name = str(item_name).strip()
    if (
        not clean_name
        or clean_name in (".", "..")
        or len(clean_name.encode("utf-8")) > 200
        or "/" in clean_name
        or "\\" in clean_name
        or any(ord(character) < 0x20 or ord(character) == 0x7F
               for character in clean_name)
    ):
        raise PreloadedTtsError(
            "物品名不能安全地用作音频文件名：[{}]".format(clean_name)
        )
    return clean_name, os.path.join(OUTPUT_DIR, clean_name + ".wav")


def synthesize_preloaded(tasks):
    tasks = list(tasks)
    if len(tasks) > 2:
        raise PreloadedTtsError("预加载 TTS 一次最多合成两个物品名")

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    request = bytearray(struct.pack("!II", REQUEST_MAGIC, len(tasks)))
    for index, (text, output_file) in enumerate(tasks, 1):
        clean_text = str(text).strip()
        absolute_output = os.path.abspath(output_file)
        request.extend(_encode_field(clean_text, "第 {} 段文本".format(index)))
        request.extend(
            _encode_field(absolute_output, "第 {} 段输出路径".format(index))
        )

    connection = _connect_to_preloaded_server()
    try:
        connection.sendall(request)
        connection.shutdown(socket.SHUT_WR)

        header = _recv_exact(connection, 12)
        magic, status, message_length = struct.unpack("!III", header)
        if magic != RESPONSE_MAGIC:
            raise PreloadedTtsError("TTS 预加载进程返回了未知协议")
        if message_length > MAX_FIELD_BYTES:
            raise PreloadedTtsError("TTS 返回信息长度异常")

        message = _recv_exact(connection, message_length).decode(
            "utf-8", errors="replace"
        )
        if status != 0:
            raise PreloadedTtsError(message or "TTS 批量合成失败")

        # 服务端先销毁 OfflineTts，再发送结果并关闭连接。继续等待 EOF，
        # 保证本函数返回时预加载进程已经走完退出路径。
        if connection.recv(1):
            raise PreloadedTtsError("TTS 返回结果后出现了额外协议数据")
    except socket.timeout:
        raise PreloadedTtsError(
            "物品名语音合成超过 {:.1f} 秒".format(
                GENERATE_TIMEOUT_SECONDS
            )
        )
    except OSError as error:
        raise PreloadedTtsError("与 TTS 预加载进程通信失败：{}".format(error))
    finally:
        connection.close()

    invalid_files = [
        output_file for _, output_file in tasks
        if not _wav_looks_valid(output_file)
    ]
    if invalid_files:
        raise PreloadedTtsError(
            "以下 WAV 波形异常（静音、直流偏置或严重削波）：\n{}".format(
                "\n".join(invalid_files)
            )
        )


def main():
    if len(sys.argv) != 3:
        print(
            "用法：generate_task_audios.py "
            '"实体物品名" "仿真物品名"',
            file=sys.stderr
        )
        return 2

    try:
        requested_items = [_item_audio_path(value) for value in sys.argv[1:3]]
        missing_tasks = []
        queued_paths = set()

        for item_name, output_file in requested_items:
            if _wav_looks_valid(output_file):
                print(
                    "音频库命中，跳过合成：{} -> {}".format(
                        item_name, output_file
                    )
                )
                continue

            if output_file in queued_paths:
                print("两个区域物品名相同，只合成一次：{}".format(item_name))
                continue

            missing_tasks.append((item_name, output_file))
            queued_paths.add(output_file)
            print(
                "音频库未命中，加入合成任务：{} -> {}".format(
                    item_name, output_file
                )
            )

        print(
            "向开机预加载 TTS 提交 {} 个物品名；"
            "提交 0 个时仅释放模型并退出..."
            .format(len(missing_tasks))
        )
        start_time = time.monotonic()
        synthesize_preloaded(missing_tasks)
        elapsed = time.monotonic() - start_time

        invalid_required_files = [
            output_file for _, output_file in requested_items
            if not _wav_looks_valid(output_file)
        ]
        if invalid_required_files:
            raise PreloadedTtsError(
                "TTS 已退出，但以下物品音频仍无效：\n{}".format(
                    "\n".join(invalid_required_files)
                )
            )

        print(
            "两个物品音频均已就绪；本次实际合成 {} 个，"
            "TTS 模型已释放且后台进程已退出，耗时 {:.3f} 秒"
            .format(len(missing_tasks), elapsed)
        )
        return 0
    except PreloadedTtsError as error:
        print("离线批量语音合成失败：{}".format(error), file=sys.stderr)
        return 1
    except Exception as error:
        print("语音脚本发生未预期错误：{}".format(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
