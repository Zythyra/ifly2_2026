#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
从标准输入接收 16 kHz / 单声道 / S16_LE 裸 PCM，使用 VAD 动态结束录音，
并将结果原子写入 WAV 文件。

默认优先使用 webrtcvad；没有安装时自动退回“动态噪声基线 + RMS 能量 VAD”。
本脚本不打开 ALSA 设备，因此不会与 speech_command_node 抢占麦克风。
"""

import argparse
import audioop
import json
import math
import os
import select
import sys
import time
import wave
from collections import deque


def parse_args():
    parser = argparse.ArgumentParser(description="从 stdin PCM 流进行 VAD 录音")
    parser.add_argument("--output", required=True, help="最终 WAV 文件路径")
    parser.add_argument("--status-file", default="", help="可选的 JSON 状态文件")
    parser.add_argument("--sample-rate", type=int, default=16000)
    parser.add_argument("--channels", type=int, default=1)
    parser.add_argument("--sample-width", type=int, default=2)
    parser.add_argument("--frame-ms", type=int, default=20, choices=(10, 20, 30))
    parser.add_argument("--min-seconds", type=float, default=2.0)
    parser.add_argument("--silence-seconds", type=float, default=1.2)
    parser.add_argument("--max-seconds", type=float, default=9.0)
    parser.add_argument("--tail-seconds", type=float, default=0.30)
    parser.add_argument(
        "--backend",
        choices=("auto", "webrtc", "energy"),
        default="auto",
    )
    parser.add_argument(
        "--webrtc-mode",
        type=int,
        choices=(0, 1, 2, 3),
        default=2,
        help="0 最宽松，3 最激进",
    )
    parser.add_argument(
        "--energy-threshold",
        type=int,
        default=450,
        help="能量 VAD 的最低 RMS 门限",
    )
    parser.add_argument(
        "--noise-ratio",
        type=float,
        default=3.0,
        help="动态门限相对噪声基线的倍率",
    )
    return parser.parse_args()


class VoiceDetector:
    def __init__(self, args):
        self.args = args
        self.backend = "energy"
        self.webrtc = None
        self.noise_rms = 120.0
        self.recent = deque(maxlen=5)

        if args.backend in ("auto", "webrtc"):
            try:
                import webrtcvad  # type: ignore

                self.webrtc = webrtcvad.Vad(args.webrtc_mode)
                self.backend = "webrtc"
            except Exception as exc:
                if args.backend == "webrtc":
                    raise RuntimeError(
                        "指定了 webrtc 后端，但无法导入 webrtcvad：{}".format(exc)
                    )
                print(
                    "[VAD] 未安装 webrtcvad，自动使用动态能量 VAD：{}".format(exc),
                    flush=True,
                )

    def _energy_is_speech(self, frame):
        rms = float(audioop.rms(frame, self.args.sample_width))
        threshold = max(
            float(self.args.energy_threshold),
            self.noise_rms * float(self.args.noise_ratio),
        )
        is_speech = rms >= threshold

        # 只用明显低于当前门限的帧更新噪声基线，避免把人声学成噪声。
        if not is_speech and rms < threshold * 0.75:
            self.noise_rms = 0.97 * self.noise_rms + 0.03 * rms

        return is_speech

    def is_speech(self, frame):
        if self.backend == "webrtc":
            raw = bool(self.webrtc.is_speech(frame, self.args.sample_rate))
        else:
            raw = self._energy_is_speech(frame)

        # 5 帧中至少 2 帧为人声，抑制单帧脉冲噪声，同时保留短音节。
        self.recent.append(raw)
        return len(self.recent) >= 2 and sum(self.recent) >= 2


def read_frame(input_fd, frame_bytes, deadline_monotonic):
    chunks = bytearray()
    while len(chunks) < frame_bytes:
        remaining_seconds = deadline_monotonic - time.monotonic()
        if remaining_seconds <= 0:
            return bytes(chunks), True

        readable, _, _ = select.select(
            [input_fd],
            [],
            [],
            min(0.20, remaining_seconds),
        )
        if not readable:
            continue

        data = os.read(input_fd, frame_bytes - len(chunks))
        if not data:
            break
        chunks.extend(data)
    return bytes(chunks), False


def atomic_write_wav(path, pcm_data, sample_rate, channels, sample_width):
    output_dir = os.path.dirname(os.path.abspath(path))
    os.makedirs(output_dir, exist_ok=True)
    tmp_path = path + ".tmp.{}".format(os.getpid())

    try:
        with wave.open(tmp_path, "wb") as wav_file:
            wav_file.setnchannels(channels)
            wav_file.setsampwidth(sample_width)
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(pcm_data)
        os.replace(tmp_path, path)
    finally:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)


def atomic_write_json(path, payload):
    if not path:
        return

    output_dir = os.path.dirname(os.path.abspath(path))
    os.makedirs(output_dir, exist_ok=True)
    tmp_path = path + ".tmp.{}".format(os.getpid())
    try:
        with open(tmp_path, "w", encoding="utf-8") as status_file:
            json.dump(payload, status_file, ensure_ascii=False, indent=2)
            status_file.write("\n")
        os.replace(tmp_path, path)
    finally:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)


def validate_args(args):
    if args.sample_rate not in (8000, 16000, 32000, 48000):
        raise ValueError("VAD 采样率必须为 8000/16000/32000/48000")
    if args.channels != 1:
        raise ValueError("当前 VAD 只支持单声道 PCM")
    if args.sample_width != 2:
        raise ValueError("当前 VAD 只支持 16-bit PCM")
    if args.min_seconds < 0:
        raise ValueError("min-seconds 不能小于 0")
    if args.silence_seconds <= 0:
        raise ValueError("silence-seconds 必须大于 0")
    if args.max_seconds <= 0:
        raise ValueError("max-seconds 必须大于 0")
    if args.max_seconds < args.min_seconds:
        raise ValueError("max-seconds 不能小于 min-seconds")


def main():
    args = parse_args()
    validate_args(args)
    detector = VoiceDetector(args)

    samples_per_frame = args.sample_rate * args.frame_ms // 1000
    frame_bytes = samples_per_frame * args.channels * args.sample_width
    max_frames = int(math.ceil(args.max_seconds * 1000.0 / args.frame_ms))
    min_frames = int(math.ceil(args.min_seconds * 1000.0 / args.frame_ms))
    silence_frames = int(
        math.ceil(args.silence_seconds * 1000.0 / args.frame_ms)
    )
    tail_frames = int(math.ceil(args.tail_seconds * 1000.0 / args.frame_ms))
    confirm_frames = max(3, int(math.ceil(0.12 * 1000.0 / args.frame_ms)))

    frames = []
    speech_started = False
    speech_frame_count = 0
    consecutive_speech = 0
    last_speech_index = -1
    stop_reason = "输入流结束"
    start_monotonic = time.monotonic()

    print(
        "[VAD] 开始接收 PCM：backend={}，最短={:.1f}s，静音={:.1f}s，最长={:.1f}s".format(
            detector.backend,
            args.min_seconds,
            args.silence_seconds,
            args.max_seconds,
        ),
        flush=True,
    )

    input_fd = sys.stdin.fileno()
    input_deadline = start_monotonic + args.max_seconds
    for frame_index in range(max_frames):
        frame, input_timed_out = read_frame(
            input_fd,
            frame_bytes,
            input_deadline,
        )
        if input_timed_out:
            if len(frames) < min_frames:
                stop_reason = "等待 PCM 数据超时"
            else:
                stop_reason = "达到最长录音时间"
            break

        if not frame:
            stop_reason = "输入流结束"
            break

        # 末尾不足一帧时补零，只影响最后不足 30 ms 的部分。
        if len(frame) < frame_bytes:
            frame += b"\x00" * (frame_bytes - len(frame))

        frames.append(frame)
        speech = detector.is_speech(frame)

        if speech:
            consecutive_speech += 1
            speech_frame_count += 1
            last_speech_index = frame_index
            if consecutive_speech >= confirm_frames:
                speech_started = True
        else:
            consecutive_speech = 0

        recorded_frames = frame_index + 1
        silent_after_speech = (
            speech_started
            and last_speech_index >= 0
            and frame_index - last_speech_index >= silence_frames
        )

        if recorded_frames >= min_frames and silent_after_speech:
            stop_reason = "检测到连续静音"
            break
    else:
        stop_reason = "达到最长录音时间"

    if not frames:
        print("[VAD] 错误：没有收到任何 PCM 数据", file=sys.stderr, flush=True)
        return 3

    if stop_reason == "等待 PCM 数据超时":
        print(
            "[VAD] 错误：PCM 数据不足，{} 秒内只收到 {:.3f} 秒音频".format(
                args.max_seconds,
                len(frames) * args.frame_ms / 1000.0,
            ),
            file=sys.stderr,
            flush=True,
        )
        return 4

    # VAD 正常截停时，只保留最后人声后的 tail-seconds，去掉多余等待静音。
    keep_frames = len(frames)
    if stop_reason == "检测到连续静音" and last_speech_index >= 0:
        keep_frames = min(len(frames), last_speech_index + 1 + tail_frames)
        keep_frames = max(keep_frames, min(min_frames, len(frames)))

    pcm_data = b"".join(frames[:keep_frames])
    atomic_write_wav(
        args.output,
        pcm_data,
        args.sample_rate,
        args.channels,
        args.sample_width,
    )

    duration = (
        len(pcm_data)
        / float(args.sample_rate * args.channels * args.sample_width)
    )
    payload = {
        "success": True,
        "backend": detector.backend,
        "reason": stop_reason,
        "duration_seconds": round(duration, 3),
        "speech_started": speech_started,
        "speech_frame_count": speech_frame_count,
        "output": os.path.abspath(args.output),
        "wall_seconds": round(time.monotonic() - start_monotonic, 3),
    }
    atomic_write_json(args.status_file, payload)

    print(
        "[VAD] 录音完成：原因={}，WAV={:.3f}s，文件={}".format(
            stop_reason,
            duration,
            args.output,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("[VAD] 收到中断信号", file=sys.stderr, flush=True)
        sys.exit(130)
    except Exception as exc:
        print("[VAD] 运行失败：{}".format(exc), file=sys.stderr, flush=True)
        sys.exit(2)