#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import csv
from pathlib import Path
from datetime import datetime

# ===================== 路径配置 =====================

# 视频所在文件夹
VIDEO_DIR = Path("/home/ucar/ucar_ws_copy/src/ucarmain2026/videos_for_yolo")

# 抽帧图片保存文件夹
OUTPUT_DIR = Path("/home/ucar/ucar_ws_copy/src/ucarmain2026/video_capture_yolo_photo")

# ===================== 抽帧参数配置 =====================

# 每秒抽 3 张图片
TARGET_FPS = 3

# rotate_record.py 中 VideoWriter 设定的录像帧率。
# RK3588 上部分 OpenCV/FFmpeg 组合可能把 XVID AVI 的 FPS 错读成 600，
# 当读取结果明显异常时使用该值兜底。
EXPECTED_VIDEO_FPS = 20.0
MIN_REASONABLE_VIDEO_FPS = 1.0
MAX_REASONABLE_VIDEO_FPS = 120.0

# 图片统一尺寸：640 × 480
RESIZE = (640, 480)

# 输出图片格式
IMAGE_FORMAT = "jpg"

# 支持的视频格式
VIDEO_SUFFIXES = [".avi", ".mp4", ".mov", ".mkv"]


def extract_video_frames(video_path, output_dir, target_fps=3):
    """
    从单个视频中抽帧，并保存为 640×480 图片
    """

    cap = cv2.VideoCapture(str(video_path))

    if not cap.isOpened():
        print(f"[错误] 无法打开视频：{video_path}")
        return []

    reported_video_fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # 某些 RK3588 系统会错误解析 XVID AVI 的帧率元数据，例如返回 600 FPS。
    # rotate_record.py 实际按 20 FPS 写入，因此遇到无效或明显异常的结果时按20 FPS处理。
    if not (MIN_REASONABLE_VIDEO_FPS <= reported_video_fps <= MAX_REASONABLE_VIDEO_FPS):
        video_fps = EXPECTED_VIDEO_FPS
        fps_warning = (
            f"[警告] OpenCV读取到异常视频FPS：{reported_video_fps:.2f}，"
            f"已改用录像程序设定值：{video_fps:.2f} FPS"
        )
    else:
        video_fps = reported_video_fps
        fps_warning = None

    if target_fps <= 0:
        cap.release()
        print(f"[错误] TARGET_FPS必须大于0，当前值：{target_fps}")
        return []

    # 按视频时间抽帧。20 FPS视频抽3张/秒时，帧间隔会按7、7、6附近循环，
    # 比固定每7帧保存一张更接近准确的3张/秒。
    sample_period_sec = 1.0 / target_fps
    average_frame_interval = video_fps / target_fps

    print("=" * 60)
    print(f"正在处理视频：{video_path.name}")
    print(f"OpenCV读取的原始FPS：{reported_video_fps:.2f}")
    if fps_warning:
        print(fps_warning)
    print(f"用于抽帧计算的FPS：{video_fps:.2f}")
    print(f"视频总帧数：{total_frames}")
    print(f"抽帧设置：每秒抽 {target_fps} 张")
    print(f"平均抽帧间隔：约每 {average_frame_interval:.2f} 帧保存一张")
    print(f"输出图片尺寸：{RESIZE[0]} × {RESIZE[1]}")

    frame_id = 0
    saved_count = 0
    records = []
    next_sample_time_sec = 0.0

    video_name = video_path.stem

    while True:
        ret, frame = cap.read()

        if not ret:
            break

        frame_time_sec = frame_id / video_fps

        # 按视频时间抽帧，避免整数帧间隔造成累计误差
        if frame_time_sec + 1e-9 >= next_sample_time_sec:
            # 统一缩放到 640×480
            frame = cv2.resize(frame, RESIZE)

            # 加入原视频帧号，避免程序高速抽帧时毫秒时间戳重名、覆盖图片。
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            image_name = (
                f"{video_name}_{timestamp}_frame{frame_id:06d}.{IMAGE_FORMAT}"
            )
            image_path = output_dir / image_name

            # 保存图片
            success = cv2.imwrite(str(image_path), frame)

            if success:
                records.append([
                    image_name,
                    video_path.name,
                    frame_id,
                    f"{frame_time_sec:.3f}",
                    RESIZE[0],
                    RESIZE[1]
                ])

                saved_count += 1
            else:
                print(f"[警告] 图片保存失败：{image_path}")

            next_sample_time_sec += sample_period_sec

        frame_id += 1

    cap.release()

    print(f"完成：{video_path.name}")
    print(f"保存图片数量：{saved_count}")

    return records


def append_records_to_csv(csv_path, records):
    """
    把新抽帧的视频记录追加写入 frame_index.csv。
    """
    csv_exists = csv_path.exists()

    with open(csv_path, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)

        if not csv_exists:
            writer.writerow([
                "image_name",
                "source_video",
                "original_frame_index",
                "timestamp_sec",
                "width",
                "height"
            ])

        writer.writerows(records)


def main():
    """
    批量处理 videos_for_yolo 文件夹下的所有视频。
    由于采用了时间戳命名，直接无视历史记录进行抽帧提取。
    """

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    if not VIDEO_DIR.exists():
        print(f"[错误] 视频文件夹不存在：{VIDEO_DIR}")
        return

    video_files = [
        p for p in VIDEO_DIR.iterdir()
        if p.is_file() and p.suffix.lower() in VIDEO_SUFFIXES
    ]

    if len(video_files) == 0:
        print(f"[错误] 没有在该目录中找到视频文件：{VIDEO_DIR}")
        return

    print("=" * 60)
    print(f"找到 {len(video_files)} 个视频文件")
    print(f"视频目录：{VIDEO_DIR}")
    print(f"图片输出目录：{OUTPUT_DIR}")
    print("模式：时间戳安全命名 (直接处理所有现有视频)")

    csv_path = OUTPUT_DIR / "frame_index.csv"

    total_new_images = 0
    processed_video_count = 0

    for video_path in video_files:
        
        # 【核心修改】：移除了跳过判断，因为视频被同名覆盖后，必须重新处理
        records = extract_video_frames(
            video_path=video_path,
            output_dir=OUTPUT_DIR,
            target_fps=TARGET_FPS
        )

        if len(records) > 0:
            append_records_to_csv(csv_path, records)
            total_new_images += len(records)
            processed_video_count += 1

    print("=" * 60)
    print("本次抽帧任务完成")
    print(f"本次处理视频数量：{processed_video_count}")
    print(f"本次新增图片数量：{total_new_images}")
    print(f"图片保存路径：{OUTPUT_DIR}")
    print(f"索引文件路径：{csv_path}")


if __name__ == "__main__":
    main()