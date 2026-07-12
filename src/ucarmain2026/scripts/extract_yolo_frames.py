#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import csv
from pathlib import Path
from datetime import datetime # 【新增】：引入 datetime 模块

# ===================== 路径配置 =====================

# 视频所在文件夹
VIDEO_DIR = Path("/home/ucar/ucar_ws_copy/src/ucarmain2026/videos_for_yolo")

# 抽帧图片保存文件夹
OUTPUT_DIR = Path("/home/ucar/ucar_ws_copy/src/ucarmain2026/video_capture_yolo_photo")

# ===================== 抽帧参数配置 =====================

# 每秒抽 3 张图片
TARGET_FPS = 3

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

    video_fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # 如果视频 FPS 读取失败，默认按 30 FPS 处理
    if video_fps <= 0:
        video_fps = 30

    # 根据原视频 FPS 计算抽帧间隔
    frame_interval = max(1, round(video_fps / target_fps))

    print("=" * 60)
    print(f"正在处理视频：{video_path.name}")
    print(f"原视频 FPS：{video_fps:.2f}")
    print(f"视频总帧数：{total_frames}")
    print(f"抽帧设置：每秒抽 {target_fps} 张")
    print(f"实际抽帧间隔：每 {frame_interval} 帧保存一张")
    print(f"输出图片尺寸：{RESIZE[0]} × {RESIZE[1]}")

    frame_id = 0
    saved_count = 0
    records = []

    video_name = video_path.stem

    while True:
        ret, frame = cap.read()

        if not ret:
            break

        # 按间隔抽帧
        if frame_id % frame_interval == 0:
            # 统一缩放到 640×480
            frame = cv2.resize(frame, RESIZE)

            # 【核心修改】：获取当前精确到毫秒的时间戳
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            
            # 【核心修改】：图片命名：视频名_时间戳.jpg
            image_name = f"{video_name}_{timestamp}.{IMAGE_FORMAT}"
            image_path = output_dir / image_name

            # 保存图片
            success = cv2.imwrite(str(image_path), frame)

            if success:
                timestamp_sec = frame_id / video_fps

                records.append([
                    image_name,
                    video_path.name,
                    frame_id,
                    f"{timestamp_sec:.3f}",
                    RESIZE[0],
                    RESIZE[1]
                ])

                saved_count += 1
            else:
                print(f"[警告] 图片保存失败：{image_path}")

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