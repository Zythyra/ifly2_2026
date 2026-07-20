# -*- coding: utf-8 -*-

from pathlib import Path

import cv2
import numpy as np


INPUT_ROOT = Path("/home/ucar/ucar_ws_copy/src/ros_nanodet/ocr/validation/crop")
OUTPUT_ROOT = Path("/home/ucar/ucar_ws_copy/src/ros_nanodet/ocr/validation/textline")
OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

# 原坐标是按照2048×1536预览图确定的
REFERENCE_WIDTH = 2048
REFERENCE_HEIGHT = 1536

# 点的顺序：左上、右上、右下、左下
TASKS = {
    "daily": {
        "input": (
            INPUT_ROOT
            / "daily"
            / "f65cee42d2bc49bb7538d5165611576c.jpg"
        ),
        "points": [
            [235, 430],
            [1840, 260],
            [1840, 645],
            [230, 825],
        ],
    },
    "electronic": {
        "input": (
            INPUT_ROOT
            / "electronic"
            / "9134fc456a2154d582cea3409a465c80.jpg"
        ),
        "points": [
            [450, 390],
            [1660, 500],
            [1645, 760],
            [440, 650],
        ],
    },
    "food": {
        "input": (
            INPUT_ROOT
            / "food"
            / "eb3ca00f6b7b0d8c020eb2a2a6c8b504.jpg"
        ),
        "points": [
            [895, 365],
            [1450, 300],
            [1460, 505],
            [875, 580],
        ],
    },
}


def point_distance(point_a, point_b):
    return float(np.linalg.norm(point_a - point_b))


for class_name, task in TASKS.items():
    input_path = task["input"]

    if not input_path.exists():
        print(f"[找不到图片] {input_path}")
        continue

    image = cv2.imread(str(input_path))

    if image is None:
        print(f"[读取失败] {input_path}")
        continue

    image_height, image_width = image.shape[:2]

    # 根据真实分辨率自动缩放预设坐标
    scale_x = image_width / REFERENCE_WIDTH
    scale_y = image_height / REFERENCE_HEIGHT

    source_points = np.float32(
        [
            [x * scale_x, y * scale_y]
            for x, y in task["points"]
        ]
    )

    top_width = point_distance(
        source_points[0],
        source_points[1],
    )
    bottom_width = point_distance(
        source_points[3],
        source_points[2],
    )
    left_height = point_distance(
        source_points[0],
        source_points[3],
    )
    right_height = point_distance(
        source_points[1],
        source_points[2],
    )

    estimated_width = max(top_width, bottom_width)
    estimated_height = max(left_height, right_height)

    if estimated_height <= 1:
        print(f"[错误] {class_name} 的裁剪区域高度无效")
        continue

    # 保持原始文字区域的宽高比，不强制拉成固定比例
    output_height = 96
    output_width = int(
        round(estimated_width / estimated_height * output_height)
    )

    output_width = max(320, min(output_width, 1200))

    destination_points = np.float32(
        [
            [0, 0],
            [output_width - 1, 0],
            [output_width - 1, output_height - 1],
            [0, output_height - 1],
        ]
    )

    transform_matrix = cv2.getPerspectiveTransform(
        source_points,
        destination_points,
    )

    textline = cv2.warpPerspective(
        image,
        transform_matrix,
        (output_width, output_height),
        flags=cv2.INTER_CUBIC,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(255, 255, 255),
    )

    # 添加少量白边，防止首尾汉字贴边
    textline = cv2.copyMakeBorder(
        textline,
        8,
        8,
        12,
        12,
        borderType=cv2.BORDER_CONSTANT,
        value=(255, 255, 255),
    )

    output_path = OUTPUT_ROOT / f"{class_name}.png"

    if not cv2.imwrite(str(output_path), textline):
        print(f"[保存失败] {output_path}")
        continue

    print("=" * 70)
    print(f"类别：{class_name}")
    print(f"原图尺寸：{image_width}×{image_height}")
    print(f"坐标缩放：x={scale_x:.3f}, y={scale_y:.3f}")
    print(f"输出尺寸：{textline.shape[1]}×{textline.shape[0]}")
    print(f"[保存成功] {output_path}")

print("=" * 70)
print(f"全部完成：{OUTPUT_ROOT}")
