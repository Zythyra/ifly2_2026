# -*- coding: utf-8 -*-

import csv
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2


ROOT = Path("/home/ucar/ucar_ws_copy/src/ros_nanodet/ocr/validation/ocr_test_75")
IMAGE_DIR = ROOT / "images"
LABEL_DIR = ROOT / "labels"
CROP_DIR = ROOT / "crops"

MANIFEST_PATH = ROOT / "crop_manifest.csv"
SKIPPED_PATH = ROOT / "crop_skipped.csv"

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

TRUE_TEXTS = {
    "food": "食品加工车间",
    "daily": "日用品加工车间",
    "electronic": "电子产品生产车间",
}

# 在原标注框四周增加少量边缘，防止文字贴边
PADDING_X_RATIO = 0.04
PADDING_Y_RATIO = 0.06


def get_true_text(stem: str) -> str:
    prefix = stem.split("_", 1)[0].lower()
    return TRUE_TEXTS.get(prefix, "")


def read_bbox(xml_path: Path):
    tree = ET.parse(xml_path)
    root = tree.getroot()

    objects = root.findall("object")

    if len(objects) != 1:
        raise ValueError(
            f"标注目标数量为{len(objects)}，当前要求单目标"
        )

    bndbox = objects[0].find("bndbox")

    if bndbox is None:
        raise ValueError("XML中缺少bndbox")

    values = {}

    for name in ("xmin", "ymin", "xmax", "ymax"):
        text = bndbox.findtext(name)

        if text is None:
            raise ValueError(f"XML缺少{name}")

        values[name] = int(round(float(text)))

    return (
        values["xmin"],
        values["ymin"],
        values["xmax"],
        values["ymax"],
    )


def main():
    if not IMAGE_DIR.exists():
        raise FileNotFoundError(f"图片目录不存在：{IMAGE_DIR}")

    if not LABEL_DIR.exists():
        raise FileNotFoundError(f"标签目录不存在：{LABEL_DIR}")

    if CROP_DIR.exists():
        shutil.rmtree(CROP_DIR)

    CROP_DIR.mkdir(parents=True, exist_ok=True)

    image_index = {}

    for image_path in IMAGE_DIR.rglob("*"):
        if (
            image_path.is_file()
            and image_path.suffix.lower() in IMAGE_SUFFIXES
        ):
            if image_path.stem in image_index:
                raise RuntimeError(
                    f"存在重名图片：{image_path.stem}"
                )

            image_index[image_path.stem] = image_path

    rows = []
    skipped = []

    xml_paths = sorted(LABEL_DIR.rglob("*.xml"))

    print("=" * 72)
    print(f"图片数量：{len(image_index)}")
    print(f"XML数量：{len(xml_paths)}")
    print("=" * 72)

    for xml_path in xml_paths:
        stem = xml_path.stem
        image_path = image_index.get(stem)

        if image_path is None:
            skipped.append({
                "stem": stem,
                "reason": "找不到同名图片",
            })
            continue

        true_text = get_true_text(stem)

        if not true_text:
            skipped.append({
                "stem": stem,
                "reason": "无法通过文件名前缀确定真实类别",
            })
            continue

        image = cv2.imread(str(image_path))

        if image is None:
            skipped.append({
                "stem": stem,
                "reason": "OpenCV读取图片失败",
            })
            continue

        image_height, image_width = image.shape[:2]

        try:
            xmin, ymin, xmax, ymax = read_bbox(xml_path)
        except Exception as exc:
            skipped.append({
                "stem": stem,
                "reason": str(exc),
            })
            continue

        # 限制到图像范围
        xmin = max(0, min(xmin, image_width - 1))
        ymin = max(0, min(ymin, image_height - 1))
        xmax = max(1, min(xmax, image_width))
        ymax = max(1, min(ymax, image_height))

        box_width = xmax - xmin
        box_height = ymax - ymin

        if box_width <= 2 or box_height <= 2:
            skipped.append({
                "stem": stem,
                "reason": "标注框宽度或高度无效",
            })
            continue

        pad_x = int(round(box_width * PADDING_X_RATIO))
        pad_y = int(round(box_height * PADDING_Y_RATIO))

        crop_xmin = max(0, xmin - pad_x)
        crop_ymin = max(0, ymin - pad_y)
        crop_xmax = min(image_width, xmax + pad_x)
        crop_ymax = min(image_height, ymax + pad_y)

        crop = image[
            crop_ymin:crop_ymax,
            crop_xmin:crop_xmax,
        ]

        if crop.size == 0:
            skipped.append({
                "stem": stem,
                "reason": "裁剪结果为空",
            })
            continue

        output_path = CROP_DIR / f"{stem}.png"

        if not cv2.imwrite(str(output_path), crop):
            skipped.append({
                "stem": stem,
                "reason": "裁剪图保存失败",
            })
            continue

        rows.append({
            "stem": stem,
            "true_text": true_text,
            "source_image": str(image_path),
            "source_xml": str(xml_path),
            "crop_image": str(output_path),
            "image_width": image_width,
            "image_height": image_height,
            "box_width": box_width,
            "box_height": box_height,
            "box_width_ratio": f"{box_width / image_width:.6f}",
            "crop_width": crop.shape[1],
            "crop_height": crop.shape[0],
        })

        print(
            f"[{len(rows):02d}] {stem} | "
            f"原图={image_width}x{image_height} | "
            f"框={box_width}x{box_height} | "
            f"裁剪={crop.shape[1]}x{crop.shape[0]}"
        )

    with MANIFEST_PATH.open(
        "w",
        encoding="utf-8-sig",
        newline="",
    ) as file:
        fieldnames = [
            "stem",
            "true_text",
            "source_image",
            "source_xml",
            "crop_image",
            "image_width",
            "image_height",
            "box_width",
            "box_height",
            "box_width_ratio",
            "crop_width",
            "crop_height",
        ]

        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    with SKIPPED_PATH.open(
        "w",
        encoding="utf-8-sig",
        newline="",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=["stem", "reason"],
        )
        writer.writeheader()
        writer.writerows(skipped)

    print("=" * 72)
    print(f"成功裁剪：{len(rows)}")
    print(f"跳过：{len(skipped)}")
    print(f"裁剪目录：{CROP_DIR}")
    print(f"清单：{MANIFEST_PATH}")
    print(f"跳过记录：{SKIPPED_PATH}")
    print("=" * 72)

    if len(rows) != 75:
        raise RuntimeError(
            f"成功裁剪数量不是75，而是{len(rows)}。"
            f"请检查{SKIPPED_PATH}"
        )


if __name__ == "__main__":
    main()
