# -*- coding: utf-8 -*-

import ast
import csv
import os
import shutil
import subprocess
import sys
import time
from difflib import SequenceMatcher
from pathlib import Path


ROOT = Path("/home/ucar/ucar_ws_copy/src/ros_nanodet/ocr/validation/ocr_test_75")
MANIFEST_PATH = ROOT / "crop_manifest.csv"

OCR_DIR = Path(
    "/home/ucar/ucar_ws_copy/src/ros_nanodet/ocr/rknn_model_zoo/examples/PPOCR/PPOCR-Rec/python"
)
OCR_SCRIPT = OCR_DIR / "ppocr_rec.py"
OCR_MODEL = "../model/ppocrv4_rec.onnx"
OCR_TEST_IMAGE = OCR_DIR.parent / "model" / "test.png"

RESULT_DIR = ROOT / "ocr_results"
RESULT_CSV = RESULT_DIR / "ocr_results.csv"
SUMMARY_PATH = RESULT_DIR / "summary.txt"

WRONG_DIR = RESULT_DIR / "exact_wrong"
UNKNOWN_DIR = RESULT_DIR / "unknown"
DECISION_WRONG_DIR = RESULT_DIR / "decision_wrong"
LOG_DIR = RESULT_DIR / "logs"

CANDIDATES = [
    "食品加工车间",
    "日用品加工车间",
    "电子产品生产车间",
]

# 当前是第一轮阈值，后面根据75张结果再调整
MIN_CONFIDENCE = 0.20
MIN_SIMILARITY = 0.55
MIN_MARGIN = 0.06

GENERIC_TEXTS = {
    "",
    "车间",
    "加工车间",
    "生产车间",
}


def parse_output(stdout: str):
    for line in reversed(stdout.splitlines()):
        text = line.strip()

        if not text.startswith("["):
            continue

        try:
            result = ast.literal_eval(text)
        except Exception:
            continue

        if not isinstance(result, list) or not result:
            continue

        first = result[0]

        if not isinstance(first, tuple) or len(first) < 2:
            continue

        return str(first[0]), float(first[1])

    return "", 0.0


def candidate_match(predicted_text: str, confidence: float):
    scores = []

    for candidate in CANDIDATES:
        similarity = SequenceMatcher(
            None,
            predicted_text,
            candidate,
        ).ratio()

        scores.append((candidate, similarity))

    scores.sort(
        key=lambda item: item[1],
        reverse=True,
    )

    best_candidate, best_score = scores[0]
    second_score = scores[1][1]
    margin = best_score - second_score

    if predicted_text.strip() in GENERIC_TEXTS:
        decision = "unknown"
    elif confidence < MIN_CONFIDENCE:
        decision = "unknown"
    elif best_score < MIN_SIMILARITY:
        decision = "unknown"
    elif margin < MIN_MARGIN:
        decision = "unknown"
    else:
        decision = best_candidate

    return (
        best_candidate,
        best_score,
        second_score,
        margin,
        decision,
    )


def percent(number, total):
    if total == 0:
        return 0.0
    return number / total * 100.0


def main():
    if not MANIFEST_PATH.exists():
        raise FileNotFoundError(
            f"找不到：{MANIFEST_PATH}"
        )

    if not OCR_SCRIPT.exists():
        raise FileNotFoundError(
            f"找不到：{OCR_SCRIPT}"
        )

    with MANIFEST_PATH.open(
        "r",
        encoding="utf-8-sig",
        newline="",
    ) as file:
        manifest = list(csv.DictReader(file))

    if len(manifest) != 75:
        raise RuntimeError(
            f"裁剪清单不是75条，而是{len(manifest)}条"
        )

    if RESULT_DIR.exists():
        shutil.rmtree(RESULT_DIR)

    WRONG_DIR.mkdir(parents=True, exist_ok=True)
    UNKNOWN_DIR.mkdir(parents=True, exist_ok=True)
    DECISION_WRONG_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    backup_path = OCR_TEST_IMAGE.parent / "test_before_ocr75.png"
    shutil.copy2(OCR_TEST_IMAGE, backup_path)

    environment = os.environ.copy()
    environment["PYTHONIOENCODING"] = "utf-8"

    results = []

    try:
        for index, row in enumerate(manifest, start=1):
            stem = row["stem"]
            true_text = row["true_text"]
            crop_path = Path(row["crop_image"])

            shutil.copy2(crop_path, OCR_TEST_IMAGE)

            start = time.perf_counter()

            process = subprocess.run(
                [
                    sys.executable,
                    OCR_SCRIPT.name,
                    "--model_path",
                    OCR_MODEL,
                ],
                cwd=str(OCR_DIR),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                env=environment,
            )

            elapsed = time.perf_counter() - start

            stdout = process.stdout

            (LOG_DIR / f"{stem}.txt").write_text(
                stdout,
                encoding="utf-8",
            )

            predicted_text, confidence = parse_output(stdout)

            (
                best_candidate,
                best_score,
                second_score,
                margin,
                decision,
            ) = candidate_match(
                predicted_text,
                confidence,
            )

            exact_correct = predicted_text == true_text
            decision_correct = decision == true_text

            if not exact_correct:
                shutil.copy2(
                    crop_path,
                    WRONG_DIR / crop_path.name,
                )

            if decision == "unknown":
                shutil.copy2(
                    crop_path,
                    UNKNOWN_DIR / crop_path.name,
                )
            elif not decision_correct:
                shutil.copy2(
                    crop_path,
                    DECISION_WRONG_DIR / crop_path.name,
                )

            result_row = {
                **row,
                "predicted_text": predicted_text,
                "confidence": f"{confidence:.8f}",
                "exact_correct": int(exact_correct),
                "best_candidate": best_candidate,
                "best_similarity": f"{best_score:.8f}",
                "second_similarity": f"{second_score:.8f}",
                "similarity_margin": f"{margin:.8f}",
                "decision": decision,
                "decision_correct": int(decision_correct),
                "elapsed_seconds": f"{elapsed:.4f}",
                "return_code": process.returncode,
            }

            results.append(result_row)

            print(
                f"[{index:02d}/75] {stem} | "
                f"真实={true_text} | "
                f"OCR={predicted_text!r} | "
                f"置信度={confidence:.4f} | "
                f"决策={decision} | "
                f"{'正确' if decision_correct else '未通过'}"
            )

    finally:
        if backup_path.exists():
            shutil.copy2(backup_path, OCR_TEST_IMAGE)
            backup_path.unlink()

    with RESULT_CSV.open(
        "w",
        encoding="utf-8-sig",
        newline="",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=list(results[0].keys()),
        )
        writer.writeheader()
        writer.writerows(results)

    total = len(results)

    exact_count = sum(
        int(row["exact_correct"])
        for row in results
    )

    decision_correct_count = sum(
        int(row["decision_correct"])
        for row in results
    )

    unknown_count = sum(
        row["decision"] == "unknown"
        for row in results
    )

    empty_count = sum(
        row["predicted_text"] == ""
        for row in results
    )

    lines = [
        "=" * 72,
        "PP-OCRv4-Rec 75张VOC裁剪图测试结果",
        "=" * 72,
        f"总样本：{total}",
        (
            f"原始文字完全正确：{exact_count}/{total} "
            f"({percent(exact_count, total):.2f}%)"
        ),
        (
            f"候选匹配后最终正确："
            f"{decision_correct_count}/{total} "
            f"({percent(decision_correct_count, total):.2f}%)"
        ),
        (
            f"unknown：{unknown_count}/{total} "
            f"({percent(unknown_count, total):.2f}%)"
        ),
        (
            f"空结果：{empty_count}/{total} "
            f"({percent(empty_count, total):.2f}%)"
        ),
        "",
        "分类别统计：",
    ]

    for true_text in CANDIDATES:
        class_rows = [
            row for row in results
            if row["true_text"] == true_text
        ]

        class_total = len(class_rows)

        class_exact = sum(
            int(row["exact_correct"])
            for row in class_rows
        )

        class_decision = sum(
            int(row["decision_correct"])
            for row in class_rows
        )

        class_unknown = sum(
            row["decision"] == "unknown"
            for row in class_rows
        )

        lines.extend([
            (
                f"{true_text}："
                f"原始完全正确={class_exact}/{class_total}，"
                f"最终正确={class_decision}/{class_total}，"
                f"unknown={class_unknown}"
            ),
        ])

    lines.extend([
        "",
        f"详细CSV：{RESULT_CSV}",
        f"原始识别错误图片：{WRONG_DIR}",
        f"unknown图片：{UNKNOWN_DIR}",
        f"候选匹配仍错误图片：{DECISION_WRONG_DIR}",
        "",
        "注意：当前脚本每张图片都会重新加载ONNX模型，",
        "elapsed_seconds只用于确认程序运行，不代表最终实时速度。",
        "=" * 72,
    ])

    summary = "\n".join(lines)

    SUMMARY_PATH.write_text(
        summary,
        encoding="utf-8",
    )

    print()
    print(summary)


if __name__ == "__main__":
    main()
