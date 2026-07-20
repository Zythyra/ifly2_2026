# -*- coding: utf-8 -*-

import csv
from pathlib import Path


ROOT = Path(
    "/home/ucar/ucar_ws_copy/src/ros_nanodet/ocr/validation/ocr_test_75"
)

ONNX_CSV = (
    ROOT
    / "ocr_results"
    / "ocr_results_rejudged.csv"
)

RKNN_CSV = (
    ROOT
    / "ocr_results_rknn"
    / "ocr_results_rejudged.csv"
)


def read_csv(path):
    if not path.exists():
        raise FileNotFoundError(path)

    with path.open(
        "r",
        encoding="utf-8-sig",
        newline="",
    ) as file:
        return {
            row["stem"]: row
            for row in csv.DictReader(file)
        }


def main():
    onnx_rows = read_csv(ONNX_CSV)
    rknn_rows = read_csv(RKNN_CSV)

    stems = sorted(
        set(onnx_rows) | set(rknn_rows)
    )

    raw_text_same = 0
    final_decision_same = 0
    differences = []

    for stem in stems:
        onnx_row = onnx_rows.get(stem)
        rknn_row = rknn_rows.get(stem)

        if onnx_row is None or rknn_row is None:
            differences.append({
                "stem": stem,
                "reason": "一侧缺少该样本",
            })
            continue

        onnx_text = onnx_row["predicted_text"]
        rknn_text = rknn_row["predicted_text"]

        onnx_decision = onnx_row["new_decision"]
        rknn_decision = rknn_row["new_decision"]

        text_equal = onnx_text == rknn_text
        decision_equal = (
            onnx_decision == rknn_decision
        )

        raw_text_same += int(text_equal)
        final_decision_same += int(
            decision_equal
        )

        if not text_equal or not decision_equal:
            differences.append({
                "stem": stem,
                "true_text": onnx_row["true_text"],
                "onnx_text": onnx_text,
                "rknn_text": rknn_text,
                "onnx_decision": onnx_decision,
                "rknn_decision": rknn_decision,
            })

    total = len(stems)

    print("=" * 76)
    print("ONNX与RKNN一致性比较")
    print("=" * 76)

    print(
        f"原始OCR文字一致："
        f"{raw_text_same}/{total} "
        f"({raw_text_same / total * 100:.2f}%)"
    )

    print(
        f"最终类别决策一致："
        f"{final_decision_same}/{total} "
        f"({final_decision_same / total * 100:.2f}%)"
    )

    print(
        f"存在差异的样本数量："
        f"{len(differences)}"
    )

    for item in differences:
        print("-" * 76)

        for key, value in item.items():
            print(f"{key}: {value}")

    print("=" * 76)


if __name__ == "__main__":
    main()
