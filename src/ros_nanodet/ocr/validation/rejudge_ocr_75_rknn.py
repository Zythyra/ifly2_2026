# -*- coding: utf-8 -*-

import csv
from difflib import SequenceMatcher
from pathlib import Path


ROOT = Path(
    "/home/ucar/ucar_ws_copy/src/ros_nanodet/ocr/validation/ocr_test_75/ocr_results_rknn"
)

INPUT_CSV = ROOT / "ocr_results.csv"
OUTPUT_CSV = ROOT / "ocr_results_rejudged.csv"
SUMMARY_PATH = ROOT / "summary_rejudged.txt"

CANDIDATES = [
    "食品加工车间",
    "日用品加工车间",
    "电子产品生产车间",
]

MIN_CONFIDENCE = 0.20
MIN_SIMILARITY = 0.55
MIN_MARGIN = 0.06

GENERIC_TEXTS = {
    "",
    "车间",
    "加工车间",
    "生产车间",
}


def decide(predicted_text: str, confidence: float):
    text = predicted_text.strip().replace(" ", "")

    scores = []

    for candidate in CANDIDATES:
        similarity = SequenceMatcher(
            None,
            text,
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

    if text in GENERIC_TEXTS:
        decision = "unknown"

    elif confidence < MIN_CONFIDENCE:
        decision = "unknown"

    else:
        # 至少两个字符，并且是唯一候选词的开头
        prefix_candidates = [
            candidate
            for candidate in CANDIDATES
            if (
                len(text) >= 2
                and candidate.startswith(text)
            )
        ]

        if (
            len(prefix_candidates) == 1
            and confidence >= 0.80
        ):
            decision = prefix_candidates[0]

        elif best_score < MIN_SIMILARITY:
            decision = "unknown"

        elif margin < MIN_MARGIN:
            decision = "unknown"

        else:
            decision = best_candidate

    return {
        "new_best_candidate": best_candidate,
        "new_best_similarity": best_score,
        "new_second_similarity": second_score,
        "new_similarity_margin": margin,
        "new_decision": decision,
    }


def percent(value, total):
    return value / total * 100 if total else 0.0


def main():
    if not INPUT_CSV.exists():
        raise FileNotFoundError(INPUT_CSV)

    with INPUT_CSV.open(
        "r",
        encoding="utf-8-sig",
        newline="",
    ) as file:
        rows = list(csv.DictReader(file))

    results = []

    for row in rows:
        predicted_text = row["predicted_text"]
        confidence = float(row["confidence"])
        true_text = row["true_text"]

        new_result = decide(
            predicted_text,
            confidence,
        )

        new_decision = new_result["new_decision"]
        new_correct = new_decision == true_text

        output_row = {
            **row,
            **new_result,
            "new_decision_correct": int(new_correct),
        }

        results.append(output_row)

        if row["decision"] != new_decision:
            print(
                f"{row['stem']}："
                f"OCR={predicted_text!r}，"
                f"原决策={row['decision']}，"
                f"新决策={new_decision}"
            )

    with OUTPUT_CSV.open(
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

    exact_correct = sum(
        int(row["exact_correct"])
        for row in results
    )

    final_correct = sum(
        int(row["new_decision_correct"])
        for row in results
    )

    unknown_count = sum(
        row["new_decision"] == "unknown"
        for row in results
    )

    wrong_class_count = sum(
        row["new_decision"] not in (
            row["true_text"],
            "unknown",
        )
        for row in results
    )

    lines = [
        "=" * 72,
        "PP-OCRv4-Rec 75张结果重新判定",
        "=" * 72,
        f"总样本：{total}",
        (
            f"原始文字完全正确："
            f"{exact_correct}/{total} "
            f"({percent(exact_correct, total):.2f}%)"
        ),
        (
            f"新规则最终正确："
            f"{final_correct}/{total} "
            f"({percent(final_correct, total):.2f}%)"
        ),
        (
            f"unknown："
            f"{unknown_count}/{total} "
            f"({percent(unknown_count, total):.2f}%)"
        ),
        f"错误分到其他类别：{wrong_class_count}",
        "",
        "分类别结果：",
    ]

    for candidate in CANDIDATES:
        class_rows = [
            row
            for row in results
            if row["true_text"] == candidate
        ]

        class_total = len(class_rows)

        class_correct = sum(
            int(row["new_decision_correct"])
            for row in class_rows
        )

        class_unknown = sum(
            row["new_decision"] == "unknown"
            for row in class_rows
        )

        lines.append(
            f"{candidate}："
            f"正确={class_correct}/{class_total}，"
            f"unknown={class_unknown}"
        )

    lines.extend([
        "",
        f"重新判定CSV：{OUTPUT_CSV}",
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
