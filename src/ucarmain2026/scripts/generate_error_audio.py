#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

from offline_tts import OfflineTtsError, synthesize


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
OUTPUT_FILE = os.path.join(PACKAGE_DIR, "audios", "U盘.wav")
TEXT = "优盘"


def main():
    try:
        print(
            "正在使用 Sherpa-ONNX Matcha 离线合成提示音："
            "[{}]...".format(TEXT)
        )
        synthesize(TEXT, OUTPUT_FILE)
        print("提示音离线合成完毕：{}".format(OUTPUT_FILE))
        return 0
    except OfflineTtsError as error:
        print("离线提示音合成失败：{}".format(error), file=sys.stderr)
        return 1
    except Exception as error:
        print("语音脚本发生未预期错误：{}".format(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())