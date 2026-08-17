#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import websocket
import hashlib
import base64
import hmac
import json
import os
import time
import re
import threading
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime

import rospy
from ucarmain2026.srv import GetTaskSemantics, GetTaskSemanticsResponse

# 保留当前比赛配置。建议赛后将密钥迁移到环境变量或本地配置文件。
APPID = "070ad4bf"
APISecret = "Mjk0NGYwNTI4YTMxMjg5NmNkZmJjMmE2"
APIKey = "9816cf9d021c8432283dec7f604477e9"
AUDIO_FILE = "/home/ucar/ucar_ws_copy/src/ucarmain2026/wakeup_record/test_record.wav"

# ================= 星火连接/收尾参数 =================
# 只限制 WebSocket 建连阶段；连接建立后会恢复为无 socket 读超时，
# 最终结果由 FINAL_RESPONSE_TIMEOUT_SECONDS 的独立看门狗负责兜底。
CONNECT_TIMEOUT_SECONDS = 3.0
FINAL_RESPONSE_TIMEOUT_SECONDS = 4.0
MAX_CONNECT_ATTEMPTS = 2

# 保留当前实测速度较快的音频发送节奏，不改为官方实时节奏。
AUDIO_FRAME_SIZE = 8000
AUDIO_FRAME_INTERVAL_SECONDS = 0.04

# “品”是三个类别中的公共字，不能用于分类；其余任意一个字都可触发类别。
CATEGORY_CHAR_TO_CODE = {
    "食": "food",
    "日": "daily",
    "用": "daily",
    "电": "electronic",
    "子": "electronic",
    "产": "electronic",
}

# 同一类别的连续有效字只算一次，例如：
#   “日用品” -> “日用” -> daily
#   “电子产品” -> “电子产” -> electronic
# 这样不会把一个完整类别词误当成多个任务类别。
CATEGORY_TOKEN_PATTERN = re.compile(r"[日用]+|[电子产]+|食")


def extract_target_categories(text):
    """按文本出现顺序提取类别代码，“品”不参与判定。"""
    categories = []
    for match in CATEGORY_TOKEN_PATTERN.finditer(text):
        token = match.group(0)
        categories.append(CATEGORY_CHAR_TO_CODE[token[0]])
    return categories


class Ws_Param(object):
    def __init__(self, APPID, APIKey, APISecret, AudioFile):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.AudioFile = AudioFile

    def create_url(self):
        url = 'wss://iat.xf-yun.com/v1'
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))
        signature_origin = (
            f"host: iat.xf-yun.com\n"
            f"date: {date}\n"
            f"GET /v1 HTTP/1.1"
        )
        signature_sha = base64.b64encode(
            hmac.new(
                self.APISecret.encode('utf-8'),
                signature_origin.encode('utf-8'),
                digestmod=hashlib.sha256
            ).digest()
        ).decode('utf-8')
        authorization = base64.b64encode(
            (
                f'api_key="{self.APIKey}", algorithm="hmac-sha256", '
                f'headers="host date request-line", signature="{signature_sha}"'
            ).encode('utf-8')
        ).decode('utf-8')
        return url + '?' + urlencode({
            "authorization": authorization,
            "date": date,
            "host": "iat.xf-yun.com"
        })


class SparkAttemptState(object):
    """保存单次 WebSocket 识别尝试的状态。"""

    def __init__(self, attempt_index):
        self.attempt_index = attempt_index
        self.result = ""
        self.opened = False
        self.final_received = False
        self.error = ""

        self.attempt_start = time.monotonic()
        self.connected_at = None
        self.audio_send_start = None
        self.audio_send_end = None
        self.final_received_at = None

        self.final_event = threading.Event()


def run_spark_attempt(ws_param, attempt_index):
    """
    执行一次星火 WebSocket 识别。

    返回：
        (success, recognized_text)
    """
    state = SparkAttemptState(attempt_index)

    def on_message(ws, message):
        try:
            msg_dict = json.loads(message)
            header = msg_dict.get("header", {})
            code = header.get("code", -1)

            if code != 0:
                state.error = "星火返回错误码 {}".format(code)
                print("[Spark] 错误：{}".format(state.error))
                state.final_event.set()
                ws.close()
                return

            # 保留原有语音文本拼接方式，不改变当前语义分类容错逻辑。
            payload = msg_dict.get("payload", {})
            result_payload = payload.get("result", {})
            encoded_text = result_payload.get("text", "")
            if encoded_text:
                text_json = json.loads(
                    base64.b64decode(encoded_text).decode('utf-8')
                )
                for item in text_json.get("ws", []):
                    for word in item.get("cw", []):
                        state.result += word.get("w", "")

            # 服务端 header.status == 2 表示本轮最终响应已经到达。
            if header.get("status") == 2:
                state.final_received = True
                state.final_received_at = time.monotonic()
                state.final_event.set()

                if state.audio_send_end is not None:
                    final_wait = (
                        state.final_received_at - state.audio_send_end
                    )
                    print(
                        "[Spark] 收到最终识别结果，末包后等待 {:.3f}s".format(
                            max(0.0, final_wait)
                        )
                    )
                else:
                    print("[Spark] 收到最终识别结果")

                ws.close()

        except Exception as exc:
            # 不再静默吞掉异常。单个中间包解析异常先记录，最终响应看门狗仍会兜底。
            print("[Spark] 响应解析异常：{}".format(exc))

    def on_error(ws, error):
        if state.final_received:
            return
        state.error = str(error)
        print("[Spark] WebSocket 报错：{}".format(error))

    def on_close(ws, close_status_code, close_msg):
        # 正常最终响应会主动 close；异常关闭由 run_spark_attempt() 统一判断。
        pass

    def on_open(ws):
        state.opened = True
        state.connected_at = time.monotonic()

        connect_elapsed = state.connected_at - state.attempt_start
        print(
            "[Spark] 第{}次 WebSocket 建连完成，耗时 {:.3f}s".format(
                attempt_index,
                connect_elapsed
            )
        )

        # CONNECT_TIMEOUT_SECONDS 只用于建连。连接已经成功后，取消 socket
        # 读取超时，最终响应超时由下面独立的 final_event 看门狗负责。
        try:
            if ws.sock is not None:
                ws.sock.settimeout(None)
        except Exception as exc:
            print("[Spark] 取消连接后 socket 超时失败：{}".format(exc))

        def send_audio():
            try:
                state.audio_send_start = time.monotonic()
                seq = 1

                with open(ws_param.AudioFile, "rb") as fp:
                    while True:
                        buf = fp.read(AUDIO_FRAME_SIZE)
                        status = 0 if seq == 1 else (2 if not buf else 1)

                        payload_data = {
                            "audio": {
                                "encoding": "raw",
                                "sample_rate": 16000,
                                "channels": 1,
                                "bit_depth": 16,
                                "seq": seq,
                                "status": status,
                                "audio": (
                                    base64.b64encode(buf).decode('utf-8')
                                    if buf else ""
                                )
                            }
                        }

                        data = {
                            "header": {
                                "app_id": APPID,
                                "status": status
                            },
                            "payload": payload_data
                        }

                        if status == 0:
                            data["parameter"] = {
                                "iat": {
                                    "domain": "slm",
                                    "language": "zh_cn",
                                    "accent": "mandarin",
                                    "result": {
                                        "encoding": "utf8",
                                        "compress": "raw",
                                        "format": "json"
                                    }
                                }
                            }

                        ws.send(json.dumps(data))

                        if status == 2:
                            state.audio_send_end = time.monotonic()
                            send_elapsed = (
                                state.audio_send_end - state.audio_send_start
                            )
                            print(
                                "[Spark] 音频发送完成，耗时 {:.3f}s，等待最终响应...".format(
                                    send_elapsed
                                )
                            )
                            break

                        seq += 1
                        time.sleep(AUDIO_FRAME_INTERVAL_SECONDS)

                # 不再固定 sleep(1) 后强制切断。
                # 等待服务端明确返回最终 status=2；超时则主动关闭连接。
                if not state.final_event.wait(FINAL_RESPONSE_TIMEOUT_SECONDS):
                    if not state.final_received:
                        state.error = (
                            "等待最终识别结果超过 {:.1f}s".format(
                                FINAL_RESPONSE_TIMEOUT_SECONDS
                            )
                        )
                        print("[Spark] 错误：{}".format(state.error))
                        ws.close()

            except Exception as exc:
                if not state.final_received:
                    state.error = "音频发送异常：{}".format(exc)
                    print("[Spark] 错误：{}".format(state.error))
                    state.final_event.set()
                    try:
                        ws.close()
                    except Exception:
                        pass

        threading.Thread(
            target=send_audio,
            name="spark_audio_sender",
            daemon=True
        ).start()

    # websocket-client 的默认 socket timeout 用于限制本次建连/握手等待。
    websocket.setdefaulttimeout(CONNECT_TIMEOUT_SECONDS)

    ws = websocket.WebSocketApp(
        ws_param.create_url(),
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )
    ws.on_open = on_open

    print(
        "[Spark] 第{}次建立 WebSocket，建连超时 {:.1f}s...".format(
            attempt_index,
            CONNECT_TIMEOUT_SECONDS
        )
    )

    try:
        ws.run_forever(sslopt={"cert_reqs": 2})
    except Exception as exc:
        if not state.final_received:
            state.error = str(exc)
            print("[Spark] run_forever 异常：{}".format(exc))

    total_elapsed = time.monotonic() - state.attempt_start

    if state.final_received:
        print(
            "[Spark] 第{}次识别总耗时 {:.3f}s".format(
                attempt_index,
                total_elapsed
            )
        )
        return True, state.result

    if not state.error:
        if not state.opened:
            state.error = "WebSocket 未能建立连接"
        else:
            state.error = "连接已关闭，但未收到最终识别结果"

    print(
        "[Spark] 第{}次识别失败，耗时 {:.3f}s：{}".format(
            attempt_index,
            total_elapsed,
            state.error
        )
    )
    return False, ""


# ================= 核心：响应 C++ 的服务回调 =================
def handle_semantic_request(req):
    if not os.path.exists(AUDIO_FILE):
        return GetTaskSemanticsResponse(
            success=False,
            target_real="NONE",
            target_sim="NONE"
        )

    request_start = time.monotonic()
    print("\n收到 C++ 主控呼叫，正在连接星火大模型...")

    websocket.enableTrace(False)
    final_result = ""
    recognition_ok = False

    for attempt_index in range(1, MAX_CONNECT_ATTEMPTS + 1):
        # 每次重试都重新生成鉴权 URL，避免复用上一轮时间戳签名。
        ws_param = Ws_Param(APPID, APIKey, APISecret, AUDIO_FILE)
        recognition_ok, recognized_text = run_spark_attempt(
            ws_param,
            attempt_index
        )

        if recognition_ok:
            final_result = recognized_text
            break

        if attempt_index < MAX_CONNECT_ATTEMPTS:
            print(
                "[Spark] 第{}次失败，立即进行第{}次连接尝试...".format(
                    attempt_index,
                    attempt_index + 1
                )
            )

    request_elapsed = time.monotonic() - request_start

    if not recognition_ok:
        print(
            "[Spark] 两次 WebSocket 尝试均失败，本轮总耗时 {:.3f}s".format(
                request_elapsed
            )
        )
        return GetTaskSemanticsResponse(
            success=False,
            target_real="ERROR",
            target_sim="ERROR"
        )

    print("[语音原文]: {}".format(final_result))

    # 完全保留原来的宽松类别提取规则。
    matches = extract_target_categories(final_result)

    if len(matches) >= 2:
        en_target_1 = matches[0]
        en_target_2 = matches[1]
        print(
            "[解析成功] 实体区: {}, 仿真区: {}".format(
                en_target_1,
                en_target_2
            )
        )
        print(
            "[Spark] C++ 服务请求总耗时 {:.3f}s".format(
                request_elapsed
            )
        )
        return GetTaskSemanticsResponse(
            success=True,
            target_real=en_target_1,
            target_sim=en_target_2
        )

    print("[提取失败] 格式不符合预期")
    print(
        "[Spark] C++ 服务请求总耗时 {:.3f}s".format(
            request_elapsed
        )
    )
    return GetTaskSemanticsResponse(
        success=False,
        target_real="ERROR",
        target_sim="ERROR"
    )


if __name__ == "__main__":
    rospy.init_node('spark_semantic_server_node')
    service = rospy.Service(
        '/get_task_semantics',
        GetTaskSemantics,
        handle_semantic_request
    )
    print("规范化星火大模型服务已启动，等待 C++ 主控呼叫...")
    rospy.spin()
