#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# 星火大模型 X2 - 扫码物品分类推理节点

import websocket
import hashlib
import base64
import hmac
import json
import re
import ssl
import threading
import time
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime

import rospy
from ucarmain2026.srv import ItemClassify, ItemClassifyResponse

# 保留当前比赛配置。建议赛后将密钥迁移到环境变量或本地配置文件。
APPID = "49973726"
APISecret = "YTE2MDNjNjdlZjU5NzE4ZDUzZTJmOTVi"
APIKey = "a96d7ffe156859dc325d186a3bb20e17"

CONNECT_TIMEOUT_SECONDS = 3.0
FINAL_RESPONSE_TIMEOUT_SECONDS = 7.0
MAX_CONNECT_ATTEMPTS = 2

CATEGORY_DISPLAY = {
    "food": "食品",
    "daily": "日用品",
    "electronic": "电子产品",
}


class Ws_Param(object):
    def __init__(self, app_id, api_key, api_secret):
        self.APPID = app_id
        self.APIKey = api_key
        self.APISecret = api_secret
        self.host = "spark-api.xf-yun.com"
        self.path = "/x2"
        self.url = "wss://{}{}".format(self.host, self.path)

    def create_url(self):
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))
        signature_origin = (
            "host: {}\n"
            "date: {}\n"
            "GET {} HTTP/1.1".format(self.host, date, self.path)
        )
        signature_sha = base64.b64encode(
            hmac.new(
                self.APISecret.encode("utf-8"),
                signature_origin.encode("utf-8"),
                digestmod=hashlib.sha256,
            ).digest()
        ).decode("utf-8")
        authorization_origin = (
            'api_key="{}", algorithm="hmac-sha256", '
            'headers="host date request-line", signature="{}"'.format(
                self.APIKey,
                signature_sha,
            )
        )
        authorization = base64.b64encode(
            authorization_origin.encode("utf-8")
        ).decode("utf-8")
        return self.url + "?" + urlencode(
            {
                "authorization": authorization,
                "date": date,
                "host": self.host,
            }
        )


class SparkAttemptState(object):
    def __init__(self, attempt_index):
        self.attempt_index = attempt_index
        self.response = ""
        self.opened = False
        self.final_received = False
        self.error = ""
        self.attempt_start = time.monotonic()
        self.connected_at = None
        self.final_received_at = None
        self.final_event = threading.Event()


def build_prompt(req):
    real_category = CATEGORY_DISPLAY.get(req.target_real, req.target_real)
    sim_category = CATEGORY_DISPLAY.get(req.target_sim, req.target_sim)

    return """
你是一个智能仓储分类助手。
现在只有三个候选物品：{item1}、{item2}、{item3}。

请从这三个候选物品中进行分类：
1. 实体区目标类别是“{real_category}（{real_code}）”，选择最符合该类别的一个候选物品。
2. 仿真区目标类别是“{sim_category}（{sim_code}）”，选择最符合该类别的一个候选物品。

必须遵守：
- real_item 和 sim_item 只能原样选择自上述三个候选物品，禁止创造、改写或补充新的物品名。
- 只回复一个 JSON 字符串，不要输出解释、Markdown 或其他文字。

格式：
{{"real_item":"候选物品名","sim_item":"候选物品名"}}
""".format(
        item1=req.item1,
        item2=req.item2,
        item3=req.item3,
        real_category=real_category,
        real_code=req.target_real,
        sim_category=sim_category,
        sim_code=req.target_sim,
    )


def run_spark_attempt(prompt, attempt_index):
    state = SparkAttemptState(attempt_index)
    ws_param = Ws_Param(APPID, APIKey, APISecret)

    def on_message(ws, message):
        try:
            data = json.loads(message)
            header = data.get("header", {})
            code = header.get("code", -1)

            if code != 0:
                state.error = "大模型返回错误码 {}：{}".format(
                    code,
                    header.get("message", "未知错误"),
                )
                print("[Spark分类] 错误：{}".format(state.error))
                state.final_event.set()
                ws.close()
                return

            choices = data.get("payload", {}).get("choices", {})
            status = choices.get("status")
            texts = choices.get("text", [])
            if texts:
                state.response += texts[0].get("content", "")

            if status == 2:
                state.final_received = True
                state.final_received_at = time.monotonic()
                state.final_event.set()
                print("[Spark分类] 收到最终响应")
                ws.close()

        except Exception as exc:
            state.error = "消息解析异常：{}".format(exc)
            print("[Spark分类] {}".format(state.error))
            state.final_event.set()
            try:
                ws.close()
            except Exception:
                pass

    def on_error(ws, error):
        if state.final_received:
            return
        state.error = str(error)
        print("[Spark分类] WebSocket 报错：{}".format(error))

    def on_close(ws, close_status_code, close_msg):
        pass

    def on_open(ws):
        state.opened = True
        state.connected_at = time.monotonic()
        print(
            "[Spark分类] 第{}次 WebSocket 建连完成，耗时 {:.3f}s".format(
                attempt_index,
                state.connected_at - state.attempt_start,
            )
        )

        try:
            if ws.sock is not None:
                ws.sock.settimeout(None)
        except Exception as exc:
            print("[Spark分类] 取消连接后 socket 超时失败：{}".format(exc))

        def send_prompt_and_watchdog():
            try:
                data = {
                    "header": {
                        "app_id": APPID,
                        "uid": "ucar_bot",
                    },
                    "parameter": {
                        "chat": {
                            "domain": "spark-x",
                            "temperature": 0.1,
                            "max_tokens": 128,
                        }
                    },
                    "payload": {
                        "message": {
                            "text": [
                                {
                                    "role": "user",
                                    "content": prompt,
                                }
                            ]
                        }
                    },
                }
                ws.send(json.dumps(data, ensure_ascii=False))
                print("[Spark分类] 分类请求已发送，等待最终响应...")

                if not state.final_event.wait(FINAL_RESPONSE_TIMEOUT_SECONDS):
                    if not state.final_received:
                        state.error = "等待最终响应超过 {:.1f}s".format(
                            FINAL_RESPONSE_TIMEOUT_SECONDS
                        )
                        print("[Spark分类] 错误：{}".format(state.error))
                        try:
                            ws.close()
                        except Exception:
                            pass

            except Exception as exc:
                state.error = "分类请求发送异常：{}".format(exc)
                print("[Spark分类] 错误：{}".format(state.error))
                state.final_event.set()
                try:
                    ws.close()
                except Exception:
                    pass

        threading.Thread(
            target=send_prompt_and_watchdog,
            name="spark_classifier_sender",
            daemon=True,
        ).start()

    websocket.setdefaulttimeout(CONNECT_TIMEOUT_SECONDS)
    ws = websocket.WebSocketApp(
        ws_param.create_url(),
        on_message=on_message,
        on_error=on_error,
        on_close=on_close,
    )
    ws.on_open = on_open

    print(
        "[Spark分类] 第{}次建立 WebSocket，建连超时 {:.1f}s...".format(
            attempt_index,
            CONNECT_TIMEOUT_SECONDS,
        )
    )

    try:
        # 保留原代码的证书策略，避免比赛环境因 CA 配置变化引入新问题。
        ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
    except Exception as exc:
        if not state.final_received:
            state.error = str(exc)
            print("[Spark分类] run_forever 异常：{}".format(exc))

    elapsed = time.monotonic() - state.attempt_start
    if state.final_received:
        print(
            "[Spark分类] 第{}次请求总耗时 {:.3f}s".format(
                attempt_index,
                elapsed,
            )
        )
        return True, state.response

    if not state.error:
        state.error = (
            "WebSocket 未能建立连接"
            if not state.opened
            else "连接关闭但未收到最终响应"
        )

    print(
        "[Spark分类] 第{}次请求失败，耗时 {:.3f}s：{}".format(
            attempt_index,
            elapsed,
            state.error,
        )
    )
    return False, ""


def parse_and_validate_response(raw_response, candidates):
    if not raw_response:
        raise ValueError("未收到大模型有效返回")

    match = re.search(r"\{.*?\}", raw_response, re.DOTALL)
    if not match:
        raise ValueError("大模型返回中没有合法 JSON 对象")

    result_dict = json.loads(match.group())
    real_item = str(result_dict.get("real_item", "")).strip()
    sim_item = str(result_dict.get("sim_item", "")).strip()

    if not real_item or not sim_item:
        raise ValueError("real_item 或 sim_item 为空")

    if real_item not in candidates:
        raise ValueError(
            "实体区结果 [{}] 不属于三个二维码候选 {}".format(
                real_item,
                sorted(candidates),
            )
        )
    if sim_item not in candidates:
        raise ValueError(
            "仿真区结果 [{}] 不属于三个二维码候选 {}".format(
                sim_item,
                sorted(candidates),
            )
        )

    return real_item, sim_item


def handle_classification(req):
    request_start = time.monotonic()
    candidates = {
        req.item1.strip(),
        req.item2.strip(),
        req.item3.strip(),
    }
    candidates.discard("")

    print("\n==============================================")
    print("收到主控分类请求，正在呼叫星火 X2 大模型...")
    print("候选物品: [{}], [{}], [{}]".format(req.item1, req.item2, req.item3))
    print(
        "目标类别: 实体区=[{}]，仿真区=[{}]".format(
            req.target_real,
            req.target_sim,
        )
    )

    if len(candidates) < 3:
        print("[Spark分类] 候选物品不足 3 个或存在空值/重复值，拒绝分类")
        return ItemClassifyResponse(
            success=False,
            real_item="识别异常",
            sim_item="识别异常",
        )

    prompt = build_prompt(req)
    websocket.enableTrace(False)

    final_response = ""
    request_ok = False

    for attempt_index in range(1, MAX_CONNECT_ATTEMPTS + 1):
        request_ok, response = run_spark_attempt(prompt, attempt_index)
        if request_ok:
            final_response = response
            break

        if attempt_index < MAX_CONNECT_ATTEMPTS:
            print(
                "[Spark分类] 第{}次失败，立即进行第{}次连接尝试...".format(
                    attempt_index,
                    attempt_index + 1,
                )
            )

    if not request_ok:
        print("[Spark分类] 两次连接均失败，本次分类返回失败")
        return ItemClassifyResponse(
            success=False,
            real_item="识别异常",
            sim_item="识别异常",
        )

    print("[模型原始返回]: {}".format(final_response))

    try:
        real_item, sim_item = parse_and_validate_response(
            final_response,
            candidates,
        )

        print("[分类成功]")
        print("实体区分配: {}".format(real_item))
        print("仿真区分配: {}".format(sim_item))
        print(
            "[Spark分类] C++ 服务请求总耗时 {:.3f}s".format(
                time.monotonic() - request_start
            )
        )

        return ItemClassifyResponse(
            success=True,
            real_item=real_item,
            sim_item=sim_item,
        )

    except Exception as exc:
        print("[分类解析失败]: {}".format(exc))
        print(
            "[Spark分类] C++ 服务请求总耗时 {:.3f}s".format(
                time.monotonic() - request_start
            )
        )
        return ItemClassifyResponse(
            success=False,
            real_item="识别异常",
            sim_item="识别异常",
        )


if __name__ == "__main__":
    rospy.init_node("spark_classifier_server_node")
    service = rospy.Service(
        "/get_item_classification",
        ItemClassify,
        handle_classification,
    )
    print("星火 X2 物品分类服务端已启动，等待 C++ 主控呼叫...")
    rospy.spin()
