#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# 星火大模型 X2 - 扫码物品分类推理节点
#
# V3 UDISK FIX 2026-08-20
#
# 在 V2 Reliable 基础上修复“U盘”必定校验失败的问题：
# - 原逻辑禁止 rz/sz 出现任何英文字母，因此正确返回“U盘”也会失败。
# - 现在仅对被选中的原始候选确实是 U盘/USB盘/优盘 时开放例外。
# - U盘相关常见返回统一规范成“U盘”，保证与现有 U盘.wav 文件名一致。
# - 其他物品仍保持“必须含中文、禁止英文字母”的严格校验。
#
# V2 Reliable 2026-08-19
#
# 目标：
# 1. 强制关闭 X2 深度思考，避免简单三选二分类浪费输出预算和时间。
# 2. max_tokens 从 128 提高到 512，避免最终 JSON 被截断。
# 3. 最终响应超时从 7s 放宽到 10s；WebSocket 建连超时仍保持 3s。
# 4. 模型不再复制原始 QR source，改为返回候选编号：
#       {"r":2,"rz":"衣架","s":3,"sz":"猪肉"}
#    Python 根据 r/s 自己映射回原始二维码候选，避免英文大小写/空格改写导致误判。
# 5. 网络失败、超时、JSON 不完整、字段非法、中文 TTS 校验失败，
#    都算当前 attempt 失败，Python 内部立即建立新 WebSocket 重试一次。
# 6. 增加逐帧 content/reasoning 长度及最终 usage 日志，方便现场定位截断问题。
#
# ROS 服务接口保持不变：
#   /get_item_classification
#   request:
#       item1 item2 item3 target_real target_sim
#   response:
#       success real_item sim_item
#
# 因此 race.cpp / race_2.cpp 不需要修改。

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


# ============================================================================
# 比赛配置
# ============================================================================

# 保留当前比赛密钥配置。
# 赛后建议迁移到环境变量或本地私有配置文件。
APPID = "49973726"
APISecret = "YTE2MDNjNjdlZjU5NzE4ZDUzZTJmOTVi"
APIKey = "a96d7ffe156859dc325d186a3bb20e17"

# 建连本来就稳定在几十到几百毫秒，保持 3s 即可。
CONNECT_TIMEOUT_SECONDS = 3.0

# 原来 7s 过于贴近 X2 的正常响应尾部。
# thinking 关闭后通常会明显更快；10s 作为现场网络抖动保护。
FINAL_RESPONSE_TIMEOUT_SECONDS = 10.0

# 单次 ROS service 内最多立即尝试两次。
# race.cpp 外层仍保留失败后的持续兜底重试。
MAX_ATTEMPTS = 2

# 128 -> 512。
# 正常短 JSON 不会因为上限提高而主动生成到 512 token，
# 这里只是避免答案尾部被截断。
MAX_OUTPUT_TOKENS = 1024

CATEGORY_DISPLAY = {
    "food": "食品",
    "daily": "日用品",
    "electronic": "电子产品",
}


# ============================================================================
# WebSocket 鉴权
# ============================================================================

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
            "GET {} HTTP/1.1".format(
                self.host,
                date,
                self.path,
            )
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


# ============================================================================
# 单次 Spark 请求状态
# ============================================================================

class SparkAttemptState(object):
    def __init__(self, attempt_index):
        self.attempt_index = attempt_index

        # 只累计最终回答 content，不累计 reasoning_content。
        self.response = ""

        self.opened = False
        self.final_received = False
        self.error = ""

        self.attempt_start = time.monotonic()
        self.connected_at = None
        self.final_received_at = None

        self.final_event = threading.Event()

        # 调试统计
        self.frame_count = 0
        self.content_chars = 0
        self.reasoning_chars = 0
        self.usage = None


# ============================================================================
# Prompt：短索引协议
# ============================================================================

def build_prompt(req):
    real_category = CATEGORY_DISPLAY.get(
        req.target_real,
        req.target_real,
    )
    sim_category = CATEGORY_DISPLAY.get(
        req.target_sim,
        req.target_sim,
    )

    return """
你只完成一次三选二物品分类。

候选：
1={item1}
2={item2}
3={item3}

实体目标类别：{real_category}（{real_code}）
仿真目标类别：{sim_category}（{sim_code}）

要求：
- r：实体目标对应的候选编号，只能是1/2/3。
- s：仿真目标对应的候选编号，只能是1/2/3。
- r和s不能相同。
- rz：r对应物品的简洁、标准中文播报名。
- sz：s对应物品的简洁、标准中文播报名。
- 若候选本身已经是中文，rz/sz直接使用自然中文物品名。
- 一般情况下，rz/sz必须包含中文，禁止含英文字母。
- 唯一例外：若选中的原始候选是“U盘”“u盘”“USB盘”“usb盘”或“优盘”，
  rz/sz必须统一输出“U盘”；“U盘”中的字母U允许保留。
- 除“U盘”这个例外外，其他rz/sz仍禁止含英文字母。
- rz/sz不能只写“食品”“日用品”“电子产品”这三个类别名。
- 只输出一个JSON对象，不要解释，不要Markdown，不要代码块。

严格格式：
{{"r":2,"rz":"衣架","s":3,"sz":"猪肉"}}
""".format(
        item1=req.item1.strip(),
        item2=req.item2.strip(),
        item3=req.item3.strip(),
        real_category=real_category,
        real_code=req.target_real,
        sim_category=sim_category,
        sim_code=req.target_sim,
    ).strip()


# ============================================================================
# 调试辅助
# ============================================================================

def _extract_usage(data):
    """
    X2 WebSocket 最终帧的 usage 位于 payload.usage.text。
    做宽松兼容，避免服务端字段形态变化影响主流程。
    """
    payload = data.get("payload", {})
    usage = payload.get("usage")

    if not usage:
        return None

    if isinstance(usage, dict):
        text = usage.get("text")
        if isinstance(text, dict):
            return text

        # 兼容可能直接把 token 字段放在 usage 下的情况。
        if any(
            key in usage
            for key in (
                "prompt_tokens",
                "completion_tokens",
                "total_tokens",
                "question_tokens",
            )
        ):
            return usage

    return usage


def _print_usage(attempt_index, usage):
    if usage is None:
        print(
            "[Spark分类] 第{}次最终帧未提供 usage".format(
                attempt_index
            )
        )
        return

    if isinstance(usage, dict):
        print(
            "[Spark usage] attempt={} prompt_tokens={} "
            "completion_tokens={} total_tokens={} question_tokens={}".format(
                attempt_index,
                usage.get("prompt_tokens", "?"),
                usage.get("completion_tokens", "?"),
                usage.get("total_tokens", "?"),
                usage.get("question_tokens", "?"),
            )
        )
    else:
        print(
            "[Spark usage] attempt={} raw={}".format(
                attempt_index,
                usage,
            )
        )


# ============================================================================
# 单次 WebSocket 请求
# ============================================================================

def run_spark_attempt(prompt, attempt_index):
    state = SparkAttemptState(attempt_index)
    ws_param = Ws_Param(
        APPID,
        APIKey,
        APISecret,
    )

    def on_message(ws, message):
        try:
            data = json.loads(message)

            header = data.get("header", {})
            code = header.get("code", -1)

            if code != 0:
                state.error = (
                    "大模型返回错误码 {}：{}".format(
                        code,
                        header.get(
                            "message",
                            "未知错误",
                        ),
                    )
                )
                print(
                    "[Spark分类] 错误：{}".format(
                        state.error
                    )
                )
                state.final_event.set()
                ws.close()
                return

            payload = data.get("payload", {})
            choices = payload.get("choices", {})
            status = choices.get("status")
            seq = choices.get(
                "seq",
                header.get("seq", "?"),
            )

            texts = choices.get("text", [])

            content_piece = ""
            reasoning_piece = ""

            if texts:
                first_text = texts[0] or {}

                content_piece = str(
                    first_text.get(
                        "content",
                        "",
                    )
                    or ""
                )

                reasoning_piece = str(
                    first_text.get(
                        "reasoning_content",
                        "",
                    )
                    or ""
                )

            state.frame_count += 1
            state.content_chars += len(
                content_piece
            )
            state.reasoning_chars += len(
                reasoning_piece
            )

            # reasoning_content 只统计，不参与最终 JSON。
            state.response += content_piece

            print(
                "[Spark帧] attempt={} frame={} seq={} status={} "
                "content={}字符 reasoning={}字符 "
                "累计content={}字符 累计reasoning={}字符".format(
                    attempt_index,
                    state.frame_count,
                    seq,
                    status,
                    len(content_piece),
                    len(reasoning_piece),
                    state.content_chars,
                    state.reasoning_chars,
                )
            )

            usage = _extract_usage(data)
            if usage is not None:
                state.usage = usage

            if status == 2:
                state.final_received = True
                state.final_received_at = (
                    time.monotonic()
                )

                state.final_event.set()

                print(
                    "[Spark分类] 收到最终响应："
                    "frames={} content={}字符 reasoning={}字符".format(
                        state.frame_count,
                        state.content_chars,
                        state.reasoning_chars,
                    )
                )

                _print_usage(
                    attempt_index,
                    state.usage,
                )

                ws.close()

        except Exception as exc:
            state.error = (
                "消息解析异常：{}".format(exc)
            )

            print(
                "[Spark分类] {}".format(
                    state.error
                )
            )

            state.final_event.set()

            try:
                ws.close()
            except Exception:
                pass

    def on_error(ws, error):
        if state.final_received:
            return

        state.error = str(error)

        print(
            "[Spark分类] WebSocket 报错：{}".format(
                error
            )
        )

    def on_close(
        ws,
        close_status_code,
        close_msg,
    ):
        pass

    def on_open(ws):
        state.opened = True
        state.connected_at = (
            time.monotonic()
        )

        print(
            "[Spark分类] 第{}次 WebSocket 建连完成，耗时 {:.3f}s".format(
                attempt_index,
                state.connected_at
                - state.attempt_start,
            )
        )

        # websocket-client 的全局 default timeout 用于建连。
        # 建连后取消 socket 读超时，最终响应由下面独立 watchdog 控制。
        try:
            if ws.sock is not None:
                ws.sock.settimeout(None)
        except Exception as exc:
            print(
                "[Spark分类] 取消连接后 socket 超时失败：{}".format(
                    exc
                )
            )

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
                            "max_tokens": MAX_OUTPUT_TOKENS,

                            # X2 默认 enabled。
                            # 当前任务只是三个候选物品分类，
                            # 强制关闭深度思考，减少首token与最终结果延迟。
                            "thinking": {
                                "type": "disabled"
                            },
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

                ws.send(
                    json.dumps(
                        data,
                        ensure_ascii=False,
                    )
                )

                print(
                    "[Spark分类] 第{}次分类请求已发送："
                    "thinking=disabled，max_tokens={}，"
                    "等待最终响应，超时={:.1f}s...".format(
                        attempt_index,
                        MAX_OUTPUT_TOKENS,
                        FINAL_RESPONSE_TIMEOUT_SECONDS,
                    )
                )

                if not state.final_event.wait(
                    FINAL_RESPONSE_TIMEOUT_SECONDS
                ):
                    if not state.final_received:
                        state.error = (
                            "等待最终响应超过 {:.1f}s".format(
                                FINAL_RESPONSE_TIMEOUT_SECONDS
                            )
                        )

                        print(
                            "[Spark分类] 错误：{}".format(
                                state.error
                            )
                        )

                        try:
                            ws.close()
                        except Exception:
                            pass

            except Exception as exc:
                state.error = (
                    "分类请求发送异常：{}".format(
                        exc
                    )
                )

                print(
                    "[Spark分类] 错误：{}".format(
                        state.error
                    )
                )

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

    # 仅建连阶段使用 3s timeout。
    websocket.setdefaulttimeout(
        CONNECT_TIMEOUT_SECONDS
    )

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
        # 保留当前比赛环境原有证书策略，避免临赛改变 CA 行为。
        ws.run_forever(
            sslopt={
                "cert_reqs": ssl.CERT_NONE
            }
        )
    except Exception as exc:
        if not state.final_received:
            state.error = str(exc)

            print(
                "[Spark分类] run_forever 异常：{}".format(
                    exc
                )
            )

    elapsed = (
        time.monotonic()
        - state.attempt_start
    )

    if state.final_received:
        print(
            "[Spark分类] 第{}次网络请求完成，耗时 {:.3f}s".format(
                attempt_index,
                elapsed,
            )
        )

        return True, state.response

    if not state.error:
        state.error = (
            "WebSocket 未能建立连接"
            if not state.opened
            else
            "连接关闭但未收到最终响应"
        )

    print(
        "[Spark分类] 第{}次网络请求失败，耗时 {:.3f}s：{}".format(
            attempt_index,
            elapsed,
            state.error,
        )
    )

    return False, ""


# ============================================================================
# 中文 TTS 校验
# ============================================================================

def contains_chinese(text):
    return any(
        "\u4e00" <= ch <= "\u9fff"
        for ch in text
    )


def contains_ascii_letter(text):
    return any(
        ("a" <= ch <= "z")
        or ("A" <= ch <= "Z")
        for ch in text
    )


def _compact_item_name(text):
    """
    用于少量已知物品名的稳健比较：
    - 去除普通/全角空格；
    - 去除常见连接符；
    - 英文字母转小写。
    """
    value = str(text).strip()

    value = re.sub(
        r"[\s_\-－—]+",
        "",
        value,
    )

    return value.lower()


def is_udisk_name(text):
    """
    判断是否是 U 盘的常见写法。

    注意：
    这里只用于“U盘”这个比赛物品的特殊兼容，
    并不意味着一般英文名称都允许进入中文TTS。
    """
    compact = _compact_item_name(text)

    return compact in (
        "u盘",
        "usb盘",
        "优盘",
    )


def validate_chinese_tts_item(
    text,
    field_name,
    source_item="",
):
    value = str(text).strip()
    source_value = str(source_item).strip()

    if not value:
        raise ValueError(
            "{} 为空".format(
                field_name
            )
        )

    # ------------------------------------------------------------------
    # V3 U盘专用兼容
    #
    # 只有“根据r/s索引选中的原始二维码候选本身确实是U盘”时，
    # 才允许 rz/sz 中保留英文字母。
    #
    # 模型常见可能输出：
    #   U盘 / u盘 / USB盘 / usb盘 / 优盘
    #
    # 全部统一成：
    #   U盘
    #
    # 这样：
    # 1. 不会再被 contains_ascii_letter() 拒绝；
    # 2. race.cpp 收到的物品名稳定；
    # 3. 可以直接匹配现有“U盘.wav”。
    # ------------------------------------------------------------------
    if is_udisk_name(source_value):
        if is_udisk_name(value):
            return "U盘"

        raise ValueError(
            "{}=[{}] 与已选中的U盘候选[{}]不匹配；"
            "U盘播报名只接受 U盘/u盘/USB盘/usb盘/优盘".format(
                field_name,
                value,
                source_value,
            )
        )

    # 非U盘物品仍保持V2的严格中文校验。
    if not contains_chinese(value):
        raise ValueError(
            "{}=[{}] 不包含中文，不能送入中文 TTS".format(
                field_name,
                value,
            )
        )

    if contains_ascii_letter(value):
        raise ValueError(
            "{}=[{}] 仍含英文字母，拒绝送入中文 TTS；"
            "当前仅U盘允许英文字母例外".format(
                field_name,
                value,
            )
        )

    if value in (
        "食品",
        "日用品",
        "电子产品",
    ):
        raise ValueError(
            "{}=[{}] 是类别名，不是具体物品名".format(
                field_name,
                value,
            )
        )

    return value


# ============================================================================
# JSON 提取 + 索引协议校验
# ============================================================================

def extract_json_object(raw_response):
    """
    X2 理论上只应返回一个 JSON。
    为了兼容偶发前后多一个普通字符，仍允许从文本中提取首个完整 {...}。

    不使用原来的非贪婪 r"{.*?}" 作为唯一依据：
    索引协议没有嵌套对象，但这里使用 JSONDecoder.raw_decode，
    可以明确判断对象是否真的完整，而不是只看有没有右花括号。
    """
    if not raw_response:
        raise ValueError(
            "未收到大模型有效返回"
        )

    raw = raw_response.strip()

    # 最理想情况：整个返回就是 JSON。
    try:
        value = json.loads(raw)
        if not isinstance(value, dict):
            raise ValueError(
                "模型返回JSON不是对象"
            )
        return value
    except json.JSONDecodeError:
        pass

    # 容错：从第一个左花括号开始尝试 raw_decode。
    start = raw.find("{")
    if start < 0:
        raise ValueError(
            "大模型返回中没有JSON对象起始符号"
        )

    decoder = json.JSONDecoder()

    try:
        value, _ = decoder.raw_decode(
            raw[start:]
        )
    except json.JSONDecodeError as exc:
        raise ValueError(
            "大模型JSON不完整或格式非法：{}".format(
                exc
            )
        )

    if not isinstance(value, dict):
        raise ValueError(
            "模型返回JSON不是对象"
        )

    return value


def parse_candidate_index(
    value,
    field_name,
):
    """
    允许模型返回 JSON number 2，
    也兼容偶发字符串 "2"。
    """
    if isinstance(value, bool):
        raise ValueError(
            "{}不能是布尔值".format(
                field_name
            )
        )

    try:
        index = int(value)
    except Exception:
        raise ValueError(
            "{}=[{}] 不是合法候选编号".format(
                field_name,
                value,
            )
        )

    # 防止 2.7 被 int() 静默截成2。
    if isinstance(value, float):
        if abs(value - index) > 1e-9:
            raise ValueError(
                "{}=[{}] 不是整数编号".format(
                    field_name,
                    value,
                )
            )

    if index < 1 or index > 3:
        raise ValueError(
            "{}={} 超出候选编号1~3".format(
                field_name,
                index,
            )
        )

    return index


def parse_and_validate_response(
    raw_response,
    candidate_list,
):
    """
    返回：
      real_source,
      real_item,
      sim_source,
      sim_item
    """
    result_dict = extract_json_object(
        raw_response
    )

    real_index = parse_candidate_index(
        result_dict.get("r"),
        "r",
    )

    sim_index = parse_candidate_index(
        result_dict.get("s"),
        "s",
    )

    if real_index == sim_index:
        raise ValueError(
            "r={} 与 s={} 相同；"
            "实体区和仿真区不能选择同一个候选".format(
                real_index,
                sim_index,
            )
        )

    # 原始 source 完全由 Python 依据索引恢复，
    # 不再信任/要求大模型逐字复制二维码字符串。
    real_source = candidate_list[
        real_index - 1
    ]

    sim_source = candidate_list[
        sim_index - 1
    ]

    # V3：
    # TTS名称校验必须结合“r/s实际选中的原始候选”进行。
    # 这样只有真正选中了U盘候选时，才允许返回“U盘”这个混合名称。
    real_item = validate_chinese_tts_item(
        result_dict.get(
            "rz",
            "",
        ),
        "rz",
        real_source,
    )

    sim_item = validate_chinese_tts_item(
        result_dict.get(
            "sz",
            "",
        ),
        "sz",
        sim_source,
    )

    return (
        real_source,
        real_item,
        sim_source,
        sim_item,
        real_index,
        sim_index,
    )


# ============================================================================
# ROS 服务
# ============================================================================

def handle_classification(req):
    request_start = (
        time.monotonic()
    )

    # 必须保留二维码原始顺序，索引协议依赖 1/2/3。
    candidate_list = [
        req.item1.strip(),
        req.item2.strip(),
        req.item3.strip(),
    ]

    print(
        "\n=============================================="
    )
    print(
        "收到主控分类请求，正在呼叫星火 X2 大模型..."
    )
    print(
        "候选物品: 1=[{}], 2=[{}], 3=[{}]".format(
            candidate_list[0],
            candidate_list[1],
            candidate_list[2],
        )
    )
    print(
        "目标类别: 实体区=[{}]，仿真区=[{}]".format(
            req.target_real,
            req.target_sim,
        )
    )

    # ------------------------------------------------------------
    # 请求输入校验
    # ------------------------------------------------------------
    if any(
        not item
        for item in candidate_list
    ):
        print(
            "[Spark分类] 三个二维码候选中存在空值，拒绝分类"
        )

        return ItemClassifyResponse(
            success=False,
            real_item="识别异常",
            sim_item="识别异常",
        )

    if len(set(candidate_list)) != 3:
        print(
            "[Spark分类] 三个二维码候选存在重复值，拒绝分类"
        )

        return ItemClassifyResponse(
            success=False,
            real_item="识别异常",
            sim_item="识别异常",
        )

    if (
        req.target_real
        not in CATEGORY_DISPLAY
        or req.target_sim
        not in CATEGORY_DISPLAY
    ):
        print(
            "[Spark分类] 目标类别非法：real=[{}] sim=[{}]".format(
                req.target_real,
                req.target_sim,
            )
        )

        return ItemClassifyResponse(
            success=False,
            real_item="识别异常",
            sim_item="识别异常",
        )

    if (
        req.target_real
        == req.target_sim
    ):
        print(
            "[Spark分类] 实体区与仿真区目标类别相同，拒绝分类"
        )

        return ItemClassifyResponse(
            success=False,
            real_item="识别异常",
            sim_item="识别异常",
        )

    prompt = build_prompt(req)

    websocket.enableTrace(False)

    # ------------------------------------------------------------
    # Python 内部最多两次完整 attempt。
    #
    # “完整 attempt”包含：
    #   WebSocket连接
    #   最终响应
    #   JSON解析
    #   索引校验
    #   中文TTS校验
    #
    # 任何一环失败都立即进入下一次，不先返回 C++。
    # ------------------------------------------------------------
    last_error = ""

    for attempt_index in range(
        1,
        MAX_ATTEMPTS + 1,
    ):
        print(
            "[Spark分类] ===== 内部尝试 {}/{} =====".format(
                attempt_index,
                MAX_ATTEMPTS,
            )
        )

        network_ok, response = (
            run_spark_attempt(
                prompt,
                attempt_index,
            )
        )

        if not network_ok:
            last_error = (
                "网络/响应阶段失败"
            )

            if (
                attempt_index
                < MAX_ATTEMPTS
            ):
                print(
                    "[Spark分类] 第{}次网络阶段失败，"
                    "不返回C++，立即建立新连接进行第{}次完整尝试...".format(
                        attempt_index,
                        attempt_index + 1,
                    )
                )
                continue

            break

        print(
            "[模型原始返回 attempt={}]: {}".format(
                attempt_index,
                response,
            )
        )

        try:
            (
                real_source,
                real_item,
                sim_source,
                sim_item,
                real_index,
                sim_index,
            ) = parse_and_validate_response(
                response,
                candidate_list,
            )

        except Exception as exc:
            last_error = (
                "解析/验证失败：{}".format(
                    exc
                )
            )

            print(
                "[分类解析失败 attempt={}]: {}".format(
                    attempt_index,
                    exc,
                )
            )

            if (
                attempt_index
                < MAX_ATTEMPTS
            ):
                print(
                    "[Spark分类] 已收到响应但内容不可用，"
                    "不返回C++，立即建立新连接进行第{}次完整尝试...".format(
                        attempt_index + 1
                    )
                )
                continue

            break

        # --------------------------------------------------------
        # 成功
        # --------------------------------------------------------
        print("[分类成功]")

        print(
            "实体区分配: r={} -> 原始候选=[{}] -> 中文播报名=[{}]".format(
                real_index,
                real_source,
                real_item,
            )
        )

        print(
            "仿真区分配: s={} -> 原始候选=[{}] -> 中文播报名=[{}]".format(
                sim_index,
                sim_source,
                sim_item,
            )
        )

        print(
            "[Spark分类] C++ 服务请求总耗时 {:.3f}s，"
            "成功attempt={}/{}".format(
                time.monotonic()
                - request_start,
                attempt_index,
                MAX_ATTEMPTS,
            )
        )

        return ItemClassifyResponse(
            success=True,
            real_item=real_item,
            sim_item=sim_item,
        )

    # ----------------------------------------------------------------
    # 两次完整 attempt 都失败后才返回 C++ false。
    # race.cpp 外层会继续按原逻辑兜底重试。
    # ----------------------------------------------------------------
    print(
        "[Spark分类] 两次完整尝试均失败，本次ROS服务返回失败；"
        "最后错误：{}".format(
            last_error
            if last_error
            else "未知错误"
        )
    )

    print(
        "[Spark分类] C++ 服务请求总耗时 {:.3f}s".format(
            time.monotonic()
            - request_start
        )
    )

    return ItemClassifyResponse(
        success=False,
        real_item="识别异常",
        sim_item="识别异常",
    )


# ============================================================================
# main
# ============================================================================

if __name__ == "__main__":
    rospy.init_node(
        "spark_classifier_server_node"
    )

    service = rospy.Service(
        "/get_item_classification",
        ItemClassify,
        handle_classification,
    )

    print(
        "星火 X2 物品分类服务端 V3 UDISK FIX 已启动"
    )
    print(
        "配置：thinking=disabled，max_tokens={}，"
        "connect_timeout={:.1f}s，final_timeout={:.1f}s，"
        "内部完整重试={}次".format(
            MAX_OUTPUT_TOKENS,
            CONNECT_TIMEOUT_SECONDS,
            FINAL_RESPONSE_TIMEOUT_SECONDS,
            MAX_ATTEMPTS,
        )
    )
    print(
        "返回协议："
        '{"r":候选编号,"rz":"实体中文名",'
        '"s":候选编号,"sz":"仿真中文名"}'
    )
    print(
        "等待 C++ 主控呼叫 /get_item_classification ..."
    )

    rospy.spin()