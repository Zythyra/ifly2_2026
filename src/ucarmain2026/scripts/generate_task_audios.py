#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import base64
import hashlib
import hmac
import json
import os
import ssl
import sys
import time
import wave
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time

import websocket


# 密钥从环境变量读取，避免提交到公开GitHub仓库。
APPID = "49973726"
API_SECRET = "YTE2MDNjNjdlZjU5NzE4ZDUzZTJmOTVi"
API_KEY = "a96d7ffe156859dc325d186a3bb20e17"
ENDPOINT_HOST = "tts-api.xfyun.cn"
ENDPOINT_PATH = "/v2/tts"
SIGN_HOSTS = ("ws-api.xfyun.cn", "tts-api.xfyun.cn")

OUTPUT_DIR = "/home/ucar/ucar_ws_copy/src/ucarmain2026/audios"


class XfyunTtsError(RuntimeError):
    pass


def check_credentials():
    missing = []
    if not APPID:
        missing.append("XFYUN_APPID")
    if not API_KEY:
        missing.append("XFYUN_API_KEY")
    if not API_SECRET:
        missing.append("XFYUN_API_SECRET")

    if missing:
        raise XfyunTtsError(
            "缺少环境变量：" + ", ".join(missing)
        )


def create_auth_url(sign_host):
    date = format_date_time(time.time())
    request_line = f"GET {ENDPOINT_PATH} HTTP/1.1"
    signature_origin = (
        f"host: {sign_host}\n"
        f"date: {date}\n"
        f"{request_line}"
    )

    signature_sha = hmac.new(
        API_SECRET.encode("utf-8"),
        signature_origin.encode("utf-8"),
        digestmod=hashlib.sha256
    ).digest()

    signature = base64.b64encode(signature_sha).decode("utf-8")
    authorization_origin = (
        f'api_key="{API_KEY}", '
        f'algorithm="hmac-sha256", '
        f'headers="host date request-line", '
        f'signature="{signature}"'
    )
    authorization = base64.b64encode(
        authorization_origin.encode("utf-8")
    ).decode("utf-8")

    query = urlencode({
        "authorization": authorization,
        "date": date,
        "host": sign_host
    })
    return f"wss://{ENDPOINT_HOST}{ENDPOINT_PATH}?{query}"


def ssl_options():
    options = {"cert_reqs": ssl.CERT_REQUIRED}
    try:
        import certifi
        options["ca_certs"] = certifi.where()
    except ImportError:
        pass
    return options


def request_pcm(text, sign_host):
    url = create_auth_url(sign_host)
    ws = None
    pcm_data = bytearray()

    try:
        print(f"🔗 正在连接讯飞，签名host={sign_host}")
        ws = websocket.create_connection(
            url,
            timeout=20,
            sslopt=ssl_options()
        )

        request_data = {
            "common": {
                "app_id": APPID
            },
            "business": {
                "aue": "raw",
                "auf": "audio/L16;rate=16000",
                "vcn": "xiaoyan",
                "speed": 50,
                "volume": 50,
                "pitch": 50,
                "bgs": 0,
                "tte": "utf8"
            },
            "data": {
                "status": 2,
                "text": base64.b64encode(
                    text.encode("utf-8")
                ).decode("utf-8")
            }
        }
        ws.send(json.dumps(request_data, ensure_ascii=False))

        while True:
            raw_message = ws.recv()
            if not raw_message:
                raise XfyunTtsError("WebSocket连接提前关闭")

            response = json.loads(raw_message)
            code = response.get("code", -1)
            if code != 0:
                raise XfyunTtsError(
                    "讯飞接口返回错误："
                    f"code={code}, "
                    f"message={response.get('message', '')}, "
                    f"sid={response.get('sid', '')}"
                )

            data = response.get("data")
            if not data:
                continue

            audio = data.get("audio")
            if audio:
                pcm_data.extend(base64.b64decode(audio))

            if data.get("status") == 2:
                break

    finally:
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass

    if not pcm_data:
        raise XfyunTtsError("接口未返回任何音频数据")

    return bytes(pcm_data)


def synthesize(text):
    errors = []
    for sign_host in SIGN_HOSTS:
        try:
            return request_pcm(text, sign_host)
        except Exception as error:
            errors.append(f"{sign_host}: {error}")
            print(f"⚠️ 当前签名方式失败：{error}")

    raise XfyunTtsError(
        "两种签名方式均失败：\n" + "\n".join(errors)
    )


def save_wav(filename, pcm_data):
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    temporary_file = filename + ".tmp"

    with wave.open(temporary_file, "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(16000)
        wav_file.writeframes(pcm_data)

    os.replace(temporary_file, filename)


def generate_wav(text, filename):
    print(f"⏳ 正在合成：{filename}")
    pcm_data = synthesize(text)
    save_wav(filename, pcm_data)
    print(f"✅ 成功合成：{filename}，PCM长度={len(pcm_data)}字节")


def main():
    # 程序名 + 三段文本，因此总参数数量应为4。
    if len(sys.argv) != 4:
        print(
            "用法：generate_task_audios.py "
            '"第二段文本" "第三段文本" "第四段文本"',
            file=sys.stderr
        )
        return 1

    try:
        check_credentials()
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        generate_wav(sys.argv[1], os.path.join(OUTPUT_DIR, "2.wav"))
        generate_wav(sys.argv[2], os.path.join(OUTPUT_DIR, "3.wav"))
        generate_wav(sys.argv[3], os.path.join(OUTPUT_DIR, "4.wav"))
        print("🎉 2.wav、3.wav、4.wav 全部生成完毕")
        return 0
    except Exception as error:
        print(f"❌ 批量音频合成失败：{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())