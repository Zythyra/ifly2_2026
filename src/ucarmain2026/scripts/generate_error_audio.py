#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import websocket
import datetime
import hashlib
import base64
import hmac
import json
import os
import wave
import _thread as thread
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime

# 你的专属密钥
APPID = "49973726"
APISecret = "YTE2MDNjNjdlZjU5NzE4ZDUzZTJmOTVi"
APIKey = "a96d7ffe156859dc325d186a3bb20e17"

# 确保文件夹存在
OUTPUT_DIR = "/home/ucar/ucar_ws_copy/src/ucarmain2026/audios"
os.makedirs(OUTPUT_DIR, exist_ok=True)
OUTPUT_FILE = os.path.join(OUTPUT_DIR, "1.wav")

TEXT = "未能正确识别，请重新唤醒"
pcm_data = b''

class Ws_Param(object):
    def __init__(self, APPID, APIKey, APISecret):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.CommonArgs = {"app_id": self.APPID}
        self.BusinessArgs = {"aue": "raw", "auf": "audio/L16;rate=16000", "vcn": "xiaoyan", "speed": 50, "volume": 50, "pitch": 50, "bgs": 0, "tte": "utf8"}

    def create_url(self):
        url = 'wss://tts-api.xfyun.cn/v2/tts'
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))
        signature_origin = f"host: ws-api.xfyun.cn\ndate: {date}\nGET /v2/tts HTTP/1.1"
        signature_sha = base64.b64encode(hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'), digestmod=hashlib.sha256).digest()).decode('utf-8')
        authorization = base64.b64encode(f'api_key="{self.APIKey}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature_sha}"'.encode('utf-8')).decode('utf-8')
        return url + '?' + urlencode({"authorization": authorization, "date": date, "host": "ws-api.xfyun.cn"})

def on_message(ws, message):
    global pcm_data
    msg_dict = json.loads(message)
    if msg_dict["code"] == 0:
        pcm_data += base64.b64decode(msg_dict["data"]["audio"])
        if msg_dict["data"]["status"] == 2:
            ws.close()

def on_close(ws, *args):
    # 转换为标准 WAV 格式并保存
    with wave.open(OUTPUT_FILE, 'wb') as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(16000)
        wav_file.writeframes(pcm_data)
    print(f"✅ 提示音合成完毕！已保存至: {OUTPUT_FILE}")

def on_open(ws):
    def run(*args):
        text_b64 = base64.b64encode(TEXT.encode('utf-8')).decode('utf-8')
        d = {"common": ws.wsParam.CommonArgs, "business": ws.wsParam.BusinessArgs, "data": {"status": 2, "text": text_b64}}
        ws.send(json.dumps(d))
    thread.start_new_thread(run, ())

if __name__ == "__main__":
    print(f"⏳ 正在请求科大讯飞合成音频: [{TEXT}]...")
    wsParam = Ws_Param(APPID, APIKey, APISecret)
    ws = websocket.WebSocketApp(wsParam.create_url(), on_message=on_message, on_close=on_close)
    ws.wsParam = wsParam
    ws.on_open = on_open
    ws.run_forever(sslopt={"cert_reqs": 2})