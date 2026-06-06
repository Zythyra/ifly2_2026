#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# 生成播报的三个音频内容保存在本地
import sys
import websocket
import base64
import hmac
import hashlib
import json
import os
import wave
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
import _thread as thread

# 你的专属密钥
APPID = "49973726"
APISecret = "YTE2MDNjNjdlZjU5NzE4ZDUzZTJmOTVi"
APIKey = "a96d7ffe156859dc325d186a3bb20e17"

OUTPUT_DIR = "/home/ucar/ucar_car/src/ucarmain2026/audios"
os.makedirs(OUTPUT_DIR, exist_ok=True)

class Ws_Param:
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

def generate_wav(text, filename):
    pcm_data = bytearray()
    
    def on_message(ws, message):
        msg_dict = json.loads(message)
        if msg_dict["code"] == 0:
            pcm_data.extend(base64.b64decode(msg_dict["data"]["audio"]))
            if msg_dict["data"]["status"] == 2:
                ws.close()
                
    def on_close(ws, *args):
        with wave.open(filename, 'wb') as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(16000)
            wav_file.writeframes(pcm_data)
        print(f"✅ 成功合成并保存: {filename}")
        
    def on_open(ws):
        def run(*args):
            text_b64 = base64.b64encode(text.encode('utf-8')).decode('utf-8')
            d = {"common": ws.wsParam.CommonArgs, "business": ws.wsParam.BusinessArgs, "data": {"status": 2, "text": text_b64}}
            ws.send(json.dumps(d))
        thread.start_new_thread(run, ())

    wsParam = Ws_Param(APPID, APIKey, APISecret)
    ws = websocket.WebSocketApp(wsParam.create_url(), on_message=on_message, on_close=on_close)
    ws.wsParam = wsParam
    ws.on_open = on_open
    ws.run_forever(sslopt={"cert_reqs": 2})

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("参数不足")
        sys.exit(1)
    
    print("⏳ 开始批量合成离线音频...")
    generate_wav(sys.argv[1], os.path.join(OUTPUT_DIR, "2.wav"))
    generate_wav(sys.argv[2], os.path.join(OUTPUT_DIR, "3.wav"))
    generate_wav(sys.argv[3], os.path.join(OUTPUT_DIR, "4.wav"))
    print("🎉 全部音频合成完毕！")