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
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
import _thread as thread

import rospy
# 【标准规范】：引入咱们刚刚亲手在 ucarmain2026 里编译出来的自定义服务！
from ucarmain2026.srv import GetTaskSemantics, GetTaskSemanticsResponse

APPID = "49973726"
APISecret = "YTE2MDNjNjdlZjU5NzE4ZDUzZTJmOTVi"
APIKey = "a96d7ffe156859dc325d186a3bb20e17"
AUDIO_FILE = "/home/ucar/ucar_car/wakeup_record/test_record.wav"

TARGET_DICT = {"食品": "food", "日用品": "daily", "电子产品": "electronic"}
final_result = ""

class Ws_Param(object):
    def __init__(self, APPID, APIKey, APISecret, AudioFile):
        self.APPID, self.APIKey, self.APISecret, self.AudioFile = APPID, APIKey, APISecret, AudioFile

    def create_url(self):
        url = 'wss://iat.xf-yun.com/v1'
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))
        signature_origin = f"host: iat.xf-yun.com\ndate: {date}\nGET /v1 HTTP/1.1"
        signature_sha = base64.b64encode(hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'), digestmod=hashlib.sha256).digest()).decode('utf-8')
        authorization = base64.b64encode(f'api_key="{self.APIKey}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature_sha}"'.encode('utf-8')).decode('utf-8')
        return url + '?' + urlencode({"authorization": authorization, "date": date, "host": "iat.xf-yun.com"})

def on_message(ws, message):
    global final_result
    try:
        msg_dict = json.loads(message)
        if msg_dict["header"]["code"] == 0:
            text_json = json.loads(base64.b64decode(msg_dict["payload"]["result"]["text"]).decode('utf-8'))
            for i in text_json.get("ws", []):
                for w in i.get("cw", []):
                    final_result += w.get("w", "")
    except Exception: pass

def on_error(ws, error): print("❌ WebSocket 报错:", error)
def on_close(ws, a, b): pass

def on_open(ws):
    def run(*args):
        frameSize = 8000
        seq = 1
        with open(AUDIO_FILE, "rb") as fp:
            while True:
                buf = fp.read(frameSize)
                status = 0 if seq == 1 else (2 if not buf else 1)
                payload_data = {"audio": {"encoding": "raw", "sample_rate": 16000, "channels": 1, "bit_depth": 16, "seq": seq, "status": status, "audio": base64.b64encode(buf).decode('utf-8') if buf else ""}}
                d = {"header": {"app_id": APPID, "status": status}, "payload": payload_data}
                if status == 0: d["parameter"] = {"iat": {"domain": "slm", "language": "zh_cn", "accent": "mandarin", "result": {"encoding": "utf8", "compress": "raw", "format": "json"}}}
                ws.send(json.dumps(d))
                if status == 2:
                    time.sleep(1)
                    break
                seq += 1
                time.sleep(0.04)
        ws.close()
    thread.start_new_thread(run, ())

# ================= 核心：响应 C++ 的服务回调 =================
def handle_semantic_request(req):
    global final_result
    final_result = "" 
    
    if not os.path.exists(AUDIO_FILE):
        return GetTaskSemanticsResponse(success=False, target_real="NONE", target_sim="NONE")

    print("\n🔄 收到 C++ 主控呼叫！正在连接星火大模型...")
    wsParam = Ws_Param(APPID, APIKey, APISecret, AUDIO_FILE)
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp(wsParam.create_url(), on_message=on_message, on_error=on_error, on_close=on_close)
    ws.on_open = on_open
    ws.run_forever(sslopt={"cert_reqs": 2})

    print(f"🧠 [语音原文]: {final_result}")
    
    matches = re.findall(r"(食品|日用品|电子产品)", final_result)
    
    if len(matches) >= 2:
        en_target_1 = TARGET_DICT[matches[0]]
        en_target_2 = TARGET_DICT[matches[1]]
        print(f"✅ [解析成功] 实体区: {en_target_1}, 仿真区: {en_target_2}")
        # 结构化返回数据
        return GetTaskSemanticsResponse(success=True, target_real=en_target_1, target_sim=en_target_2)
    else:
        print("❌ [提取失败] 格式不符合预期")
        return GetTaskSemanticsResponse(success=False, target_real="ERROR", target_sim="ERROR")

if __name__ == "__main__":
    rospy.init_node('spark_semantic_server_node')
    s = rospy.Service('/get_task_semantics', GetTaskSemantics, handle_semantic_request)
    print("🟢 规范化星火大模型服务已启动，等待 C++ 主控呼叫...")
    rospy.spin()