#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# 2026 智能车比赛 - 在线流式语音合成 (TTS) 节点

import websocket
import datetime
import hashlib
import base64
import hmac
import json
import os
import _thread as thread
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
import rospy
from std_msgs.msg import String

# ================= 你的专属密钥与配置 =================
APPID = "49973726"
APISecret = "YTE2MDNjNjdlZjU5NzE4ZDUzZTJmOTVi"
APIKey = "a96d7ffe156859dc325d186a3bb20e17"

# 极速缓存的裸音频存放点 (放在 /tmp 内存盘里，读写速度最快)
OUTPUT_FILE = "/tmp/ucar_online_tts.pcm"
# ======================================================

class Ws_Param(object):
    def __init__(self, APPID, APIKey, APISecret):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.CommonArgs = {"app_id": self.APPID}
        # aue=raw: 请求原生 PCM 数据; vcn=xiaoyan: 甜美女声; volume=50: 音量
        self.BusinessArgs = {"aue": "raw", "auf": "audio/L16;rate=16000", "vcn": "xiaoyan", "speed": 50, "volume": 50, "pitch": 50, "bgs": 0, "tte": "utf8"}

    def create_url(self):
        url = 'wss://tts-api.xfyun.cn/v2/tts'
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))
        signature_origin = "host: ws-api.xfyun.cn\ndate: " + date + "\nGET /v2/tts HTTP/1.1"
        signature_sha = base64.b64encode(hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'), digestmod=hashlib.sha256).digest()).decode('utf-8')
        authorization_origin = "api_key=\"%s\", algorithm=\"%s\", headers=\"%s\", signature=\"%s\"" % (self.APIKey, "hmac-sha256", "host date request-line", signature_sha)
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8')
        v = {"authorization": authorization, "date": date, "host": "ws-api.xfyun.cn"}
        return url + '?' + urlencode(v)

def on_message(ws, message):
    try:
        msg_dict = json.loads(message)
        code = msg_dict["code"]
        if code != 0:
            errMsg = msg_dict["message"]
            print(f"❌ 合成报错: {code}, {errMsg}")
            return
            
        audio = msg_dict["data"]["audio"]
        audio_bg = base64.b64decode(audio)
        
        # 将接收到的音频流源源不断地追加到临时文件中
        with open(OUTPUT_FILE, "ab") as f:
            f.write(audio_bg)
            
        # 如果 status 为 2，说明云端已经把这句话发完了
        if msg_dict["data"]["status"] == 2:
            ws.close()
    except Exception as e:
        print("❌ 接收消息异常:", e)

def on_error(ws, error):
    print("❌ WebSocket 报错:", error)

def on_close(ws, *args):
    # 【极客绝杀】：连接一关闭，瞬间调用 Linux 底层 ALSA 音频驱动进行裸流外放！
    # -t raw: 纯 PCM; -c 1: 单声道; -f S16_LE: 16位精度; -r 16000: 采样率
    play_cmd = f"aplay -t raw -c 1 -f S16_LE -r 16000 -q {OUTPUT_FILE}"
    os.system(play_cmd)

def on_open(ws):
    def run(*args):
        # 将文字进行 Base64 编码后发给云端
        text_b64 = base64.b64encode(ws.tts_text.encode('utf-8')).decode('utf-8')
        d = {
            "common": ws.wsParam.CommonArgs,
            "business": ws.wsParam.BusinessArgs,
            "data": {"status": 2, "text": text_b64}
        }
        ws.send(json.dumps(d))
    thread.start_new_thread(run, ())

# ================= 核心：ROS 回调函数 =================
def tts_callback(msg):
    text = msg.data
    print(f"\n🎙️ 收到主控播报指令: [{text}]")
    
    # 每次合成前清空旧的音频缓存
    if os.path.exists(OUTPUT_FILE):
        os.remove(OUTPUT_FILE)
        
    wsParam = Ws_Param(APPID, APIKey, APISecret)
    wsUrl = wsParam.create_url()
    
    # 动态挂载文本和参数
    ws = websocket.WebSocketApp(wsUrl, on_message=on_message, on_error=on_error, on_close=on_close)
    ws.tts_text = text
    ws.wsParam = wsParam
    ws.on_open = on_open
    
    # 阻塞执行网络请求和播放
    ws.run_forever(sslopt={"cert_reqs": 2})
    print("✅ 播报完毕！")

if __name__ == "__main__":
    rospy.init_node('online_tts_node', anonymous=True)
    rospy.Subscriber('/voice_tts', String, tts_callback)
    
    print("========================================")
    print("🟢 星火云端在线 TTS 播报节点已启动！")
    print("👂 正在监听 /voice_tts 话题，准备随时开口...")
    print("========================================")
    
    rospy.spin()