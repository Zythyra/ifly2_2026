# -*- coding:utf-8 -*-
# 独立测试节点：读取本地 WAV 音频并调用【讯飞星火中文识别大模型】API
import websocket
import datetime
import hashlib
import base64
import hmac
import json
import os
import time
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
import _thread as thread

# ================= 你的专属密钥与配置 =================
APPID = "49973726"
APISecret = "YTE2MDNjNjdlZjU5NzE4ZDUzZTJmOTVi"
APIKey = "a96d7ffe156859dc325d186a3bb20e17"

# 音频文件绝对路径
AUDIO_FILE = "/home/ucar/ucar_car/wakeup_record/test_record.wav"
# ======================================================

STATUS_FIRST_FRAME = 0
STATUS_CONTINUE_FRAME = 1
STATUS_LAST_FRAME = 2

final_result = ""

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

        signature_origin = "host: " + "iat.xf-yun.com" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v1 " + "HTTP/1.1"

        signature_sha = hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'),
                                 digestmod=hashlib.sha256).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding='utf-8')

        authorization_origin = "api_key=\"%s\", algorithm=\"%s\", headers=\"%s\", signature=\"%s\"" % (
            self.APIKey, "hmac-sha256", "host date request-line", signature_sha)
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8')
        
        v = {
            "authorization": authorization,
            "date": date,
            "host": "iat.xf-yun.com"
        }
        return url + '?' + urlencode(v)


def on_message(ws, message):
    global final_result
    try:
        msg_dict = json.loads(message)
        
        # 【大模型修正】：提取嵌套在 header 里的 code
        code = msg_dict["header"]["code"]
        if code != 0:
            errMsg = msg_dict["header"]["message"]
            print(f"❌ 请求错误: {code}, 错误信息: {errMsg}")
            return
            
        # 【大模型修正】：提取被层层包裹的 Base64 文本结果
        payload = msg_dict.get("payload")
        if payload and "result" in payload:
            text_base64 = payload["result"]["text"]
            # 解开 base64 拿到真正的 json 字符串
            text_json_str = base64.b64decode(text_base64).decode('utf-8')
            text_json = json.loads(text_json_str)
            
            # 解析出中文字符
            ws_arr = text_json.get("ws", [])
            for i in ws_arr:
                for w in i.get("cw", []):
                    final_result += w.get("w", "")
                    
    except Exception as e:
        print("❌ 接收消息解析异常:", e)
        print("-> 原始数据为:", message)

def on_error(ws, error):
    print("❌ WebSocket 报错:", error)

def on_close(ws, close_status_code, close_msg):
    global final_result
    print("\n=================================================")
    print("🧠 [星火大模型识别完成] 最终语音内容：")
    print(f"👉 {final_result}")
    print("=================================================\n")

def on_open(ws):
    def run(*args):
        frameSize = 8000
        intervel = 0.04
        status = STATUS_FIRST_FRAME
        seq = 1 # 【大模型修正】：加入必选的 seq 序列号校验

        with open(wsParam.AudioFile, "rb") as fp:
            while True:
                buf = fp.read(frameSize)
                if not buf:
                    status = STATUS_LAST_FRAME

                if status == STATUS_FIRST_FRAME:
                    # 【大模型修正】：标准三段式 Header - Parameter - Payload
                    d = {
                        "header": {
                            "app_id": wsParam.APPID,
                            "status": 0
                        },
                        "parameter": {
                            "iat": {
                                "domain": "slm", # 启用 Speech Large Model 大模型
                                "language": "zh_cn",
                                "accent": "mandarin",
                                "result": {
                                    "encoding": "utf8",
                                    "compress": "raw",
                                    "format": "json"
                                }
                            }
                        },
                        "payload": {
                            "audio": {
                                "encoding": "raw",
                                "sample_rate": 16000,
                                "channels": 1,
                                "bit_depth": 16,
                                "seq": seq,
                                "status": 0,
                                "audio": str(base64.b64encode(buf), 'utf-8')
                            }
                        }
                    }
                    ws.send(json.dumps(d))
                    status = STATUS_CONTINUE_FRAME
                
                elif status == STATUS_CONTINUE_FRAME:
                    seq += 1
                    d = {
                        "header": {"app_id": wsParam.APPID, "status": 1},
                        "payload": {
                            "audio": {
                                "encoding": "raw", "sample_rate": 16000, "channels": 1,
                                "bit_depth": 16, "seq": seq, "status": 1,
                                "audio": str(base64.b64encode(buf), 'utf-8')
                            }
                        }
                    }
                    ws.send(json.dumps(d))
                
                elif status == STATUS_LAST_FRAME:
                    seq += 1
                    d = {
                        "header": {"app_id": wsParam.APPID, "status": 2},
                        "payload": {
                            "audio": {
                                "encoding": "raw", "sample_rate": 16000, "channels": 1,
                                "bit_depth": 16, "seq": seq, "status": 2,
                                "audio": ""
                            }
                        }
                    }
                    ws.send(json.dumps(d))
                    time.sleep(1)
                    break
                
                time.sleep(intervel)
        ws.close()

    thread.start_new_thread(run, ())

if __name__ == "__main__":
    if not os.path.exists(AUDIO_FILE):
        print(f"❌ 找不到音频文件，请先运行录音节点: {AUDIO_FILE}")
        exit()
        
    print(f"🚀 开始加载音频文件: {AUDIO_FILE}")
    print("🔄 正在与讯飞【星火中英识别大模型】建立连接...")
    
    wsParam = Ws_Param(APPID, APIKey, APISecret, AUDIO_FILE)
    websocket.enableTrace(False)
    wsUrl = wsParam.create_url()
    
    ws = websocket.WebSocketApp(wsUrl, on_message=on_message, on_error=on_error, on_close=on_close)
    ws.on_open = on_open
    ws.run_forever(sslopt={"cert_reqs": 2})