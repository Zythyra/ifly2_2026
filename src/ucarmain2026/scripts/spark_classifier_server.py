#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# 星火大模型(X2) - 扫码物品分类推理节点

import websocket
import hashlib
import base64
import hmac
import json
import os
import time
import re
import ssl
from urllib.parse import urlparse, urlencode
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
import _thread as thread
import rospy

# 引入自定义服务
from ucarmain2026.srv import ItemClassify, ItemClassifyResponse

# ================= 核心密钥配置 =================
APPID = "49973726"
APISecret = "YTE2MDNjNjdlZjU5NzE4ZDUzZTJmOTVi"
APIKey = "a96d7ffe156859dc325d186a3bb20e17"

final_response = ""

class Ws_Param(object):
    def __init__(self, APPID, APIKey, APISecret):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.host = "spark-api.xf-yun.com"
        # 【核心修正1】：保持你想要的 X2 路径
        self.path = "/x2"  
        self.url = f"wss://{self.host}{self.path}"

    def create_url(self):
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))
        signature_origin = f"host: {self.host}\ndate: {date}\nGET {self.path} HTTP/1.1"
        signature_sha = base64.b64encode(hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'), digestmod=hashlib.sha256).digest()).decode('utf-8')
        authorization_origin = f'api_key="{self.APIKey}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature_sha}"'
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8')
        v = {"authorization": authorization, "date": date, "host": self.host}
        return self.url + '?' + urlencode(v)

def on_message(ws, message):
    global final_response
    try:
        data = json.loads(message)
        code = data['header']['code']
        if code != 0:
            print(f"\n❌ 大模型请求报错: {code}, {data['header']['message']}")
            ws.close()
            return
        
        choices = data['payload']['choices']
        status = choices['status']
        content = choices['text'][0]['content']
        final_response += content
        
        if status == 2:  # 接收完毕
            ws.close()
    except Exception as e:
        print("❌ 消息解析异常:", e)

def on_error(ws, error):
    print("❌ WebSocket 报错:", error)

def on_close(ws, a, b):
    pass

def handle_classification(req):
    global final_response
    final_response = "" 
    
    print("\n==============================================")
    print("🔄 收到主控分类请求，正在呼叫星火 X2 大模型...")
    print(f"📦 候选物品: [{req.item1}], [{req.item2}], [{req.item3}]")
    print(f"🎯 目标类别: 实体区=[{req.target_real}], 仿真区=[{req.target_sim}]")
    
    # 提示词工程：强化 JSON 输出约束
    prompt = f"""
    你是一个智能仓储分类助手。
    现在有三个物品：{req.item1}、{req.item2}、{req.item3}。
    我需要你进行逻辑分类：
    这三个物品中，哪一个最符合“{req.target_real}”这个类别？
    哪一个最符合“{req.target_sim}”这个类别？
    
    请严格只回复一个 JSON 字符串，绝对不要输出任何其他解释文字和Markdown符号！格式如下：
    {{"real_item": "此处填入物品名", "sim_item": "此处填入物品名"}}
    """

    wsParam = Ws_Param(APPID, APIKey, APISecret)
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp(wsParam.create_url(), on_message=on_message, on_error=on_error, on_close=on_close)

    def on_open(ws):
        def run(*args):
            data = {
                "header": {"app_id": APPID, "uid": "ucar_bot"},
                "parameter": {
                    "chat": {
                        # 【核心修正2】：X2 模型在后端的专属鉴权代号是 spark-x
                        "domain": "spark-x",  
                        "temperature": 0.1,  
                        "max_tokens": 128
                    }
                },
                "payload": {
                    "message": {
                        "text": [{"role": "user", "content": prompt}]
                    }
                }
            }
            ws.send(json.dumps(data))
        thread.start_new_thread(run, ())

    ws.on_open = on_open
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

    print(f"🧠 [模型原始返回]: {final_response}")
    
    # 【核心修正3】：增加判空防崩溃保护机制
    try:
        # 如果模型因为鉴权失败返回空内容，主动拦截而不是抛给正则引擎
        if not final_response:
            raise ValueError("未收到大模型的有效数据（鉴权被拒绝或网络超时）。")

        match = re.search(r'\{.*?\}', final_response, re.DOTALL)
        if not match:
            raise ValueError("大模型返回的内容中没有合法的 JSON 括号。")

        json_str = match.group()
        result_dict = json.loads(json_str)
        
        r_item = result_dict.get("real_item", "未知物品")
        s_item = result_dict.get("sim_item", "未知物品")
        
        print("✅ [分类成功]")
        print(f"-> 实体区分配: {r_item}")
        print(f"-> 仿真区分配: {s_item}")
        
        return ItemClassifyResponse(success=True, real_item=r_item, sim_item=s_item)
        
    except Exception as e:
        print(f"❌ [分类解析失败]: {e}")
        # 即便发生错误，也会优雅地给主控返回兜底文本，绝不让 C++ 节点死锁
        return ItemClassifyResponse(success=False, real_item="识别异常", sim_item="识别异常")

if __name__ == "__main__":
    rospy.init_node('spark_classifier_server_node')
    s = rospy.Service('/get_item_classification', ItemClassify, handle_classification)
    print("🟢 星火大模型 X2 [物品分类]服务端已启动，等待 C++ 主控呼叫...")
    rospy.spin()