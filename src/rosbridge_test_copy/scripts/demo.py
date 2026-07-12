import websocket
import json

def on_message(ws, message):
    data = json.loads(message)
    if "msg" in data:
        imu_data = data['msg']
        print(f"[IMU Data] x={imu_data['linear_acceleration']['x']}, y={imu_data['linear_acceleration']['y']}, z={imu_data['linear_acceleration']['z']}")

def on_error(ws, error):
    print(f"[WebSocket Error]: {error}")

def on_close(ws, close_status_code, close_msg):
    print("WebSocket connection closed")

def on_open(ws):
    subscribe_message = {
        "op": "subscribe",
        "topic": "/imu"  # 修改为实际需要订阅的ROS话题（例如/imu）
    }
    ws.send(json.dumps(subscribe_message))
    print("Subscribed to /imu")

if __name__ == "__main__":
    websocket.enableTrace(True)  # 启用调试日志（可选）
    ws_url = "ws://192.168.48.221:9090"  # 修改为rosbridge_server的IP和端口（默认9090）
    
    ws = websocket.WebSocketApp(
        ws_url,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )
    ws.run_forever()  # 启动WebSocket客户端