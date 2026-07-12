# #!/miniforge3/envs/opencv_env/bin/python
# # -*- coding: utf-8 -*-  

# import roslibpy
# import time

# class CarAsHost:
#     def __init__(self):
#         self.client = roslibpy.Ros(host='192.168.1.108', port=9090)
#         self.client.run()
        
#         # 接收PC消息
#         self.subscriber = roslibpy.Topic(self.client, '/pc_to_car', 'std_msgs/String')
#         self.subscriber.subscribe(self.on_pc_message)
        
#         # 添加发布者用于发送指令给PC端
#         self.car_to_pc = roslibpy.Topic(self.client, '/car_to_pc', 'std_msgs/String')
        
#         # 存储检测结果
#         self.detected_items = []
        
#         print("✅ 小车主机初始化完成，等待PC连接...")

#     def on_pc_message(self, msg):
#         data = msg['data']
        
#         if data == 'PC_READY':
#             print("检测到PC端已启动，5秒后发送导航指令")
#             time.sleep(5)
#             self.car_to_pc.publish(roslibpy.Message({'data': 'START_NAVIGATION'}))
#             print("已发送导航启动指令给PC端")
#         elif data.startswith('DETECTED_ITEMS:'):
#             items = data.split(':')[1]
#             self.detected_items = items.split(',')
#             print("收到检测物品列表: {}".format(self.detected_items))
#         elif data == 'NAVIGATION_COMPLETED':
#             print("导航完成! 检测到的物品有:", self.detected_items)

#     def start(self):
#         try:
#             while self.client.is_connected:
#                 time.sleep(1)
#         except KeyboardInterrupt:
#             self.client.close()

# if __name__ == '__main__':
#     car_host = CarAsHost()
#     car_host.start()

#!/miniforge3/envs/opencv_env/bin/python
# -*- coding: utf-8 -*-  

import roslibpy
import time

class CarAsHost:
    def __init__(self):
        self.client = roslibpy.Ros(host='192.168.1.108', port=9090)
        self.client.run()
        
        # 接收PC消息
        self.subscriber = roslibpy.Topic(self.client, '/pc_to_car', 'std_msgs/String')
        self.subscriber.subscribe(self.on_pc_message)
        
        # 添加发布者用于发送指令给PC端
        self.car_to_pc = roslibpy.Topic(self.client, '/car_to_pc', 'std_msgs/String')
        
        # 存储检测结果
        self.detected_items = []
        
        print("✅ 小车主机初始化完成，准备发送启动指令...")
        time.sleep(2)  # 短暂延迟确保连接稳定
        self.car_to_pc.publish(roslibpy.Message({'data': 'START_NAVIGATION'}))
        print("已发送导航启动指令给PC端")

    def on_pc_message(self, msg):
        data = msg['data']
        
        if data.startswith('DETECTED_ITEMS:'):
            items = data.split(':')[1]
            self.detected_items = items.split(',')
            print("收到检测物品列表: {}".format(self.detected_items))
        elif data == 'NAVIGATION_COMPLETED':
            print("导航完成! 检测到的物品有:", self.detected_items)

    def start(self):
        try:
            while self.client.is_connected:
                time.sleep(1)
        except KeyboardInterrupt:
            self.client.close()

if __name__ == '__main__':
    car_host = CarAsHost()
    car_host.start()