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
        print("✅ 小车主机初始化完成，等待PC指令...")

    def on_pc_message(self, msg):
        data = msg['data']
        
        if data.startswith('NAVIGATING_TO:'):
            target = data.split(':')[1]
            print(f"收到导航目标: {target}")
        elif data == 'NAVIGATION_COMPLETED':
            print("导航完成!")

    def start(self):
        try:
            while self.client.is_connected:
                time.sleep(1)
        except KeyboardInterrupt:
            self.client.close()

if __name__ == '__main__':
    car_host = CarAsHost()
    car_host.start()