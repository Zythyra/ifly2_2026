#! /home/ucar/miniforge3/envs/opencv_env/bin/python
# -*- coding: UTF-8 -*-
#2025_5_29


import rospy
import threading
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from demo.demo import detect, init
import time
import os
import signal

class_names = ['Watermelon', 'Cake', 'Apple', 'Banana', 'Chili', 'Tomato', 'Milk', 'Cola', 'Potato']

class nanodet_detect():
    def __init__(self):
        # 初始化检测模型
        self.predictor = init()
        self.bridge = CvBridge()
        
        # 共享变量（需线程安全）
        self.latest_frame = None
        self.latest_result = []  # 初始化为空列表
        self.lock = threading.Lock()
        self.shutdown_flag = False  # 新增关闭标志
        
        # ROS 初始化
        rospy.set_param('/detect', 1)  # 显式初始化参数
        self.img_pub = rospy.Publisher('/nanodet/detect', Float64MultiArray, queue_size=10)
        rospy.Subscriber('/ucar_camera/image_raw', Image, self.image_callback, queue_size=1)
        
        # 启动工作线程
        self.detect_thread = threading.Thread(target=self.detection_worker)
        self.detect_thread.daemon = True
        self.publish_thread = threading.Thread(target=self.publish_worker)
        self.publish_thread.daemon = True
        self.shutdown_monitor = threading.Thread(target=self.monitor_shutdown)
        self.shutdown_monitor.daemon = True
        
        self.detect_thread.start()
        self.publish_thread.start()
        self.shutdown_monitor.start()

    def monitor_shutdown(self):
        """独立线程监控关闭信号"""
        rate = rospy.Rate(10)  # 10Hz检查频率
        while not rospy.is_shutdown() and not self.shutdown_flag:
            if rospy.get_param('/detect') == -1:
                rospy.loginfo("Received shutdown command, terminating...")
                self.shutdown_flag = True
                # 两种关闭方式确保可靠性
                # os.kill(os.getpid(), signal.SIGINT)  # 发送中断信号
                rospy.signal_shutdown("Admin shutdown request")
                break
            rate.sleep()

    def image_callback(self, img):
        """图像回调（非阻塞）"""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, 'bgr8')
            with self.lock:
                self.latest_frame = cv_img
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")

    def detection_worker(self):
        """检测工作线程"""
        rate = rospy.Rate(10)  # 10Hz检测频率
        while not rospy.is_shutdown() and not self.shutdown_flag:
            start_time = time.time()
            
            # 获取最新帧
            with self.lock:
                if self.latest_frame is None:
                    rate.sleep()
                    continue
                frame = self.latest_frame.copy()
            
            # 执行检测
            try:
                result = detect(frame, self.predictor)
                processed = self.process_result(result)
                with self.lock:
                    self.latest_result = processed
                
            except Exception as e:
                rospy.logwarn(f"Detection error: {e}")
                with self.lock:
                    self.latest_result = []
            
            # 保持10Hz频率
            elapsed = time.time() - start_time
            if elapsed < 0.1:
                rate.sleep()

    def process_result(self, result):
        """处理检测结果"""
        detections = []
        for class_id, boxes in result.items():
            if len(boxes) > 0 and boxes[0][4] > 0.75:  # 置信度阈值0.75
                x1, y1, x2, y2, score = boxes[0]
                detections.extend([float(class_id), x1, y1, x2, y2])
        return detections

    def publish_worker(self):
        """发布工作线程"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.shutdown_flag:
            with self.lock:
                current_result = self.latest_result if isinstance(self.latest_result, list) else []
            
            # 更安全的发布方式
            try:
                msg = Float64MultiArray(data=current_result)
                self.img_pub.publish(msg)
            except Exception as e:
                rospy.logerr(f"Publish failed: {e}")
            
            rate.sleep()

    def __del__(self):
        """析构函数确保资源释放"""
        if hasattr(self, 'predictor'):
            del self.predictor
        rospy.loginfo("Node resources cleaned up")

if __name__ == '__main__':
    rospy.init_node("nanodet_detect")
    rospy.loginfo("Nanodet detector started (use /detect=-1 to shutdown)")
    rospy.set_param('/detect', 1)
    try:
        detector = nanodet_detect()
        rospy.spin()
    finally:
        rospy.loginfo("Node shutdown complete")