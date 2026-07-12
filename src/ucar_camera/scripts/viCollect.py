#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class VisionBasedNavigator:
    def __init__(self):
        rospy.init_node('vision_based_navigator', anonymous=True)
        
        # 订阅视觉检测话题
        self.vision_sub = rospy.Subscriber('/nanodet/detect', Float64MultiArray, self.vision_callback)
        
        # 发布速度命令话题
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 相机参数
        self.cam_width = 1280.0
        self.cam_height = 960.0
        self.cam_center_x = self.cam_width / 2  # 640
        self.cam_center_y = self.cam_height / 2  # 480
        
        # 控制参数（可根据机器人动态调整）
        self.linear_speed = 0.2  # 前进速度 (m/s)
        self.angular_gain = 0.005  # 角速度比例增益
        
        # 当前边界框（xmin, ymin, xmax, ymax）
        self.bbox = None

    def vision_callback(self, msg):
        data = msg.data
        if len(data) < 6 or (len(data) - 1) % 5 != 0:
            rospy.logwarn("无效的检测消息格式")
            self.bbox = None
            return
        
        # 解析第一个检测（class_id, xl, yl, xr, yr），忽略后续和时间戳
        xmin = data[1]
        ymin = data[2]
        xmax = data[3]
        ymax = data[4]
        
        # 验证边界框有效性
        if xmin < xmax and ymin < ymax and 0 <= xmin < self.cam_width and 0 <= xmax < self.cam_width:
            self.bbox = {'xmin': xmin, 'ymin': ymin, 'xmax': xmax, 'ymax': ymax}
        else:
            self.bbox = None

    def navigate(self):
        rate = rospy.Rate(5)  # 5Hz，与视觉话题匹配
        while not rospy.is_shutdown():
            twist = Twist()
            
            if self.bbox is not None:
                # 计算边界框中心
                center_x = (self.bbox['xmin'] + self.bbox['xmax']) / 2
                # center_y = (self.bbox['ymin'] + self.bbox['ymax']) / 2  # 可选使用y轴
                
                # 计算水平误差（用于转向控制）
                error_x = self.cam_center_x - center_x
                
                # 设置速度
                twist.linear.x = self.linear_speed  # 检测到目标时前进
                twist.angular.z = self.angular_gain * error_x  # 转向以居中目标
                
                # 可选：根据边界框大小判断距离（例如太大则停止）
                bbox_width = self.bbox['xmax'] - self.bbox['xmin']
                if bbox_width > 0.8 * self.cam_width:
                    twist.linear.x = 0.0
            else:
                # 未检测到目标：停止
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = VisionBasedNavigator()
        navigator.navigate()
    except rospy.ROSInterruptException:
        pass