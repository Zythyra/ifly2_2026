#! /home/ucar/miniforge3/envs/opencv_env/bin/python
# -*- coding: UTF-8 -*-
#2025_7_6


# 可以采集原始大小的数据
import rospy
from std_msgs.msg import Header, Float64, Int8
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
import os

class zhong_detect():
    def __init__(self):
        im_sub = rospy.Subscriber('/ucar_camera/image_raw', Image, self.detectimg)
        self.bridge = CvBridge()  # OpenCV与ROS的消息转换类

    def detectimg(self, img):
        frame = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        cv2.imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/for_loc/1.jpg",frame)


if __name__ == '__main__':
    rospy.init_node("take")
    rospy.loginfo("take node started")
    detector = zhong_detect()
    rospy.spin()