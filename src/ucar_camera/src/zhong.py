#! /home/ucar/miniforge3/envs/opencv_env/bin/python
# -*- coding: UTF-8 -*-
#2025_5_29

import rospy
from std_msgs.msg import Header, Float64, Int8
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
import os

global m, cap, predictor, frame, xflag
m = 0
xflag = 0

class zhong_detect():
    def __init__(self):
        im_sub = rospy.Subscriber('/ucar_camera/image_raw', Image, self.detectimg)
        self.img_pub = rospy.Publisher('/zhong/detect', Int8, queue_size=1)
        self.bridge = CvBridge()  # OpenCV与ROS的消息转换类

    def detectimg(self, img):
        frame = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        # print(frame.shape)
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        # ed_lines = cv2.createLineSegmentDetector(refine=cv2.LSD_REFINE_STD)
        # lines, width, prec, nfa = ed_lines.detect(edges)
        # if lines is not None:
        #     ines = lines.reshape(-1, 1, 4)
        #     for x1, y1, x2, y2 in lines[:, 0]:
        #         cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        # cv2.imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/testzhong.jpg",frame)
        frame = frame[500:720,320:960] 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (5, 5), 0)  # 降噪
        edges = cv2.Canny(gray, 50, 120)
        line = 100
        minLineLength = 150
        maxLineGap = 10
        lines = cv2.HoughLinesP(edges, 1, np.pi / 90, 10, lines=line, minLineLength=minLineLength,maxLineGap=maxLineGap)
        if lines is not None:
            lines1 = lines[:, 0, :]
            for x1, y1, x2, y2 in lines1:
                angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
                if -10 <= angle <= 10:
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/testzhong.jpg",frame)
                    # rospy.set_param("/zhong",1)
                    rospy.set_param("/all_done",1)
                    # os.system("rosnode kill /base_driver")
                    rospy.loginfo("find zhong")
                    rospy.signal_shutdown("finish")
        # else:
        #     rospy.loginfo("noline")
        cv2.imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/testzhong.jpg",frame)


if __name__ == '__main__':
    rospy.init_node("zhong_detect")
    rospy.loginfo("zhong_detect node started")
    rospy.set_param("/zf",0)#障碍物避完参数
    rospy.set_param("/all_done",0)
    # rospy.set_param("/xflag",0)
    while xflag==0:
        xflag=rospy.get_param("/zf")
    rospy.loginfo("開始終點識別")
    detector = zhong_detect()
    rospy.spin()