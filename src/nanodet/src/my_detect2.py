#! /home/ucar/miniforge3/envs/opencv_env/bin/python
# -*- coding: UTF-8 -*-
#2025_5_29

import rospy
from std_msgs.msg import Header, Float64, Int8
from sensor_msgs.msg import Image
from std_msgs.msg import String
from demo.demo2 import detect
from demo.demo2 import init
import cv2
from cv_bridge import CvBridge

# 定义类别名称和对应的编号
class_names = ['Red', 'Green']

# 创建一个字典，将类别名称映射到对应的编号
class_mapping = {name: i for i, name in enumerate(class_names)}

global m, cap, predictor, frame
m = 0
predictor = init()


class nanodet_detect():
    def __init__(self):
        im_sub = rospy.Subscriber('/ucar_camera/image_raw', Image, self.detectimg)
        # self.img_pub = rospy.Publisher('/nanodet/detect2', Int8, queue_size=1)
        self.bridge = CvBridge()  # OpenCV与ROS的消息转换类

    def detectimg(self, img):
        frame = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        result = detect(frame, predictor)
        # rospy.loginfo("%s",result)
        # 遍历检测结果
        #这段循环来遍历一帧里所有检测结果和置信度
        all_detections = []
        for class_id, detections in result.items():
            # print(detections)
            # for detection in detections:
            #     if len(detection) >= 5:  # 确保数据完整[x1,y1,x2,y2,score]
            if len(detections) > 0: 
                    x1, y1, x2, y2, score = detections[0][:5]
                    if score > 0.3:  # 置信度阈值
                        all_detections.append({
                            'class_id': int(class_id),
                            'class_name': class_names[int(class_id)],
                            'bbox': [float(x1), float(y1), float(x2), float(y2)],
                            'score': float(score)
                        })

        # 找出最左侧的物体（x1最小的）
        if all_detections:
            leftmost_obj = min(all_detections, key=lambda x: x['bbox'][0])

        # 发布结果
        value = rospy.get_param('detect2')
        if value == -1:
            rospy.loginfo("nanokill2")
            rospy.signal_shutdown("Received stop signal")
        if value == 1:
            if not all_detections:
                rospy.loginfo("No object detected in this frame2")
            else:
                # self.img_pub.publish(leftmost_obj['class_id'])
                rospy.set_param('light', leftmost_obj['class_id'] + 1)
                rospy.loginfo(f"Leftmost object: {leftmost_obj['class_name']} (ID: {leftmost_obj['class_id']})")
                return


if __name__ == '__main__':
    rospy.init_node("nanodet_detect2")
    rospy.loginfo("nanodet_detect node started2")
    rospy.set_param("detect2",0)
    # value = 0
    # rospy.loginfo("%d",value)
    # while value != -1:
    #     while not rospy.is_shutdown():
    #         if value == 1:
    detector = nanodet_detect()
    #         value = rospy.get_param('detect2')
    # if value == -1:
    #     rospy.loginfo("nanokill2")
    #     rospy.signal_shutdown("Received stop signal")
    rospy.spin()