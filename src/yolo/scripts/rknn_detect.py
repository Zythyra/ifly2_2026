#!/home/ucar/miniforge3/envs/rknn_env/bin/python3
# -*- coding: UTF-8 -*-
import os
os.environ['ROSCONSOLE_CONFIG_FILE'] = '/dev/null'
os.environ['ROSCONSOLE_FORMAT'] = '{message}'
os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = ''

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rknn import YOLOv5RKNN

class_names = ['Watermelon', 'Cake', 'Apple', 'Banana', 'Chili', 'Tomato', 'Milk', 'Cola', 'Potato']

class YOLOv5RknnDetect:
    def __init__(self):
        rospy.set_param('/detect', 1)  # 初始化检测标志位
        self.detector = YOLOv5RKNN(
            model_path="/home/ucar/ucar_ws_copy/src/yolo/scripts/rel_model.rknn",
            target_size=640,
            conf_threshold=0.65,
            nms_threshold=0.45
        )
        
        rospy.set_param('/detect', 1)  # 显式初始化参数
        self.detector.class_names = class_names
        self.bridge = CvBridge()  # OpenCV与ROS的消息转换类
        self.img_pub = rospy.Publisher('/yolo/detect', Float64MultiArray, queue_size=10)
        self.im_sub = rospy.Subscriber('/ucar_camera/image_raw', Image, self.detectimg)


    def detectimg(self, img):
        frame = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        bboxes, scores, class_ids = self.detector.inference(frame)

        # 遍历检测结果
        all_detections = []
        #这段循环来遍历一帧里所有检测结果和置信度
        for box, score, class_id in zip(bboxes, scores, class_ids):
            if score > 0.65:  # 置信度阈值
                x1, y1, x2, y2 = box
                all_detections.append({
                    'class_id': int(class_id),
                    'class_name': class_names[int(class_id)],
                    'bbox': [float(x1), float(y1), float(x2), float(y2)],
                    'score': float(score)
                })

        value = rospy.get_param('detect')
        if value == -1:
            rospy.loginfo("yolokill")
            rospy.signal_shutdown("Received stop signal")
        else :
            if not all_detections:
                rospy.loginfo("No object detected in this frame2")
            else:
                output_data = []
                # print("all_detections:", all_detections)
                for detection in all_detections:
                    output_data.extend([
                        float(detection['class_id']),
                        detection['bbox'][0],  # xl
                        detection['bbox'][1],  # yl
                        detection['bbox'][2],  # xr
                        detection['bbox'][3]   # yr
                    ])

                msg = Float64MultiArray(data=output_data)
                self.img_pub.publish(msg)
                return
            

if __name__ == '__main__':
    rospy.init_node("yolo_rknn_detect")
    rospy.loginfo("Starting YOLO RKNN detect node started")
    rospy.set_param('/detect', 1)
    detector = YOLOv5RknnDetect()
    rospy.spin()
