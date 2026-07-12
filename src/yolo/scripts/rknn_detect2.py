#!/home/ucar/miniforge3/envs/rknn_env/bin/python3
# -*- coding: UTF-8 -*-
import os
os.environ['ROSCONSOLE_CONFIG_FILE'] = '/dev/null'
os.environ['ROSCONSOLE_FORMAT'] = '{message}'
os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = ''

import rospy
import numpy as np
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rknn2 import YOLOv5RKNN

# 定义类别名称和对应的编号
class_names = ['Red', 'Green']

class YOLOv5RknnDetect:
    def __init__(self):
        # 初始化YOLOv5检测器
        self.detector = YOLOv5RKNN(
            model_path="/home/ucar/ucar_ws_copy/src/yolo/scripts/rgl_model.rknn",
            target_size=640,
            conf_threshold=0.65,
            nms_threshold=0.45
        )
        
        # 设置类别名称（覆盖默认值）
        self.detector.class_names = class_names
        self.bridge = CvBridge()  # OpenCV与ROS的消息转换类
        rospy.set_param('detect2', 0)  # 初始化检测标志位
        self.sub = rospy.Subscriber('/ucar_camera/image_raw', Image, self.detectimg, queue_size=1)
    
    def detectimg(self, img_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        bboxes, scores, class_ids = self.detector.inference(frame)
        
        all_detections = []
        for box, score, cls_id in zip(bboxes, scores, class_ids):
            if score > 0.3:  # 置信度阈值
                x1, y1, x2, y2 = box
                all_detections.append({
                    'class_id': int(cls_id),
                    'class_name': class_names[int(cls_id)],
                    'bbox': [float(x1), float(y1), float(x2), float(y2)],
                    'score': float(score)
                })
        
        # 找出最左侧的红绿灯
        value = rospy.get_param('detect2')
        if value == -1:
            rospy.signal_shutdown("Received stop signal")
        if value == 1:
            if not all_detections:
                rospy.loginfo("No object detected in this frame2")
            else:
                leftmost_obj = min(all_detections, key=lambda x: x['bbox'][0])
                rospy.set_param('light', leftmost_obj['class_id'] + 1)
                rospy.loginfo(f"Leftmost object: {leftmost_obj['class_name']} (ID: {leftmost_obj['class_id']})")
            


    def __del__(self):
        if hasattr(self, 'detector'):
            del self.detector
        rospy.loginfo("YOLOv5 detector resources cleaned up")

if __name__ == '__main__':
    rospy.init_node("yolo_rknn_detect2")
    rospy.loginfo("Starting YOLO RKNN detect2 node started")
    rospy.set_param('detect2', 0) 
    detector = YOLOv5RknnDetect()
    rospy.spin()