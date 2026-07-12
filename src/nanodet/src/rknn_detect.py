#!/home/ucar/miniforge3/envs/rknn_env/bin/python3
# -*- coding: UTF-8 -*-
# 2025_07_20

import os

os.environ['ROSCONSOLE_CONFIG_FILE'] = '/dev/null'  # 禁用ROS日志配置文件
os.environ['ROSCONSOLE_FORMAT'] = '{message}'      # 设置日志格式只显示消息部分
os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = ''      # 清空Python日志配置文件路径
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rknn import NanoDetRKNN  # 假设 NanoDetRKNN 类在 nanodet_rknn.py 文件中

# 定义类别名称和对应的编号
class_names = ['Watermelon', 'Cake', 'Apple', 'Banana', 'Chili', 'Tomato', 'Milk', 'Cola', 'Potato']

# 创建一个字典，将类别名称映射到对应的编号
class_mapping = {name: i for i, name in enumerate(class_names)}

class NanoDetRknnDetect:
    def __init__(self):
        self.detector = NanoDetRKNN(
            model_path="/home/ucar/ucar_ws_copy/src/nanodet/src/rel_model.rknn",
            target_size=320,
            conf_threshold=0.65,  # 与原代码保持一致
            nms_threshold=0.45,
            reg_max=7,
            pre_nms_topk=1000
        )
        rospy.set_param('/detect', 1)  # 显式初始化参数
        self.bridge = CvBridge()  # OpenCV与ROS的消息转换类
        self.img_pub = rospy.Publisher('/nanodet/detect', Float64MultiArray, queue_size=10)
        im_sub = rospy.Subscriber('/ucar_camera/image_raw', Image, self.detectimg)

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
            rospy.loginfo("nanokill")
            rospy.signal_shutdown("Received stop signal")
        else :
            if not all_detections:
                rospy.loginfo("No object detected in this frame2")
            else:
                output_data = []
                for detection in all_detections:
                    output_data.extend([
                        float(detection['class_id']),
                        detection['bbox'][0],  # xl
                        detection['bbox'][1],  # yl
                        detection['bbox'][2],  # xr
                        detection['bbox'][3]   # yr
                    ])
                
                # 发布 Float64MultiArray
                msg = Float64MultiArray(data=output_data)
                self.img_pub.publish(msg)
                return



if __name__ == '__main__':
    rospy.init_node("nanodet_rknn_detect")
    rospy.loginfo("NanoDet RKNN detect node started")
    rospy.set_param('/detect', 1)
    detector = NanoDetRknnDetect()
    rospy.spin()