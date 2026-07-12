#!/home/ucar/miniforge3/envs/rknn_env/bin/python3
# -*- coding: UTF-8 -*-
# 2025_07_20
import os

os.environ['ROSCONSOLE_CONFIG_FILE'] = '/dev/null'  # 禁用ROS日志配置文件
os.environ['ROSCONSOLE_FORMAT'] = '{message}'      # 设置日志格式只显示消息部分
os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = ''      # 清空Python日志配置文件路径
import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rknn2 import NanoDetRKNN  # 假设 NanoDetRKNN 类在 nanodet_rknn.py 文件中


# 定义类别名称和对应的编号
class_names = ['Red', 'Green']

class NanoDetRknnDetect:
    def __init__(self):
        self.detector = NanoDetRKNN(
            model_path="/home/ucar/ucar_ws_copy/src/nanodet/src/rgl_model.rknn",
            target_size=320,
            conf_threshold=0.65,  # 与原代码保持一致
            nms_threshold=0.45,
            reg_max=7,
            pre_nms_topk=1000
        )
        self.bridge = CvBridge()
        # rospy.set_param('detect2', 0)
        self.sub = rospy.Subscriber('/ucar_camera/image_raw', Image, self.detectimg, queue_size=1)

    def detectimg(self, img):
        frame = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        bboxes, scores, class_ids = self.detector.inference(frame)
        
        # 遍历检测结果
        all_detections = []
        for box, score, class_id in zip(bboxes, scores, class_ids):
            if score > 0.3:  # 置信度阈值
                x1, y1, x2, y2 = box
                all_detections.append({
                    'class_id': int(class_id),
                    'class_name': class_names[int(class_id)],
                    'bbox': [float(x1), float(y1), float(x2), float(y2)],
                    'score': float(score)
                })

        # 找出最左侧的物体
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
        rospy.loginfo("节点资源已清理")

if __name__ == '__main__':
    rospy.init_node("nanodet_rknn_detect2")
    rospy.loginfo("NanoDet RKNN detect2 node started")
    # rospy.set_param('detect2', 0)
    detector = NanoDetRknnDetect()
    rospy.spin()