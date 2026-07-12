#!/home/ucar/miniforge3/envs/opencv_env/bin/python
# coding:utf-8
from pyzbar.pyzbar import decode
from PIL import Image
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from cv_bridge import CvBridge

def Identifying_QR_codes(img):  # 识别二维码
    # 将图像转换为灰度
    img = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    # 识别图像中的二维码
    barcodes = decode(img)
    # 输出识别结果
    for barcode in barcodes:
        barcodeData = barcode.data.decode("utf-8")
        print(barcodeData)  # 5/29 被xu注释
        ewm_pub.publish(barcodeData)


image = rospy.Subscriber('/ucar_camera/image_raw', Image, Identifying_QR_codes)
ewm_pub = rospy.Publisher('ewm', String, queue_size=10)
bridge = CvBridge()
rospy.init_node("ewm")
rospy.loginfo("ewm node started")
rospy.spin()