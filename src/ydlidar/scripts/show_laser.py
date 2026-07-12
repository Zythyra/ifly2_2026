#!/usr/bin/env python3

import rospy
import numpy as np
import os
import time
from sensor_msgs.msg import LaserScan

last_save_time = 0
save_interval = 5  # 每 5 秒保存一次
save_dir = os.path.expanduser("/home/ucar/ucar_ws_copy/src/ydlidar/laser_msg")  # 保存到 文件夹

if not os.path.exists(save_dir):
    os.makedirs(save_dir)

def scan_callback(scan):
    global last_save_time

    current_time = time.time()
    if current_time - last_save_time >= save_interval:
        last_save_time = current_time

        # 构造保存路径
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(current_time))
        filename = os.path.join(save_dir, f"scan_{timestamp}.npy")

        # 保存 ranges 数据
        ranges = np.array(scan.ranges)
        np.save(filename, ranges)
        rospy.loginfo(f"Saved scan data to {filename}")

def main():
    rospy.init_node("laser_save_node")
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
