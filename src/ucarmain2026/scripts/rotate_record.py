#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# 视觉数据集采集工具：原地旋转 360 度并录制单份视频 (含硬排空与画面翻转)

import rospy
from geometry_msgs.msg import Twist
import cv2
import math
import time
import os

# 固定覆盖的视频保存路径
VIDEO_PATH = "/home/ucar/ucar_car/src/ucarmain2026/videos_for_yolo/scan_record.avi"

# 旋转控制参数
ANGULAR_SPEED = 0.5    # 旋转角速度 (rad/s)
TARGET_ANGLE = 2 * math.pi  # 目标角度 (360度 = 2π)

def main():
    # 初始化 ROS 节点和控制发布者
    rospy.init_node('rotate_record_node', anonymous=True)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(20) # 20Hz 发送控制指令

    # 确保存放视频的文件夹存在
    save_dir = os.path.dirname(VIDEO_PATH)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # 打开底层物理摄像头
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        rospy.logerr("❌ 无法打开摄像头！请检查占用情况。")
        return

    # ---------------- 核心补丁 1：清空初始硬件缓存与暖机 ----------------
    rospy.loginfo("⏳ 正在清空底层图像缓存并等待自动曝光收敛...")
    for _ in range(15):  
        cap.grab()
    time.sleep(0.1)
    rospy.loginfo("✅ 缓存已排空，画面就绪！")
    # ------------------------------------------------------------------

    # 设置视频录制器 (使用 XVID 编码，兼容性极好)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(VIDEO_PATH, fourcc, 20.0, (640, 480))

    rospy.loginfo("========================================")
    rospy.loginfo("🟢 360度全景录像程序启动！")
    rospy.loginfo(f"💾 视频将覆盖保存至: {VIDEO_PATH}")
    rospy.loginfo("========================================")

    # 计算大概需要旋转的时间： t = 距离 / 速度 + 0.5秒余量
    duration = (TARGET_ANGLE / ANGULAR_SPEED) + 0.5 
    
    twist_cmd = Twist()
    twist_cmd.angular.z = ANGULAR_SPEED

    start_time = rospy.Time.now().to_sec()
    
    # 核心循环：边发速度，边录视频
    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        elapsed = current_time - start_time
        
        # 抓取画面并写入视频文件
        ret, frame = cap.read()
        if ret:
            # ---------------- 核心补丁 2：翻转画面，回归物理现实方向 ----------------
            frame = cv2.flip(frame, 1)
            # ------------------------------------------------------------------------
            out.write(frame)

        if elapsed >= duration:
            rospy.loginfo("🏁 360度旋转完成，正在保存视频...")
            break

        # 持续发送旋转指令
        cmd_pub.publish(twist_cmd)
        rate.sleep()

    # 刹车，停止底盘
    stop_cmd = Twist()
    cmd_pub.publish(stop_cmd)

    # 释放所有硬件资源并封顶视频文件
    cap.release()
    out.release()
    rospy.loginfo("✅ 视频已成功保存！程序退出。")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass