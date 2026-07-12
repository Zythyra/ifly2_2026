#! /home/ucar/miniforge3/envs/opencv_env/bin/python
# -*- coding: UTF-8 -*-

import rospy
import rospkg
import sys
import os
import rospkg
# 导入你自己的包名和消息名
rospack = rospkg.RosPack()
msg_path = rospack.get_path('nanodet') + '/devel/lib/python3/dist-packages'
sys.path.insert(0, msg_path)
from nanodet.msg import IDStamp



def talker():
    # 1. 初始化节点
    rospy.init_node('my_stamped_array_publisher', anonymous=True)

    # 2. 创建一个Publisher，发布话题为'stamped_array_topic'，消息类型为MyStampedArray
    pub = rospy.Publisher('stamped_array_topic', IDStamp, queue_size=10)

    # 3. 设置发布频率为 1Hz
    rate = rospy.Rate(1) 

    # 4. 初始化一个序列号计数器
    seq_counter = 0

    rospy.loginfo("Publisher node started. Publishing to 'stamped_array_topic'...")

    while not rospy.is_shutdown():
        # 创建一个消息实例
        msg = IDStamp()

        # --- 填充Header字段 ---
        msg.header.seq = seq_counter
        msg.header.stamp = rospy.Time.now() # 使用当前时间作为时间戳
        msg.header.frame_id = 'base_link'   # 举例，设置一个坐标系ID

        # --- 填充Data字段 ---
        # Float64MultiArray本身有一个名为'data'的属性，它是一个Python列表
        current_time_float = rospy.Time.now().to_sec()
        msg.data.data = [1.23, 4.56, 7.89, current_time_float]

        # 打印将要发送的消息内容
        rospy.loginfo("\n--- Publishing Message ---")
        rospy.loginfo("Header:")
        rospy.loginfo("  seq: %d", msg.header.seq)
        rospy.loginfo("  stamp: %s", msg.header.stamp)
        rospy.loginfo("  frame_id: %s", msg.header.frame_id)
        rospy.loginfo("Data: %s", msg.data.data)
        
        # 发布消息
        pub.publish(msg)

        # 计数器自增
        seq_counter += 1

        # 按照设定的频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass