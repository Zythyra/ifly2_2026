#! /usr/bin/env python


# ros::param::set("/detect", 1);
# ros::param::set("/detect", -1);


#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
flag_controlled_publisher_param.py

flag =  1 → 开始发布
flag =  0 → 暂停发布
flag = -1 → 立即关闭节点
flag 参数存放在参数服务器，键名缺省为 /flag
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
flag_controlled_publisher_param_edge.py

参数服务器键: /flag
    1  → 立即发布一次消息
    0  → 待命，不发布
   -1  → 关闭节点
"""

import rospy
from std_msgs.msg import String


class FlagControlledPublisherParamEdge:
    def __init__(self):
        # 频率 & 话题均可通过私有参数覆盖
        self.loop_hz  = rospy.get_param("~loop_hz", 10)          # 主循环频率
        self.topic    = rospy.get_param("~topic",    "/demo")    # 发布话题
        self.flag_key = rospy.get_param("~flag_key", "/flag")    # 参数键名

        # 若 /flag 未设定，则初始化为 0
        if not rospy.has_param(self.flag_key):
            rospy.set_param(self.flag_key, 0)

        self.pub = rospy.Publisher(self.topic, String, queue_size=10)

        # 记录上一次的 flag，用来检测“是否变化为 1”
        self.last_flag = rospy.get_param(self.flag_key)

    def run(self):
        rate = rospy.Rate(self.loop_hz)
        while not rospy.is_shutdown():
            flag = rospy.get_param(self.flag_key, 0)

            # 检测到 flag 变化
            if flag != self.last_flag:
                rospy.loginfo(f"flag 由 {self.last_flag} → {flag}")

            # 触发条件：上一次不是 1，当前是 1
            while self.last_flag != 1 and flag == 1:
                msg = String(data="hello, world")
                self.pub.publish(msg)
                rospy.loginfo("已发布一次消息")

                # ★ 如果想在 flag==1 时持续发布，替换为：
                # while flag == 1 and not rospy.is_shutdown():
                #     self.pub.publish(msg)
                #     rate.sleep()
                #     flag = rospy.get_param(self.flag_key, 0)

            # 检测退出
            if flag == -1:
                rospy.logwarn("检测到 flag==-1，节点关闭")
                rospy.signal_shutdown("flag == -1")
                break

            self.last_flag = flag
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("flag_controlled_publisher_param_edge")
    node = FlagControlledPublisherParamEdge()
    node.run()
