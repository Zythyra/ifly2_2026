#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class WaypointNavigator:
    def __init__(self):
        rospy.init_node("waypoint_navigator")
        
        # 连接 move_base action server
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base!")

        # 预设路径点（单位：米，坐标系：map）
        self.waypoints = [
            [(4.282443714141846, -0.7489101254940033), 2.0],
            [(6.373447456359863, -0.363661561012268), 1.57],
            [(4.617380142211914, -0.000820284366608), 3.14],
            [(2.282443714141846, -0.2489101254940033), 2.0]
        ]

    def send_goal(self, x, y, yaw):
        """ 发送目标点到 move_base """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # 目标点是基于 "map" 参考系的
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = yaw  # 仅仅简化成 yaw（不考虑完整的四元数）

        rospy.loginfo(f"Sending goal: x={x}, y={y}, yaw={yaw}")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached!")
        else:
            rospy.logwarn("Failed to reach goal!")

    def navigate(self):
        """ 遍历所有路径点 """
        for point, yaw in self.waypoints:
            x, y = point
            self.send_goal(x, y, yaw)
            rospy.sleep(1)  # 稍作停顿后继续下一个目标点

if __name__ == "__main__":
    navigator = WaypointNavigator()
    navigator.navigate()
