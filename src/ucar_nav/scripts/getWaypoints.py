#!/usr/bin/env python3

import rospy
import os
from geometry_msgs.msg import PoseStamped

# 设置存储 waypoints 的目录
WAYPOINTS_DIR = os.path.expanduser("src/ucar_nav/waypoints")  # 存放在 ~/waypoints
if not os.path.exists(WAYPOINTS_DIR):
    os.makedirs(WAYPOINTS_DIR)

waypoint_count = 0  # 计数器，用于生成文件名

def goal_callback(msg):
    global waypoint_count
    filename = os.path.join(WAYPOINTS_DIR, f"waypoint_{waypoint_count}.txt")
    
    with open(filename, "w") as f:
        f.write(f"frame_id: {msg.header.frame_id}\n")
        f.write(f"timestamp: {msg.header.stamp.to_sec()}\n")
        f.write(f"position: [{msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}]\n")
        f.write(f"orientation: [{msg.pose.orientation.x}, {msg.pose.orientation.y}, {msg.pose.orientation.z}, {msg.pose.orientation.w}]\n")

    rospy.loginfo(f"Saved waypoint {waypoint_count} to {filename}")
    waypoint_count += 1  # 递增计数器

def main():
    rospy.init_node("waypoints_saver", anonymous=True)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)
    rospy.loginfo("Waiting for /move_base_simple/goal messages...")
    rospy.spin()

if __name__ == "__main__":
    main()
