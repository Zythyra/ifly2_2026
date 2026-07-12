#！/usr/bin/env python3
import rospy

rospy.init_node("param_reader")

# 读取单个参数（如果参数不存在，则使用默认值）
param_value = rospy.get_param("/point1/x_y_yaw", "default_value")

print(param_value[0])
rospy.set_param("/point1/x_y_yaw", [2.33333, 3.12313, 313411])
param_value = rospy.get_param("/point1/x_y_yaw", "default_value")
print(param_value[0])

# 读取嵌套参数（字典）
# param_dict = rospy.get_param("/waypoints", {})

print(f"Parameter Value: {param_value}")
# print(f"Waypoint Data: {param_dict}")
