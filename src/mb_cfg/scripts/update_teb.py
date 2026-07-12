#! /usr/bin/env python


# 当 rosparam set /teb/update_flag true 时，进行参数的替换

"""
方案 C：轮询参数 ~/update_flag，置 true 时刷新 TEB 动态参数
"""

import os, yaml, rospy
from dynamic_reconfigure.client import Client

def load_yaml(path):
    if not os.path.isfile(path):
        rospy.logerr("YAML 文件不存在: %s", path)
        return {}
    with open(path, "r") as f:
        rospy.loginfo("I open it")
        return yaml.safe_load(f) or {}

def update_teb(cfg):
    try:
        client = Client("/move_base/TebLocalPlannerROS", timeout=5.0)
        res    = client.update_configuration(cfg)
        rospy.loginfo("TEB 参数已刷新:\n%s", res)
    except rospy.ROSException as e:
        rospy.logerr("刷新失败: %s", e)

def main():
    rospy.init_node("update_teb_params")

    # --- 配置文件路径 ---
    yaml_path = "/home/ucar/ucar_ws_copy/src/ucar_nav/launch/config/xys/teb_local_planner_params.yaml"
    raw_cfg   = load_yaml(yaml_path)
    # 只保留 dynamic_reconfigure 支持的标量
    cfg = {k: v for k, v in raw_cfg.items()
           if isinstance(v, (bool, int, float, str))}

    # --- 待监控的参数名 ---
    flag_param = rospy.get_param("/teb/update_flag")

    rospy.loginfo("The value is %d ", flag_param)

    while not rospy.is_shutdown():
        try:
            if flag_param:
                update_teb(cfg)
                # rospy.set_param(flag_param, False)  # 复位
                rospy.loginfo("flag: %d",flag_param)
        except KeyError:
            # 参数还未创建时 get_param 会抛 KeyError
            pass
        rate = rospy.Rate(0.5)
        rate.sleep()

if __name__ == "__main__":
    main()
