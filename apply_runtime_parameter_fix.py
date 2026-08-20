#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
给当前 rk3588 / my_planner 打运行时参数修复补丁。

只修复：
  /move_base/MyPlanner/clearance_optimizer/enabled

使它和以下现有参数一样，能在 MyPlanner 控制循环运行期间真正动态生效：
  /move_base/MyPlanner/enable_path_replanning
  /move_base/MyPlanner/c2_max_reference_speed
  /move_base/MyPlanner/mpc_max_vx
  /move_base/MyPlanner/mpc_max_translational_speed
  /move_base/MyPlanner/max_vel_x

不会修改其他 MPC / Patrol-R2 / C5 几何逻辑。

用法：
  python3 apply_runtime_parameter_fix.py \
      /home/ucar/ucar_ws_copy/src/my_planner
"""

from pathlib import Path
import shutil
import sys
import time


class PatchError(RuntimeError):
    pass


def replace_once(text, old, new, label):
    count = text.count(old)
    if count == 0:
        raise PatchError("没有找到补丁位置：{}".format(label))
    if count > 1:
        raise PatchError(
            "补丁位置 [{}] 出现 {} 次，拒绝盲目修改".format(label, count)
        )
    return text.replace(old, new, 1)


def main():
    if len(sys.argv) >= 2:
        package_dir = Path(sys.argv[1]).resolve()
    else:
        package_dir = Path(
            "/home/ucar/ucar_ws_copy/src/my_planner"
        ).resolve()

    header_path = package_dir / "include" / "clearance_path_optimizer.h"
    optimizer_cpp_path = package_dir / "src" / "clearance_path_optimizer.cpp"
    planner_cpp_path = package_dir / "src" / "my_planner.cpp"

    for path in (header_path, optimizer_cpp_path, planner_cpp_path):
        if not path.is_file():
            raise PatchError("找不到文件：{}".format(path))

    header = header_path.read_text(encoding="utf-8")
    optimizer_cpp = optimizer_cpp_path.read_text(encoding="utf-8")
    planner_cpp = planner_cpp_path.read_text(encoding="utf-8")

    # ------------------------------------------------------------
    # 版本/结构保护：避免补丁误打到旧版规划器
    # ------------------------------------------------------------
    required_planner_markers = [
        "void MyPlanner::refreshRuntimeParameters()",
        'private_nh_.getParamCached(\n        "enable_path_replanning"',
        'private_nh_.getParamCached(\n        "c2_max_reference_speed"',
        'private_nh_.getParamCached(\n        "mpc_max_vx"',
        'private_nh_.getParamCached(\n        "mpc_max_translational_speed"',
        'private_nh_.getParamCached(\n        "max_vel_x"',
        "refreshRuntimeParameters();",
        "Patrol-R2",
    ]
    for marker in required_planner_markers:
        if marker not in planner_cpp:
            raise PatchError(
                "当前 my_planner.cpp 缺少预期标记：{}\n"
                "说明工作空间不是当前 Patrol-R2/C5.4 基线，未做任何修改。"
                .format(marker)
            )

    if "bool ClearancePathOptimizer::enabled() const" not in optimizer_cpp:
        raise PatchError(
            "clearance_path_optimizer.cpp 中找不到 enabled()"
        )

    # 若已经打过本补丁，则只做检查，不重复插入。
    already_patched = (
        "void setEnabled(bool enabled);" in header
        and "void ClearancePathOptimizer::setEnabled(bool enabled)"
            in optimizer_cpp
        and '"clearance_optimizer/enabled"' in planner_cpp[
            planner_cpp.find("void MyPlanner::refreshRuntimeParameters()"):
            planner_cpp.find("void MyPlanner::updateHealedPath()")
        ]
    )

    if already_patched:
        print("检测到运行时 clearance_optimizer/enabled 补丁已经存在。")
        print("未重复修改。")
        return

    # ------------------------------------------------------------
    # 1. ClearancePathOptimizer 公开运行时开关
    # ------------------------------------------------------------
    header = replace_once(
        header,
        """    void reset();
    bool enabled() const;
""",
        """    void reset();
    bool enabled() const;

    // 运行时开关。用于比赛流程中临时完全旁路 C5。
    // 状态变化时自动 reset，避免重新开启后复用旧 active path。
    void setEnabled(bool enabled);
""",
        "clearance_path_optimizer.h: enabled接口",
    )

    optimizer_cpp = replace_once(
        optimizer_cpp,
        """bool ClearancePathOptimizer::enabled() const
{
    return initialized_ && config_.enabled;
}
""",
        """bool ClearancePathOptimizer::enabled() const
{
    return initialized_ && config_.enabled;
}

void ClearancePathOptimizer::setEnabled(bool enabled)
{
    if (config_.enabled == enabled)
        return;

    config_.enabled = enabled;

    // 切换时清掉 warm-start / active path 内部状态。
    // reset() 按当前 C5.4 设计不会发布空 Path，
    // 所以 RViz 可能仍保留上一条 optimized_path；
    // 但控制器从本周期开始已经不会再使用它。
    reset();

    ROS_WARN(
        "C5.4 clearance_optimizer 运行时已%s；"
        "%s",
        config_.enabled ? "开启" : "关闭",
        config_.enabled
            ? "后续控制周期重新从当前Reference建立active path。"
            : "后续控制周期完全旁路C5，直接保留当前raw/Patrol Reference。");
}
""",
        "clearance_path_optimizer.cpp: enabled实现",
    )

    # ------------------------------------------------------------
    # 2. MyPlanner::refreshRuntimeParameters()
    #    增加 clearance_optimizer/enabled 的每周期读取
    # ------------------------------------------------------------
    planner_cpp = replace_once(
        planner_cpp,
        """    bool requested_enable_path_replanning =
        enable_path_replanning_;
""",
        """    bool requested_clearance_optimizer_enabled =
        clearance_path_optimizer_.enabled();
    bool requested_enable_path_replanning =
        enable_path_replanning_;
""",
        "my_planner.cpp: runtime变量",
    )

    planner_cpp = replace_once(
        planner_cpp,
        """    private_nh_.getParamCached(
        "enable_path_replanning",
        requested_enable_path_replanning);
""",
        """    private_nh_.getParamCached(
        "clearance_optimizer/enabled",
        requested_clearance_optimizer_enabled);
    private_nh_.getParamCached(
        "enable_path_replanning",
        requested_enable_path_replanning);
""",
        "my_planner.cpp: runtime参数读取",
    )

    planner_cpp = replace_once(
        planner_cpp,
        """    if (requested_enable_path_replanning
        != enable_path_replanning_)
""",
        """    if (requested_clearance_optimizer_enabled
        != clearance_path_optimizer_.enabled())
    {
        clearance_path_optimizer_.setEnabled(
            requested_clearance_optimizer_enabled);

        // C5状态切换后，旧的失败/重规划确认不能带到新模式。
        c5_failure_replan_confirm_counter_ = 0;
        c5_failure_replan_candidate_ = false;
        c5_failure_replan_reason_.clear();

        ROS_WARN(
            "运行时 clearance_optimizer/enabled 已真正切换为 %s。",
            clearance_path_optimizer_.enabled()
                ? "true" : "false");
    }

    if (requested_enable_path_replanning
        != enable_path_replanning_)
""",
        "my_planner.cpp: runtime C5切换",
    )

    # ------------------------------------------------------------
    # 3. 最终静态检查
    # ------------------------------------------------------------
    checks = {
        "header setEnabled声明":
            "void setEnabled(bool enabled);" in header,
        "optimizer setEnabled实现":
            "void ClearancePathOptimizer::setEnabled(bool enabled)"
            in optimizer_cpp,
        "C5运行时参数读取":
            '"clearance_optimizer/enabled"' in planner_cpp,
        "原enable_path_replanning动态读取保留":
            '"enable_path_replanning"' in planner_cpp,
        "原c2速度动态读取保留":
            '"c2_max_reference_speed"' in planner_cpp,
        "原mpc vx动态读取保留":
            '"mpc_max_vx"' in planner_cpp,
        "原mpc总平移速度动态读取保留":
            '"mpc_max_translational_speed"' in planner_cpp,
        "原max_vel_x动态读取保留":
            '"max_vel_x"' in planner_cpp,
        "每控制周期刷新保留":
            "refreshRuntimeParameters();" in planner_cpp,
    }

    failed = [name for name, ok in checks.items() if not ok]
    if failed:
        raise PatchError(
            "补丁后静态检查失败：{}".format(", ".join(failed))
        )

    # ------------------------------------------------------------
    # 4. 先备份，再写入
    # ------------------------------------------------------------
    stamp = time.strftime("%Y%m%d_%H%M%S")
    backups = []

    for path in (header_path, optimizer_cpp_path, planner_cpp_path):
        backup = path.with_name(
            path.name + ".bak_runtime_params_" + stamp
        )
        shutil.copy2(path, backup)
        backups.append(backup)

    header_path.write_text(header, encoding="utf-8")
    optimizer_cpp_path.write_text(optimizer_cpp, encoding="utf-8")
    planner_cpp_path.write_text(planner_cpp, encoding="utf-8")

    print("")
    print("运行时参数补丁完成。")
    print("已修改：")
    print("  {}".format(header_path))
    print("  {}".format(optimizer_cpp_path))
    print("  {}".format(planner_cpp_path))
    print("")
    print("备份：")
    for backup in backups:
        print("  {}".format(backup))

    print("")
    print("现在以下6项均可在运行时真正生效：")
    print("  /move_base/MyPlanner/clearance_optimizer/enabled")
    print("  /move_base/MyPlanner/enable_path_replanning")
    print("  /move_base/MyPlanner/c2_max_reference_speed")
    print("  /move_base/MyPlanner/mpc_max_vx")
    print("  /move_base/MyPlanner/mpc_max_translational_speed")
    print("  /move_base/MyPlanner/max_vel_x")
    print("")
    print("接下来重新编译：")
    print("  cd /home/ucar/ucar_ws_copy")
    print("  source /opt/ros/noetic/setup.bash")
    print("  catkin_make --force-cmake --pkg my_planner -j4")
    print("  source devel/setup.bash")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("补丁失败：{}".format(exc), file=sys.stderr)
        sys.exit(1)
