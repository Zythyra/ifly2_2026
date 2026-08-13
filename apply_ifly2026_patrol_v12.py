#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IFLY2026 Patrol V12 patcher

基线：V11 target_patrol_docking 固定Path巡检版。
只修改：
  1) target_patrol_docking.cpp
  2) target_patrol_docking.launch
不修改 MyPlanner / C5.4 / my_planner_params.yaml。

V12巡检几何：
  初始move_base仍到 (0.25, 4.25, 0deg)
  -> 纯全向P/PP安全横移到 (0.25, 4.30, 0deg)
  -> 上墙巡检 y=4.30: (0.25,4.30) -> (4.75,4.30)
  -> 角点过渡: (4.80,4.30,-90deg)
  -> 右墙巡检 x=4.80: (4.80,4.30) -> (4.80,2.75)
  -> 角点过渡: (4.80,2.70,-180deg)
  -> 下墙巡检 y=2.70: (4.80,2.70) -> (0.25,2.70)
  -> 角点过渡: (0.20,2.70,+90deg)
  -> 左墙巡检 x=0.20: (0.20,2.70) -> (0.20,4.25)

角点过渡不走move_base，不锁巡检Path；直接用/set_speed做map坐标XY+yaw同时P闭环，
并带速度下限、加速度限幅、稳定帧与超时保护。绝对目标使重复调用幂等。
"""

import argparse
import re
import shutil
import sys
from pathlib import Path

V11_MARKER = "IFLY2026_FIXED_PATH_PATROL_V11_PRECISE_CORNER_DEADZONE_FIX_20260808"
V12_MARKER = "IFLY2026_FIXED_PATH_PATROL_V12_SAFE_OFFSET_CORNER_TRANSITION_20260810"


class PatchError(RuntimeError):
    pass


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def backup_and_write(path: Path, text: str, dry_run: bool) -> None:
    if dry_run:
        return
    backup = Path(str(path) + ".bak_v11")
    if not backup.exists():
        shutil.copy2(path, backup)
    tmp = Path(str(path) + ".v12.tmp")
    tmp.write_text(text, encoding="utf-8")
    tmp.replace(path)


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise PatchError(f"{label}: 预期匹配1处，实际{count}处")
    return text.replace(old, new, 1)


def regex_replace_once(text: str, pattern: str, repl: str,
                       label: str, flags: int = 0) -> str:
    out, count = re.subn(pattern, repl, text, count=1, flags=flags)
    if count != 1:
        raise PatchError(f"{label}: 预期匹配1处，实际{count}处")
    return out


def find_files(root: Path):
    return [p for p in root.rglob("*") if p.is_file()]


def locate_files(workspace: Path):
    src = workspace / "src"
    if not src.is_dir():
        raise PatchError(f"未找到工作空间src目录：{src}")

    files = find_files(src)
    cpp = []
    launch = []
    already_v12 = []

    for p in files:
        if p.suffix not in {".cpp", ".launch", ".xml"}:
            continue
        try:
            t = read_text(p)
        except (UnicodeDecodeError, OSError):
            continue
        if V12_MARKER in t and "target_patrol_docking" in t:
            already_v12.append(p)
        if (p.suffix == ".cpp" and V11_MARKER in t and
                "class TargetPatrolDocking" in t and
                "buildSegments()" in t and
                "patrol_path_publisher_" in t):
            cpp.append(p)
        if (p.suffix in {".launch", ".xml"} and V11_MARKER in t and
                "target_patrol_docking" in t and
                "segment_end_tolerance" in t):
            launch.append(p)

    if already_v12 and not cpp:
        raise PatchError(
            "检测到V12已经安装：\n  - " +
            "\n  - ".join(str(p) for p in already_v12[:10]))

    def prefer(items, name):
        exact = [p for p in items if p.name == name]
        if len(exact) == 1:
            return exact[0]
        if len(items) == 1:
            return items[0]
        raise PatchError(
            f"无法唯一定位{name}，候选{len(items)}个：\n  - " +
            "\n  - ".join(str(p) for p in items[:20]))

    return prefer(cpp, "target_patrol_docking.cpp"), \
           prefer(launch, "target_patrol_docking.launch")


CTOR_PARAMS = r'''        # V12：角点安全偏移与同时转向/平移控制参数。
        pnh_.param("patrol_transition_position_kp",
                   patrol_transition_position_kp_, 2.50);
        pnh_.param("patrol_transition_yaw_kp",
                   patrol_transition_yaw_kp_, 2.00);
        pnh_.param("patrol_transition_min_linear_speed",
                   patrol_transition_min_linear_speed_, 0.025);
        pnh_.param("patrol_transition_max_linear_speed",
                   patrol_transition_max_linear_speed_, 0.12);
        pnh_.param("patrol_transition_min_angular_speed",
                   patrol_transition_min_angular_speed_, 0.12);
        pnh_.param("patrol_transition_max_angular_speed",
                   patrol_transition_max_angular_speed_, 0.65);
        pnh_.param("patrol_transition_linear_accel",
                   patrol_transition_linear_accel_, 0.60);
        pnh_.param("patrol_transition_angular_accel",
                   patrol_transition_angular_accel_, 1.50);
        pnh_.param("patrol_transition_position_tolerance",
                   patrol_transition_position_tolerance_, 0.012);
        pnh_.param("patrol_transition_yaw_tolerance_deg",
                   patrol_transition_yaw_tolerance_deg_, 1.5);
        pnh_.param("patrol_transition_stable_frames",
                   patrol_transition_stable_frames_, 3);
        pnh_.param("patrol_transition_timeout",
                   patrol_transition_timeout_, 8.0);
'''

NORMALIZE_BLOCK = r'''        patrol_transition_position_kp_ =
            std::fabs(patrol_transition_position_kp_);
        patrol_transition_yaw_kp_ =
            std::fabs(patrol_transition_yaw_kp_);
        patrol_transition_min_linear_speed_ =
            std::fabs(patrol_transition_min_linear_speed_);
        patrol_transition_max_linear_speed_ = std::max(
            std::fabs(patrol_transition_max_linear_speed_),
            patrol_transition_min_linear_speed_);
        patrol_transition_min_angular_speed_ =
            std::fabs(patrol_transition_min_angular_speed_);
        patrol_transition_max_angular_speed_ = std::max(
            std::fabs(patrol_transition_max_angular_speed_),
            patrol_transition_min_angular_speed_);
        patrol_transition_linear_accel_ =
            std::max(0.05, std::fabs(patrol_transition_linear_accel_));
        patrol_transition_angular_accel_ =
            std::max(0.05, std::fabs(patrol_transition_angular_accel_));
        patrol_transition_position_tolerance_ =
            std::max(0.003, std::fabs(patrol_transition_position_tolerance_));
        patrol_transition_yaw_tolerance_deg_ =
            std::max(0.2, std::fabs(patrol_transition_yaw_tolerance_deg_));
        patrol_transition_stable_frames_ =
            std::max(1, patrol_transition_stable_frames_);
        patrol_transition_timeout_ =
            std::max(1.0, patrol_transition_timeout_);
'''

SEGMENTS_BLOCK = r'''    void buildSegments() {
        segments_.clear();
        // V12安全偏移巡检线：墙仍为原场地边界，但车体中心路线向角点安全侧
        // 额外偏移5cm，避免90度姿态调整时车头/车尾扫到墙。
        // 每段仍走到用户指定的“逻辑终点”，随后由runPatrolPoseTransition()
        // 同时完成5cm平移和90度转向，再启动下一条固定Path。
        addSegment("上墙巡检", 0.25, 4.30, 4.75, 4.30,
                   0.0, 0.5 * kPi, WALL_TOP);
        addSegment("右墙巡检", 4.80, 4.30, 4.80, 2.75,
                   -0.5 * kPi, 0.0, WALL_RIGHT);
        addSegment("下墙巡检", 4.80, 2.70, 0.25, 2.70,
                   -kPi, -0.5 * kPi, WALL_BOTTOM);
        addSegment("左墙巡检", 0.20, 2.70, 0.20, 4.25,
                   0.5 * kPi, kPi, WALL_LEFT);
    }
'''

TRANSITION_METHODS = r'''    double limitPatrolTransitionRate(double desired,
                                     double previous,
                                     double max_delta) const {
        return previous + clampValue(
            desired - previous, -max_delta, max_delta);
    }

    bool runPatrolPoseTransition(double target_x,
                                 double target_y,
                                 double target_yaw,
                                 const std::string& label) {
        if (!isInsideRoom(target_x, target_y)) {
            ROS_ERROR("%s目标(%.3f, %.3f)超出房间边界",
                      label.c_str(), target_x, target_y);
            return false;
        }

        // 角点过渡只由/set_speed直接接管。调用前固定巡检Path已经解除，
        // 不让move_base最终姿态调整在墙边原地旋转。
        stopRobot();

        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(patrol_transition_timeout_);
        ros::WallTime last_time = ros::WallTime::now();
        ros::Rate rate(std::max(10.0, control_rate_));

        double command_vx = 0.0;
        double command_vy = 0.0;
        double command_wz = 0.0;
        int stable_frames = 0;

        ROS_INFO("%s开始：目标=(%.3f, %.3f, %.1f度)，"
                 "采用全向XY+yaw同时P闭环。",
                 label.c_str(), target_x, target_y,
                 target_yaw * 180.0 / kPi);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            ros::spinOnce();

            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishVelocity(0.0, 0.0, 0.0);
                rate.sleep();
                continue;
            }

            const double world_error_x = target_x - pose.x;
            const double world_error_y = target_y - pose.y;
            const double position_error =
                std::hypot(world_error_x, world_error_y);
            const double yaw_error =
                normalizeAngle(target_yaw - pose.yaw);

            // map误差转到当前车体坐标，因此在旋转过程中仍能正确向绝对目标横移。
            const double c = std::cos(pose.yaw);
            const double s = std::sin(pose.yaw);
            const double base_error_x =
                c * world_error_x + s * world_error_y;
            const double base_error_y =
                -s * world_error_x + c * world_error_y;

            double desired_vx = 0.0;
            double desired_vy = 0.0;
            double desired_wz = 0.0;

            if (position_error > patrol_transition_position_tolerance_) {
                desired_vx = clampValue(
                    patrol_transition_position_kp_ * base_error_x,
                    -patrol_transition_max_linear_speed_,
                    patrol_transition_max_linear_speed_);
                desired_vy = clampValue(
                    patrol_transition_position_kp_ * base_error_y,
                    -patrol_transition_max_linear_speed_,
                    patrol_transition_max_linear_speed_);

                const double linear_norm =
                    std::hypot(desired_vx, desired_vy);
                if (linear_norm > 1e-9 &&
                    linear_norm < patrol_transition_min_linear_speed_) {
                    const double scale =
                        patrol_transition_min_linear_speed_ / linear_norm;
                    desired_vx *= scale;
                    desired_vy *= scale;
                }

                const double limited_norm =
                    std::hypot(desired_vx, desired_vy);
                if (limited_norm > patrol_transition_max_linear_speed_) {
                    const double scale =
                        patrol_transition_max_linear_speed_ / limited_norm;
                    desired_vx *= scale;
                    desired_vy *= scale;
                }
            }

            const double yaw_tolerance =
                patrol_transition_yaw_tolerance_deg_ * kPi / 180.0;
            if (std::fabs(yaw_error) > yaw_tolerance) {
                desired_wz = clampValue(
                    patrol_transition_yaw_kp_ * yaw_error,
                    -patrol_transition_max_angular_speed_,
                    patrol_transition_max_angular_speed_);
                if (std::fabs(desired_wz) <
                    patrol_transition_min_angular_speed_) {
                    desired_wz = std::copysign(
                        patrol_transition_min_angular_speed_, yaw_error);
                }
            }

            const ros::WallTime now = ros::WallTime::now();
            double dt = (now - last_time).toSec();
            last_time = now;
            if (!std::isfinite(dt) || dt <= 0.0) dt = 0.05;
            dt = clampValue(dt, 0.01, 0.20);

            command_vx = limitPatrolTransitionRate(
                desired_vx, command_vx,
                patrol_transition_linear_accel_ * dt);
            command_vy = limitPatrolTransitionRate(
                desired_vy, command_vy,
                patrol_transition_linear_accel_ * dt);
            command_wz = limitPatrolTransitionRate(
                desired_wz, command_wz,
                patrol_transition_angular_accel_ * dt);

            const bool position_ok =
                position_error <= patrol_transition_position_tolerance_;
            const bool yaw_ok = std::fabs(yaw_error) <= yaw_tolerance;

            if (position_ok && yaw_ok) {
                ++stable_frames;
                command_vx = 0.0;
                command_vy = 0.0;
                command_wz = 0.0;
                if (stable_frames >= patrol_transition_stable_frames_) {
                    stopRobot();
                    ROS_INFO("%s完成：当前位置=(%.3f, %.3f, %.1f度)。",
                             label.c_str(), pose.x, pose.y,
                             pose.yaw * 180.0 / kPi);
                    return true;
                }
            } else {
                stable_frames = 0;
            }

            publishVelocity(command_vx, command_vy, command_wz);
            ROS_INFO_THROTTLE(
                0.5,
                "%s中：pose=(%.3f,%.3f,%.1f度)，"
                "pos_err=%.3fm，yaw_err=%.1f度，cmd=(%.3f,%.3f,%.3f)",
                label.c_str(), pose.x, pose.y,
                pose.yaw * 180.0 / kPi,
                position_error, yaw_error * 180.0 / kPi,
                command_vx, command_vy, command_wz);
            rate.sleep();
        }

        stopRobot();
        ROS_ERROR("%s超时：未能在%.1fs内完成安全位姿过渡。",
                  label.c_str(), patrol_transition_timeout_);
        return false;
    }

    bool runCornerTransitionAfterSegment(std::size_t segment_index) {
        switch (segment_index) {
            case 0:
                // 上墙终点仍是x=4.75；同时右移到x=4.80并转到-90度。
                return runPatrolPoseTransition(
                    4.80, 4.30, -0.5 * kPi,
                    "上墙→右墙安全角点过渡");
            case 1:
                // 右墙终点仍是y=2.75；同时下移到y=2.70并转到-180度。
                return runPatrolPoseTransition(
                    4.80, 2.70, -kPi,
                    "右墙→下墙安全角点过渡");
            case 2:
                // 下墙终点仍是x=0.25；同时左移到x=0.20并转到+90度。
                return runPatrolPoseTransition(
                    0.20, 2.70, 0.5 * kPi,
                    "下墙→左墙安全角点过渡");
            default:
                // 第四条巡检线最终直接停在(0.20,4.25)，不再做额外角点过渡。
                return true;
        }
    }

'''

MEMBERS = r'''    // V12角点过渡：短距离全向XY+yaw同时闭环。
    double patrol_transition_position_kp_;
    double patrol_transition_yaw_kp_;
    double patrol_transition_min_linear_speed_;
    double patrol_transition_max_linear_speed_;
    double patrol_transition_min_angular_speed_;
    double patrol_transition_max_angular_speed_;
    double patrol_transition_linear_accel_;
    double patrol_transition_angular_accel_;
    double patrol_transition_position_tolerance_;
    double patrol_transition_yaw_tolerance_deg_;
    int patrol_transition_stable_frames_;
    double patrol_transition_timeout_;
'''

LAUNCH_PARAMS = r'''    <!-- V12：安全偏移巡检线角点过渡。短距离过渡直接使用/set_speed，
         XY位置与yaw同时闭环，不使用move_base最终姿态调整。 -->
    <param name="patrol_transition_position_kp" value="2.50" />
    <param name="patrol_transition_yaw_kp" value="2.00" />
    <param name="patrol_transition_min_linear_speed" value="0.025" />
    <param name="patrol_transition_max_linear_speed" value="0.12" />
    <param name="patrol_transition_min_angular_speed" value="0.12" />
    <param name="patrol_transition_max_angular_speed" value="0.65" />
    <param name="patrol_transition_linear_accel" value="0.60" />
    <param name="patrol_transition_angular_accel" value="1.50" />
    <param name="patrol_transition_position_tolerance" value="0.012" />
    <param name="patrol_transition_yaw_tolerance_deg" value="1.5" />
    <param name="patrol_transition_stable_frames" value="3" />
    <param name="patrol_transition_timeout" value="8.0" />
'''


def patch_cpp(text: str) -> str:
    if V12_MARKER in text:
        raise PatchError("target_patrol_docking.cpp 已经是V12")
    if V11_MARKER not in text:
        raise PatchError("target_patrol_docking.cpp不是指定V11基线")

    text = text.replace(V11_MARKER, V12_MARKER)

    # 构造参数：只在control_rate后插入一组V12参数。
    anchor = '        pnh_.param("control_rate", control_rate_, 15.0);\n'
    text = replace_once(text, anchor, anchor + "\n" + CTOR_PARAMS,
                        "插入V12角点参数")

    # normalizeParameters：插到duplicate_coordinate_distance_归一化之前。
    anchor = '''        duplicate_coordinate_distance_ =\n            std::fabs(duplicate_coordinate_distance_);\n'''
    text = replace_once(text, anchor, NORMALIZE_BLOCK + anchor,
                        "插入V12参数归一化")

    # 四条固定巡检线全部替换为用户的新偏移坐标。
    text = regex_replace_once(
        text,
        r'    void buildSegments\(\) \{.*?\n    \}\n\n    void addSegment',
        SEGMENTS_BLOCK + '\n    void addSegment',
        "替换V12四条巡检线",
        flags=re.DOTALL)

    # 初始move_base仍去(0.25,4.25)，到点后先横移5cm到第一条新巡检线。
    pattern = (
        r'(        if \(!navigateToPose\(start_x_, start_y_,\n'
        r'                            start_yaw_deg_ \* kPi / 180\.0,\n'
        r'                            "初始巡检点"\)\) \{\n'
        r'            return false;\n'
        r'        \}\n)')
    replacement = (
        r'\1'
        '        // V12：导航起点保持(0.25,4.25)不变；到点后不让move_base在墙边\n'
        '        // 再做最终姿态动作，直接用短距离全向P/PP横移到y=4.30。\n'
        '        if (!runPatrolPoseTransition(\n'
        '                0.25, 4.30, 0.0, "巡检起点安全横移")) {\n'
        '            return false;\n'
        '        }\n')
    text = regex_replace_once(text, pattern, replacement,
                              "插入起点5cm安全横移")

    # 新的全向过渡控制函数放到completePatrolSegment之前。
    text = replace_once(
        text,
        '    SegmentResult completePatrolSegment(std::size_t segment_index) {\n',
        TRANSITION_METHODS +
        '    SegmentResult completePatrolSegment(std::size_t segment_index) {\n',
        "插入V12角点过渡控制函数")

    # 段完成后先解除巡检锁/恢复普通参数，再做角点5cm+90度同步过渡，
    # 之后才执行原有“延后现实目标”处理，避免下一段从危险旧角点启动。
    complete_anchor = (
        '        ROS_INFO("%s完成；已恢复普通导航速度%.1f和路径重规划。",\n'
        '                 segment.name.c_str(), normal_navigation_speed_limit_);\n')
    complete_insert = complete_anchor + (
        '        if (!runCornerTransitionAfterSegment(segment_index)) {\n'
        '            ROS_ERROR("%s完成后安全角点过渡失败。", segment.name.c_str());\n'
        '            return SEGMENT_ABORTED;\n'
        '        }\n')
    text = replace_once(text, complete_anchor, complete_insert,
                        "段结束后插入V12角点过渡")

    # 成员变量。
    member_anchor = '    double segment_end_tolerance_;\n    double control_rate_;\n'
    text = replace_once(text, member_anchor,
                        member_anchor + '\n' + MEMBERS,
                        "插入V12成员变量")

    return text


def patch_launch(text: str) -> str:
    if V12_MARKER in text:
        raise PatchError("target_patrol_docking.launch 已经是V12")
    if V11_MARKER not in text:
        raise PatchError("target_patrol_docking.launch不是指定V11基线")

    text = text.replace(V11_MARKER, V12_MARKER)

    # 只要有唯一control_rate参数就紧随其后插入V12控制参数。
    pattern = r'(?m)^(\s*<param name="control_rate" value="[^"]+" />\s*)$'
    match = re.findall(pattern, text)
    if len(match) != 1:
        raise PatchError(f"launch control_rate参数预期1处，实际{len(match)}处")
    text = re.sub(pattern, r'\1\n' + LAUNCH_PARAMS.rstrip(),
                  text, count=1)

    # 增加一段明确路线说明，不依赖旧注释文本。
    route_comment = '''
    <!-- V12巡检中心线：
         起点move_base=(0.25,4.25,0deg)，随后横移到(0.25,4.30)。
         1) y=4.30，到x=4.75；同步过渡到(4.80,4.30,-90deg)。
         2) x=4.80，到y=2.75；同步过渡到(4.80,2.70,-180deg)。
         3) y=2.70，到x=0.25；同步过渡到(0.20,2.70,+90deg)。
         4) x=0.20，最终停在y=4.25。 -->
'''
    node_pattern = r'(<node[^>]*\btarget_patrol_docking\b[^>]*>)'
    # 某些launch的type/name分行，无法依赖单行node标签；退化为插到第一个map_frame前。
    map_anchor = '    <param name="map_frame" value="map" />\n'
    if text.count(map_anchor) == 1:
        text = text.replace(map_anchor, route_comment + map_anchor, 1)
    else:
        # 不影响运行，只跳过注释插入。
        pass
    return text


def post_validate(cpp: str, launch: str) -> None:
    checks = [
        (V12_MARKER in cpp, "cpp V12 marker"),
        (V12_MARKER in launch, "launch V12 marker"),
        ('runPatrolPoseTransition(\n                0.25, 4.30, 0.0' in cpp,
         "initial shift"),
        ('addSegment("上墙巡检", 0.25, 4.30, 4.75, 4.30,' in cpp,
         "top patrol line"),
        ('addSegment("右墙巡检", 4.80, 4.30, 4.80, 2.75,' in cpp,
         "right patrol line"),
        ('addSegment("下墙巡检", 4.80, 2.70, 0.25, 2.70,' in cpp,
         "bottom patrol line"),
        ('addSegment("左墙巡检", 0.20, 2.70, 0.20, 4.25,' in cpp,
         "left patrol line"),
        ('4.80, 4.30, -0.5 * kPi' in cpp, "corner 1"),
        ('4.80, 2.70, -kPi' in cpp, "corner 2"),
        ('0.20, 2.70, 0.5 * kPi' in cpp, "corner 3"),
        ('runCornerTransitionAfterSegment(segment_index)' in cpp,
         "segment completion transition"),
        ('patrol_transition_position_kp_' in cpp, "transition params cpp"),
        ('name="patrol_transition_position_kp"' in launch,
         "transition params launch"),
    ]
    failed = [name for ok, name in checks if not ok]
    if failed:
        raise PatchError("V12安装后检查失败：\n  - " + "\n  - ".join(failed))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "workspace", nargs="?", default="/home/ucar/ucar_ws_copy",
        help="catkin工作空间根目录")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    workspace = Path(args.workspace).expanduser().resolve()
    try:
        cpp_path, launch_path = locate_files(workspace)
        cpp_old = read_text(cpp_path)
        launch_old = read_text(launch_path)
        cpp_new = patch_cpp(cpp_old)
        launch_new = patch_launch(launch_old)
        post_validate(cpp_new, launch_new)

        print(f"target cpp   : {cpp_path}")
        print(f"target launch: {launch_path}")
        print("V12静态补丁检查：通过")
        if args.dry_run:
            print("dry-run：未写入文件")
            return 0

        backup_and_write(cpp_path, cpp_new, False)
        backup_and_write(launch_path, launch_new, False)
        print("V12安装完成；原文件已保留.bak_v11备份。")
        print("接下来执行：")
        print("  cd /home/ucar/ucar_ws_copy")
        print("  catkin_make")
        print("  source devel/setup.bash")
        return 0
    except PatchError as error:
        print(f"[V12 PATCH ERROR] {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
