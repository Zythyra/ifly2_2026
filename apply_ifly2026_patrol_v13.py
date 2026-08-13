#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IFLY2026 Patrol V13 patcher

基线：已经安装 V12 SAFE_OFFSET_CORNER_TRANSITION 的 target_patrol_docking。
只修改 target_patrol_docking.cpp / target_patrol_docking.launch：
  1) 提高角点过渡 yaw 闭环速度；
  2) 增加“旋转优先”线速度缩放，避免5cm平移先完成而车头仍朝墙；
  3) V12 四条巡检线坐标完全不变。

MyPlanner 的固定巡检硬路径绕过 C5/path-healing 由本交付包中的
my_planner 替换文件完成，不在这个脚本里修改。
"""

import argparse
import shutil
from pathlib import Path

V12_MARKER = "IFLY2026_FIXED_PATH_PATROL_V12_SAFE_OFFSET_CORNER_TRANSITION_20260810"
V13_MARKER = "IFLY2026_FIXED_PATH_PATROL_V13_HARD_PATH_FAST_CORNER_20260810"

class PatchError(RuntimeError):
    pass

def read_text(p: Path) -> str:
    return p.read_text(encoding="utf-8")

def replace_once(text: str, old: str, new: str, label: str) -> str:
    n = text.count(old)
    if n != 1:
        raise PatchError(f"{label}: 预期1处，实际{n}处")
    return text.replace(old, new, 1)

def locate(workspace: Path):
    src = workspace / "src"
    if not src.is_dir():
        raise PatchError(f"未找到工作空间src目录：{src}")
    cpp = []
    launch = []
    v13 = []
    for p in src.rglob("*"):
        if not p.is_file() or p.suffix not in {".cpp", ".launch", ".xml"}:
            continue
        try:
            t = read_text(p)
        except Exception:
            continue
        if V13_MARKER in t and "target_patrol_docking" in t:
            v13.append(p)
        if p.suffix == ".cpp" and V12_MARKER in t and "class TargetPatrolDocking" in t:
            cpp.append(p)
        if p.suffix in {".launch", ".xml"} and V12_MARKER in t and "target_patrol_docking" in t:
            launch.append(p)
    if v13 and not cpp:
        raise PatchError("检测到V13已经安装：\n  - " + "\n  - ".join(str(x) for x in v13[:10]))
    def pick(xs, name):
        exact = [p for p in xs if p.name == name]
        if len(exact) == 1:
            return exact[0]
        if len(xs) == 1:
            return xs[0]
        raise PatchError(f"无法唯一定位{name}，候选{len(xs)}个：\n  - " + "\n  - ".join(str(x) for x in xs[:20]))
    return pick(cpp, "target_patrol_docking.cpp"), pick(launch, "target_patrol_docking.launch")

NEW_CTOR_PARAMS = '''        pnh_.param("patrol_transition_yaw_priority_start_deg",
                   patrol_transition_yaw_priority_start_deg_, 55.0);
        pnh_.param("patrol_transition_yaw_priority_release_deg",
                   patrol_transition_yaw_priority_release_deg_, 20.0);
        pnh_.param("patrol_transition_yaw_priority_min_linear_scale",
                   patrol_transition_yaw_priority_min_linear_scale_, 0.15);
'''

NEW_NORMALIZE = '''        patrol_transition_yaw_priority_start_deg_ =
            std::max(5.0, std::fabs(patrol_transition_yaw_priority_start_deg_));
        patrol_transition_yaw_priority_release_deg_ = clampValue(
            std::fabs(patrol_transition_yaw_priority_release_deg_),
            0.0, patrol_transition_yaw_priority_start_deg_ - 1.0);
        patrol_transition_yaw_priority_min_linear_scale_ = clampValue(
            patrol_transition_yaw_priority_min_linear_scale_, 0.0, 1.0);
'''

PRIORITY_BLOCK = '''            // V13：角点过渡采用“旋转优先的同步平移”。
            // 当车头仍大幅朝向墙面时，只保留少量平移；随着yaw误差减小，
            // 线速度连续恢复到100%。这样仍然边转边移，但不会先走完5cm。
            double rotation_priority_scale = 1.0;
            const double abs_yaw_error_deg =
                std::fabs(yaw_error) * 180.0 / kPi;
            if (abs_yaw_error_deg >= patrol_transition_yaw_priority_start_deg_) {
                rotation_priority_scale =
                    patrol_transition_yaw_priority_min_linear_scale_;
            } else if (abs_yaw_error_deg >
                       patrol_transition_yaw_priority_release_deg_) {
                const double span = std::max(
                    1.0,
                    patrol_transition_yaw_priority_start_deg_
                    - patrol_transition_yaw_priority_release_deg_);
                const double progress = clampValue(
                    (patrol_transition_yaw_priority_start_deg_
                     - abs_yaw_error_deg) / span,
                    0.0, 1.0);
                rotation_priority_scale =
                    patrol_transition_yaw_priority_min_linear_scale_
                    + (1.0 - patrol_transition_yaw_priority_min_linear_scale_)
                      * progress;
            }
            desired_vx *= rotation_priority_scale;
            desired_vy *= rotation_priority_scale;

'''

NEW_MEMBERS = '''    double patrol_transition_yaw_priority_start_deg_;
    double patrol_transition_yaw_priority_release_deg_;
    double patrol_transition_yaw_priority_min_linear_scale_;
'''

NEW_LAUNCH = '''    <!-- V13：角点旋转优先。大角度误差时压低平移，车头转开墙面后逐步恢复线速度。 -->
    <param name="patrol_transition_yaw_priority_start_deg" value="55.0" />
    <param name="patrol_transition_yaw_priority_release_deg" value="20.0" />
    <param name="patrol_transition_yaw_priority_min_linear_scale" value="0.15" />
'''

def patch_cpp(text: str) -> str:
    if V13_MARKER in text:
        raise PatchError("target_patrol_docking.cpp 已经是V13")
    if V12_MARKER not in text:
        raise PatchError("target_patrol_docking.cpp不是指定V12基线")
    text = text.replace(V12_MARKER, V13_MARKER, 1)

    # 明显提高旋转闭环速度，同时不提高5cm平移上限。
    text = replace_once(text,
        '        pnh_.param("patrol_transition_yaw_kp",\n                   patrol_transition_yaw_kp_, 2.00);',
        '        pnh_.param("patrol_transition_yaw_kp",\n                   patrol_transition_yaw_kp_, 3.00);',
        "yaw_kp")
    text = replace_once(text,
        '        pnh_.param("patrol_transition_min_angular_speed",\n                   patrol_transition_min_angular_speed_, 0.12);',
        '        pnh_.param("patrol_transition_min_angular_speed",\n                   patrol_transition_min_angular_speed_, 0.25);',
        "min angular")
    text = replace_once(text,
        '        pnh_.param("patrol_transition_max_angular_speed",\n                   patrol_transition_max_angular_speed_, 0.65);',
        '        pnh_.param("patrol_transition_max_angular_speed",\n                   patrol_transition_max_angular_speed_, 1.20);',
        "max angular")
    text = replace_once(text,
        '        pnh_.param("patrol_transition_angular_accel",\n                   patrol_transition_angular_accel_, 1.50);',
        '        pnh_.param("patrol_transition_angular_accel",\n                   patrol_transition_angular_accel_, 4.00);\n' + NEW_CTOR_PARAMS.rstrip(),
        "angular accel + priority params")

    norm_anchor = '''        patrol_transition_position_tolerance_ =
            std::max(0.003, std::fabs(patrol_transition_position_tolerance_));
'''
    text = replace_once(text, norm_anchor, NEW_NORMALIZE + norm_anchor,
                        "priority normalize")

    priority_anchor = '''            const double yaw_tolerance =
                patrol_transition_yaw_tolerance_deg_ * kPi / 180.0;
'''
    text = replace_once(text, priority_anchor, PRIORITY_BLOCK + priority_anchor,
                        "rotation priority block")

    member_anchor = '''    double patrol_transition_position_tolerance_;
'''
    text = replace_once(text, member_anchor, NEW_MEMBERS + member_anchor,
                        "priority members")

    text = text.replace(
        '采用全向XY+yaw同时P闭环。',
        '采用全向XY+yaw同时P闭环，并启用V13旋转优先。', 1)

    return text

def patch_launch(text: str) -> str:
    if V13_MARKER in text:
        raise PatchError("target_patrol_docking.launch 已经是V13")
    if V12_MARKER not in text:
        raise PatchError("target_patrol_docking.launch不是指定V12基线")
    text = text.replace(V12_MARKER, V13_MARKER, 1)
    text = replace_once(text,
        '<param name="patrol_transition_yaw_kp" value="2.00" />',
        '<param name="patrol_transition_yaw_kp" value="3.00" />',
        "launch yaw kp")
    text = replace_once(text,
        '<param name="patrol_transition_min_angular_speed" value="0.12" />',
        '<param name="patrol_transition_min_angular_speed" value="0.25" />',
        "launch min angular")
    text = replace_once(text,
        '<param name="patrol_transition_max_angular_speed" value="0.65" />',
        '<param name="patrol_transition_max_angular_speed" value="1.20" />',
        "launch max angular")
    text = replace_once(text,
        '<param name="patrol_transition_angular_accel" value="1.50" />',
        '<param name="patrol_transition_angular_accel" value="4.00" />\n' + NEW_LAUNCH.rstrip(),
        "launch angular accel + priority")
    return text

def validate(cpp: str, launch: str):
    checks = [
        (V13_MARKER in cpp, "V13 cpp marker"),
        (V13_MARKER in launch, "V13 launch marker"),
        ('patrol_transition_yaw_kp_, 3.00' in cpp, "yaw kp=3"),
        ('patrol_transition_max_angular_speed_, 1.20' in cpp, "max wz=1.2"),
        ('patrol_transition_angular_accel_, 4.00' in cpp, "angular accel=4"),
        ('rotation_priority_scale' in cpp, "rotation priority logic"),
        ('patrol_transition_yaw_priority_start_deg_' in cpp, "priority params"),
        ('name="patrol_transition_yaw_priority_start_deg" value="55.0"' in launch,
         "launch priority params"),
        ('addSegment("上墙巡检", 0.25, 4.30, 4.75, 4.30,' in cpp,
         "V12 top line preserved"),
        ('addSegment("右墙巡检", 4.80, 4.30, 4.80, 2.75,' in cpp,
         "V12 right line preserved"),
        ('addSegment("下墙巡检", 4.80, 2.70, 0.25, 2.70,' in cpp,
         "V12 bottom line preserved"),
        ('addSegment("左墙巡检", 0.20, 2.70, 0.20, 4.25,' in cpp,
         "V12 left line preserved"),
    ]
    failed = [name for ok, name in checks if not ok]
    if failed:
        raise PatchError("V13静态检查失败：" + ", ".join(failed))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("workspace", help="ROS工作空间，例如 /home/ucar/ucar_ws_copy")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    ws = Path(args.workspace).expanduser().resolve()
    cpp_path, launch_path = locate(ws)
    cpp_new = patch_cpp(read_text(cpp_path))
    launch_new = patch_launch(read_text(launch_path))
    validate(cpp_new, launch_new)
    print(f"target cpp   : {cpp_path}")
    print(f"target launch: {launch_path}")
    print("V13静态补丁检查：通过")
    if args.dry_run:
        print("dry-run：未写入文件")
        return
    for p, text in [(cpp_path, cpp_new), (launch_path, launch_new)]:
        backup = Path(str(p) + ".bak_v12")
        if not backup.exists():
            shutil.copy2(p, backup)
        tmp = Path(str(p) + ".v13.tmp")
        tmp.write_text(text, encoding="utf-8")
        tmp.replace(p)
    print("V13已写入；原V12文件已备份为 .bak_v12")

if __name__ == "__main__":
    try:
        main()
    except PatchError as e:
        print(f"[V13 PATCH ERROR] {e}", file=__import__('sys').stderr)
        raise SystemExit(2)
