#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IFLY2026 V11 patcher

从已经验证可跑通的 V10 工作空间直接安装 V11。
默认先创建 .bak_v10 备份；关键 V10 片段不匹配时拒绝写入。
"""

import argparse
import re
import shutil
import sys
from pathlib import Path

V10_MARKER = "IFLY2026_FIXED_PATH_PATROL_V10_OSCILLATION_GUARD_FAST_ALIGN_20260806"
V11_MARKER = "IFLY2026_FIXED_PATH_PATROL_V11_PRECISE_CORNER_DEADZONE_FIX_20260808"


class PatchError(RuntimeError):
    pass


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def write_text(path: Path, text: str, dry_run: bool) -> None:
    if dry_run:
        return
    backup = Path(str(path) + ".bak_v10")
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(text, encoding="utf-8")


def replace_exact(text: str, old: str, new: str, label: str,
                  expected_min: int = 1, expected_max=None) -> str:
    count = text.count(old)
    if count < expected_min:
        raise PatchError(f"{label}: 未找到预期 V10 片段")
    if expected_max is not None and count > expected_max:
        raise PatchError(f"{label}: 匹配到 {count} 处，超过预期 {expected_max} 处")
    return text.replace(old, new)


def regex_replace_once(text: str, pattern: str, repl: str, label: str,
                       flags: int = 0) -> str:
    out, count = re.subn(pattern, repl, text, count=1, flags=flags)
    if count != 1:
        raise PatchError(f"{label}: 预期匹配 1 处，实际 {count} 处")
    return out


def find_candidates(root: Path):
    return [p for p in root.rglob("*") if p.is_file()]


def pick_unique(files, predicate, label: str, preferred_basename=None) -> Path:
    found = []
    for p in files:
        try:
            if predicate(p):
                found.append(p)
        except (UnicodeDecodeError, OSError):
            pass
    if len(found) > 1 and preferred_basename:
        preferred = [p for p in found if p.name == preferred_basename]
        if len(preferred) == 1:
            return preferred[0]
    if len(found) != 1:
        details = "\n".join(f"  - {p}" for p in found[:20])
        raise PatchError(
            f"{label}: 需要唯一匹配，实际找到 {len(found)} 个。\n{details}"
        )
    return found[0]


def locate_files(workspace: Path):
    src = workspace / "src"
    if not src.is_dir():
        raise PatchError(f"未找到工作空间 src：{src}")
    files = find_candidates(src)

    def text_has(p: Path, *needles: str) -> bool:
        if p.suffix not in {".cpp", ".yaml", ".yml", ".launch", ".xml"}:
            return False
        t = read_text(p)
        return all(n in t for n in needles)

    planner_cpp = pick_unique(
        files,
        lambda p: p.suffix == ".cpp" and text_has(
            p, V10_MARKER, "PLUGINLIB_EXPORT_CLASS", "computePatrolHeadingHoldCommand"
        ),
        "V10 MyPlanner C++", preferred_basename="my_planner.cpp",
    )

    patrol_cpp = pick_unique(
        files,
        lambda p: p.suffix == ".cpp" and text_has(
            p, V10_MARKER, "class TargetPatrolDocking", "buildSegments()"
        ),
        "V10 target_patrol_docking C++", preferred_basename="target_patrol_docking.cpp",
    )

    patrol_launch = pick_unique(
        files,
        lambda p: p.suffix in {".launch", ".xml"} and text_has(
            p, V10_MARKER, "target_patrol_docking", "segment_end_tolerance"
        ),
        "V10 target_patrol_docking launch", preferred_basename="target_patrol_docking.launch",
    )

    # MyPlanner YAML：优先使用项目标准固定路径。
    # 实车参数可能已调过，因此不能依赖V10某几个默认值来定位。
    canonical_yaml = (
        workspace / "src" / "my_planner" / "config" / "my_planner_params.yaml"
    )
    if canonical_yaml.is_file():
        planner_yaml = canonical_yaml
    else:
        # 若目录结构变化，再按参数键宽松搜索。
        planner_yaml = pick_unique(
            files,
            lambda p: p.suffix in {".yaml", ".yml"} and text_has(
                p,
                "patrol_pp_align_tolerance_deg:",
                "patrol_heading_max_wz:",
                "patrol_pp_max_vy:",
                "patrol_pp_goal_position_tolerance:",
            ),
            "V10 MyPlanner YAML", preferred_basename="my_planner_params.yaml",
        )

    return planner_cpp, planner_yaml, patrol_cpp, patrol_launch


def patch_planner_cpp(text: str) -> str:
    if V10_MARKER not in text:
        raise PatchError("planner cpp 不是指定 V10 基线")

    text = text.replace(V10_MARKER, V11_MARKER)

    # 构造函数默认值：YAML漏加载时也保持V11行为。
    replacements = [
        ("patrol_pp_align_tolerance_(5.0 * M_PI / 180.0)",
         "patrol_pp_align_tolerance_(1.5 * M_PI / 180.0)",
         "planner ctor align tolerance"),
        ("patrol_heading_kp_(1.20)", "patrol_heading_kp_(1.50)",
         "planner ctor heading kp"),
        ("patrol_heading_max_wz_(0.18)", "patrol_heading_max_wz_(0.30)",
         "planner ctor heading max wz"),
        ("patrol_heading_acc_lim_(0.60)", "patrol_heading_acc_lim_(1.50)",
         "planner ctor heading acc"),
        ("patrol_pp_lateral_gain_(1.80)", "patrol_pp_lateral_gain_(2.20)",
         "planner ctor lateral gain"),
        ("patrol_pp_max_vy_(0.20)", "patrol_pp_max_vy_(0.32)",
         "planner ctor max vy"),
        ("patrol_pp_acc_lim_y_(0.80)", "patrol_pp_acc_lim_y_(1.20)",
         "planner ctor lateral acc"),
        ("patrol_pp_avoid_offset_rate_(0.35)", "patrol_pp_avoid_offset_rate_(0.50)",
         "planner ctor avoid rate"),
        ("patrol_pp_goal_position_tolerance_(0.04)",
         "patrol_pp_goal_position_tolerance_(0.015)",
         "planner ctor goal tolerance"),
    ]
    for old, new, label in replacements:
        text = replace_exact(text, old, new, label, 1, 1)

    text = replace_exact(
        text,
        "double patrol_pp_align_tolerance_deg = 5.0;",
        "double patrol_pp_align_tolerance_deg = 1.5;",
        "planner param align local default", 1, 1
    )
    text = replace_exact(
        text,
        'private_nh.param("patrol_pp_align_tolerance_deg",\n'
        '                     patrol_pp_align_tolerance_deg, 5.0);',
        'private_nh.param("patrol_pp_align_tolerance_deg",\n'
        '                     patrol_pp_align_tolerance_deg, 1.5);',
        "planner param align default", 1, 1
    )

    numeric_default_pairs = [
        ('private_nh.param("patrol_heading_kp",\n'
         '                     patrol_heading_kp_, 1.20);',
         'private_nh.param("patrol_heading_kp",\n'
         '                     patrol_heading_kp_, 1.50);',
         "planner param heading kp"),
        ('private_nh.param("patrol_heading_max_wz",\n'
         '                     patrol_heading_max_wz_, 0.18);',
         'private_nh.param("patrol_heading_max_wz",\n'
         '                     patrol_heading_max_wz_, 0.30);',
         "planner param heading max wz"),
        ('private_nh.param("patrol_heading_acc_lim",\n'
         '                     patrol_heading_acc_lim_, 0.60);',
         'private_nh.param("patrol_heading_acc_lim",\n'
         '                     patrol_heading_acc_lim_, 1.50);',
         "planner param heading acc"),
        ('private_nh.param("patrol_pp_lateral_gain",\n'
         '                     patrol_pp_lateral_gain_, 1.80);',
         'private_nh.param("patrol_pp_lateral_gain",\n'
         '                     patrol_pp_lateral_gain_, 2.20);',
         "planner param lateral gain"),
        ('private_nh.param("patrol_pp_max_vy",\n'
         '                     patrol_pp_max_vy_, 0.20);',
         'private_nh.param("patrol_pp_max_vy",\n'
         '                     patrol_pp_max_vy_, 0.32);',
         "planner param max vy"),
        ('private_nh.param("patrol_pp_acc_lim_y",\n'
         '                     patrol_pp_acc_lim_y_, 0.80);',
         'private_nh.param("patrol_pp_acc_lim_y",\n'
         '                     patrol_pp_acc_lim_y_, 1.20);',
         "planner param lateral acc"),
        ('private_nh.param("patrol_pp_avoid_offset_rate",\n'
         '                     patrol_pp_avoid_offset_rate_, 0.35);',
         'private_nh.param("patrol_pp_avoid_offset_rate",\n'
         '                     patrol_pp_avoid_offset_rate_, 0.50);',
         "planner param avoid rate"),
        ('private_nh.param("patrol_pp_goal_position_tolerance",\n'
         '                     patrol_pp_goal_position_tolerance_, 0.04);',
         'private_nh.param("patrol_pp_goal_position_tolerance",\n'
         '                     patrol_pp_goal_position_tolerance_, 0.015);',
         "planner param goal tolerance"),
    ]
    for old, new, label in numeric_default_pairs:
        text = replace_exact(text, old, new, label, 1, 1)

    # 巡检起步/续巡旋转：完全复用普通导航 P + min/max 限幅。
    old_rotation = r"""    double angular_speed = 0\.0;
    if \(patrol_pp_active\)
    \{
        // 巡检起步旋转采用和普通初始姿态调整相同的比例限幅思路。
        // 进入约5度后直接交给行进中航向保持，不再停车精调和等待停稳。
        angular_speed = std::min\(
            std::abs\(angle_error\) \* patrol_align_kp_,
            patrol_align_max_wz_\);
        if \(std::abs\(angle_error\) <= patrol_align_near_angle_\)
            angular_speed = std::min\(angular_speed, patrol_align_near_wz_\);
    \}
    else
    \{
        angular_speed =
            std::abs\(angle_error\) \* initial_angular_gain_;
        angular_speed =
            clampValue\(angular_speed,
                       initial_min_angular_speed_,
                       initial_max_angular_speed_\);
    \}"""
    new_rotation = """    // V11：固定巡检与普通导航使用完全相同的纯比例初始姿态调整。
    // tolerance只负责判定“是否已经对准”；只要还在tolerance外，
    // 就通过普通导航已有的最小角速度跨过底盘死区。
    double angular_speed =
        std::abs(angle_error) * initial_angular_gain_;
    angular_speed =
        clampValue(angular_speed,
                   initial_min_angular_speed_,
                   initial_max_angular_speed_);"""
    text = regex_replace_once(
        text, old_rotation, new_rotation,
        "planner initial rotation V11", flags=re.MULTILINE
    )

    # 行进中航向保持：deadband外必须给到底盘可执行的最小wz。
    old_heading = r"""double MyPlanner::computePatrolHeadingHoldCommand\(double angle_error\) const
\{
    if \(!patrol_heading_hold_enabled_
        \|\| std::abs\(angle_error\) <= patrol_heading_deadband_\)
    \{
        return 0\.0;
    \}

    return clampValue\(
        patrol_heading_kp_ \* angle_error,
        -patrol_heading_max_wz_, patrol_heading_max_wz_\);
\}"""
    new_heading = """double MyPlanner::computePatrolHeadingHoldCommand(double angle_error) const
{
    if (!patrol_heading_hold_enabled_
        || std::abs(angle_error) <= patrol_heading_deadband_)
    {
        return 0.0;
    }

    double wz = clampValue(
        patrol_heading_kp_ * angle_error,
        -patrol_heading_max_wz_, patrol_heading_max_wz_);

    // V11：deadband外必须跨过底盘实际角速度死区。
    // 复用普通导航initial_min_angular_speed_，避免再维护一套重复参数。
    if (std::abs(wz) < initial_min_angular_speed_)
        wz = std::copysign(initial_min_angular_speed_, angle_error);

    return clampValue(
        wz, -patrol_heading_max_wz_, patrol_heading_max_wz_);
}"""
    text = regex_replace_once(
        text, old_heading, new_heading,
        "planner heading deadzone fix", flags=re.MULTILINE
    )

    text = text.replace(
        "巡检车头需要快速转入原始固定路径的5度接管范围",
        "巡检车头需要转入原始固定路径的精确对准范围"
    )
    text = text.replace(
        "巡检车头已在5度接管范围内，直接边走边调整",
        "巡检车头已在精确对准范围内，直接开始路径跟踪"
    )
    return text


def patch_planner_yaml(text: str) -> str:
    if "IFLY2026 V11" not in text:
        text = (
            "# IFLY2026 V11：真实角点 + 巡检纯P对准 + 航向死区补偿 + 横移增强\n"
            "# 基于V10，其余MPC/C4.1参数保持不变。\n"
            + text
        )

    # V11按“参数名”更新，不要求用户当前值等于V10默认值。
    # 这样即使V10实车阶段已经手工调参，也能正常升级。
    params = {
        "patrol_pp_align_tolerance_deg": "1.5",
        "patrol_heading_kp": "1.50",
        "patrol_heading_max_wz": "0.30",
        "patrol_heading_acc_lim": "1.50",
        "patrol_pp_lateral_gain": "2.20",
        "patrol_pp_max_vy": "0.32",
        "patrol_pp_acc_lim_y": "1.20",
        "patrol_pp_avoid_offset_rate": "0.50",
        "patrol_pp_goal_position_tolerance": "0.015",
    }

    for key, value in params.items():
        pattern = rf"(?m)^(\s*{re.escape(key)}\s*:\s*)[^#\r\n]+"
        text, count = re.subn(
            pattern,
            lambda m, v=value: m.group(1) + v,
            text,
            count=1,
        )
        if count != 1:
            raise PatchError(f"planner yaml参数缺失：{key}")

    text = text.replace(
        "# 开始巡检前按普通初始姿态调整的比例限幅方式快速旋转；进入5度后\n"
        "# 不停车、不等待角速度稳定，立即开始vx/vy跟踪并由航向闭环继续收敛。",
        "# V11：开始/续巡前直接复用普通导航初始姿态P控制；\n"
        "# 误差进入1.5度后退出纯旋转并开始vx/vy跟踪。"
    )
    text = text.replace(
        "# 以下两个参数仅为兼容旧V9配置保留；V10不再进入停车等待状态。",
        "# 旧巡检旋转参数保留兼容；V11起步旋转实际复用普通导航initial_*参数。"
    )
    return text


def patch_patrol_cpp(text: str) -> str:
    if V10_MARKER not in text:
        raise PatchError("target_patrol_docking cpp 不是指定 V10 基线")

    text = text.replace(V10_MARKER, V11_MARKER)
    text = replace_exact(
        text,
        'pnh_.param("segment_end_tolerance", segment_end_tolerance_, 0.025);',
        'pnh_.param("segment_end_tolerance", segment_end_tolerance_, 0.015);',
        "patrol cpp segment tolerance", 1, 1
    )

    endpoint_pairs = [
        ('addSegment("上墙巡检", 0.25, 4.25, 4.70, 4.25,',
         'addSegment("上墙巡检", 0.25, 4.25, 4.75, 4.25,',
         "top true corner"),
        ('addSegment("右墙巡检", 4.75, 4.25, 4.75, 2.80,',
         'addSegment("右墙巡检", 4.75, 4.25, 4.75, 2.75,',
         "right true corner"),
        ('addSegment("下墙巡检", 4.75, 2.75, 0.30, 2.75,',
         'addSegment("下墙巡检", 4.75, 2.75, 0.25, 2.75,',
         "bottom true corner"),
        ('addSegment("左墙巡检", 0.25, 2.75, 0.25, 4.20,',
         'addSegment("左墙巡检", 0.25, 2.75, 0.25, 4.25,',
         "left true corner"),
    ]
    for old, new, label in endpoint_pairs:
        text = replace_exact(text, old, new, label, 1, 1)

    text = text.replace(
        "// 5cm仅用于当前巡检段的末端：在到达下一条巡检线前提前停车。",
        "// V11取消5cm提前停车：每段必须走到真实巡检线交点后才能换向。"
    )
    return text


def patch_patrol_launch(text: str) -> str:
    if V10_MARKER not in text:
        raise PatchError("target_patrol_docking launch 不是指定 V10 基线")

    text = text.replace(V10_MARKER, V11_MARKER)
    text = replace_exact(
        text,
        '<param name="segment_end_tolerance" value="0.025" />',
        '<param name="segment_end_tolerance" value="0.015" />',
        "launch segment tolerance", 1, 1
    )
    text = text.replace(
        "四条基准线和提前5cm结束的位置保持不变。",
        "四条基准线全部走到真实角点；上一段终点与下一段起点严格重合。"
    )
    return text


def post_validate(planner_cpp: str, planner_yaml: str,
                  patrol_cpp: str, patrol_launch: str) -> None:
    checks = [
        (V11_MARKER in planner_cpp, "planner cpp V11 marker"),
        (V11_MARKER in patrol_cpp, "patrol cpp V11 marker"),
        (V11_MARKER in patrol_launch, "launch V11 marker"),
        ("patrol_pp_align_tolerance_deg: 1.5" in planner_yaml,
         "yaml 1.5deg align"),
        ("patrol_pp_goal_position_tolerance: 0.015" in planner_yaml,
         "yaml 1.5cm planner goal"),
        ("patrol_pp_max_vy: 0.32" in planner_yaml, "yaml max vy"),
        ("deadband外必须跨过底盘实际角速度死区" in planner_cpp,
         "heading deadzone floor"),
        ('addSegment("上墙巡检", 0.25, 4.25, 4.75, 4.25,' in patrol_cpp,
         "top corner endpoint"),
        ('addSegment("右墙巡检", 4.75, 4.25, 4.75, 2.75,' in patrol_cpp,
         "right corner endpoint"),
        ('addSegment("下墙巡检", 4.75, 2.75, 0.25, 2.75,' in patrol_cpp,
         "bottom corner endpoint"),
        ('addSegment("左墙巡检", 0.25, 2.75, 0.25, 4.25,' in patrol_cpp,
         "left corner endpoint"),
        ('segment_end_tolerance" value="0.015"' in patrol_launch,
         "launch segment tolerance"),
    ]
    failed = [name for ok, name in checks if not ok]
    if failed:
        raise PatchError("V11 后检查失败：\n  - " + "\n  - ".join(failed))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "workspace", nargs="?", default="/home/ucar/ucar_ws_copy",
        help="catkin工作空间根目录"
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    workspace = Path(args.workspace).expanduser().resolve()
    paths = locate_files(workspace)
    planner_cpp_path, planner_yaml_path, patrol_cpp_path, patrol_launch_path = paths

    print("[V11] 已定位V10基线：")
    print("  planner cpp  :", planner_cpp_path)
    print("  planner yaml :", planner_yaml_path)
    print("  patrol cpp   :", patrol_cpp_path)
    print("  patrol launch:", patrol_launch_path)

    planner_cpp = patch_planner_cpp(read_text(planner_cpp_path))
    planner_yaml = patch_planner_yaml(read_text(planner_yaml_path))
    patrol_cpp = patch_patrol_cpp(read_text(patrol_cpp_path))
    patrol_launch = patch_patrol_launch(read_text(patrol_launch_path))

    post_validate(planner_cpp, planner_yaml, patrol_cpp, patrol_launch)

    if args.dry_run:
        print("[V11] dry-run通过：所有关键V10片段均唯一匹配，未写文件。")
        return 0

    write_text(planner_cpp_path, planner_cpp, False)
    write_text(planner_yaml_path, planner_yaml, False)
    write_text(patrol_cpp_path, patrol_cpp, False)
    write_text(patrol_launch_path, patrol_launch, False)

    print("[V11] 安装完成；四个原文件均保留.bak_v10备份。")
    print("[V11] 参数：align=1.5deg；heading=Kp1.50/max0.30/acc1.50；")
    print("      lateral=Kp2.20/max_vy0.32/acc1.20/offset_rate0.50；")
    print("      planner_goal_tol=0.015m；segment_tol=0.015m；真实角点启用。")
    print(f"下一步：cd {workspace} && catkin_make && source devel/setup.bash")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except PatchError as exc:
        print(f"[V11][ERROR] {exc}", file=sys.stderr)
        raise SystemExit(2)
