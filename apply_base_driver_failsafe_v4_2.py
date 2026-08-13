#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
BASE-FAILSAFE V4.1 -> V4.2

Adds:
    base_diag_enable: true / false

false disables only BASE-DIAG output and diagnostic topics.
FAILSAFE logic and braking remain fully active.
'''

from pathlib import Path
import shutil
import sys

path = Path(sys.argv[1] if len(sys.argv) > 1
            else "src/ucar_controller/src/base_driver.cpp")

if not path.exists():
    raise SystemExit(f"file not found: {path}")

text = path.read_text(encoding="utf-8")

if "BASE-FAILSAFE V4.2" in text:
    raise SystemExit("BASE-FAILSAFE V4.2 is already installed; aborting.")

if "BASE-FAILSAFE V4.1" not in text:
    raise SystemExit(
        "This patch expects BASE-FAILSAFE V4.1. "
        "Restore/install V4.1 first."
    )

backup = path.with_name(path.name + ".bak_failsafe_v4_1")
if not backup.exists():
    shutil.copy2(path, backup)
    print(f"saved V4.1 backup: {backup}")


def replace_once(old, new, desc):
    global text
    n = text.count(old)
    if n != 1:
        raise SystemExit(
            f"patch failed at {desc}: expected exactly 1 match, got {n}"
        )
    text = text.replace(old, new, 1)


# 1) Global switch.
anchor = '''bool g_fs_fast_delay_enable = true;
double g_fs_fast_delay_min_ms = 300.0;
'''
replacement = '''// ==================== BASE-FAILSAFE V4.2 ====================
// Diagnostic-output switch.
// false disables only BASE-DIAG logs and pure diagnostic topics.
// It NEVER disables any failsafe detector or emergency braking.
bool g_base_diag_enable = true;

bool g_fs_fast_delay_enable = true;
double g_fs_fast_delay_min_ms = 300.0;
'''
replace_once(anchor, replacement, "global base_diag_enable")


# 2) ROS private parameter.
anchor = '''  // FAST_DELAY: high-confidence single-fresh-odom trigger.
  pravite_nh.param("failsafe_fast_delay_enable", g_fs_fast_delay_enable, true);
'''
replacement = '''  // Competition/debug diagnostic output switch.
  pravite_nh.param("base_diag_enable", g_base_diag_enable, true);

  // FAST_DELAY: high-confidence single-fresh-odom trigger.
  pravite_nh.param("failsafe_fast_delay_enable", g_fs_fast_delay_enable, true);
'''
replace_once(anchor, replacement, "ROS parameter base_diag_enable")


# 3) Disable pure diagnostic topic publishing when requested.
applied_target = "g_applied_cmd_pub.publish(applied_cmd);"
applied_count = text.count(applied_target)
if applied_count < 1:
    raise SystemExit(
        "patch failed: g_applied_cmd_pub.publish(applied_cmd) not found"
    )
text = text.replace(
    applied_target,
    "if (g_base_diag_enable) g_applied_cmd_pub.publish(applied_cmd);"
)

tx_target = "g_tx_diag_pub.publish(diag_msg);"
tx_count = text.count(tx_target)
if tx_count < 1:
    raise SystemExit(
        "patch failed: g_tx_diag_pub.publish(diag_msg) not found"
    )
text = text.replace(
    tx_target,
    "if (g_base_diag_enable) g_tx_diag_pub.publish(diag_msg);"
)


# 4) Guard every ROS log call whose message contains [BASE-DIAG].
#    [BASE-FAILSAFE] logs are intentionally untouched.
def guard_base_diag_logs(src):
    marker = "[BASE-DIAG]"
    call_starts = []
    search_from = 0

    while True:
        marker_pos = src.find(marker, search_from)
        if marker_pos < 0:
            break

        lower = max(0, marker_pos - 2000)
        candidates = []

        for macro in (
            "ROS_INFO_THROTTLE",
            "ROS_WARN_THROTTLE",
            "ROS_ERROR_THROTTLE",
            "ROS_DEBUG_THROTTLE",
            "ROS_INFO",
            "ROS_WARN",
            "ROS_ERROR",
            "ROS_DEBUG",
        ):
            p = src.rfind(macro, lower, marker_pos)
            if p >= 0:
                candidates.append(p)

        if not candidates:
            raise RuntimeError(
                f"Could not find ROS macro for [BASE-DIAG] at {marker_pos}"
            )

        call_start = max(candidates)

        # Reject an older unrelated call.
        if ";" in src[call_start:marker_pos]:
            raise RuntimeError(
                f"Ambiguous ROS macro before [BASE-DIAG] at {marker_pos}"
            )

        prefix = src[max(0, call_start - 60):call_start]
        if "g_base_diag_enable" not in prefix:
            call_starts.append(call_start)

        search_from = marker_pos + len(marker)

    for pos in sorted(set(call_starts), reverse=True):
        src = src[:pos] + "if (g_base_diag_enable) " + src[pos:]

    return src, len(set(call_starts))


text, guarded_logs = guard_base_diag_logs(text)
if guarded_logs < 1:
    raise SystemExit("patch failed: no [BASE-DIAG] logs were guarded")


# 5) Version marker.
text = text.replace("BASE-FAILSAFE V4.1", "BASE-FAILSAFE V4.2")

required = [
    "BASE-FAILSAFE V4.2",
    "bool g_base_diag_enable = true;",
    'pravite_nh.param("base_diag_enable", g_base_diag_enable, true);',
    "if (g_base_diag_enable) g_applied_cmd_pub.publish(applied_cmd);",
    "if (g_base_diag_enable) g_tx_diag_pub.publish(diag_msg);",
]
for token in required:
    if token not in text:
        raise SystemExit(f"patch incomplete, missing token: {token}")

path.write_text(text, encoding="utf-8")

print(f"BASE-FAILSAFE V4.2 written to: {path}")
print(f"V4.1 backup: {backup}")
print(f"guarded [BASE-DIAG] log calls: {guarded_logs}")
print(f"guarded applied_cmd publishes: {applied_count}")
print(f"guarded tx_diag publishes: {tx_count}")
print("")
print("Competition mode:")
print("  base_diag_enable: false")
print("")
print("Debug mode:")
print("  base_diag_enable: true")
print("")
print("FAILSAFE braking remains enabled in both modes.")
