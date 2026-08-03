#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Upgrade BASE-FAILSAFE V4 -> BASE-FAILSAFE V4.1.

V4.1:
  - Adds a high-confidence FAST_DELAY trigger that brakes on one fresh odom frame.
  - Shortens the regular delay evaluation window from 300 ms to 220 ms.
  - Lowers regular fault delay from 300 ms to 250 ms.
  - Keeps regular delay confirmation at 2 fresh odom frames.
  - Shortens angular reverse confirmation from 6 to 5 cycles.

Emergency behavior is unchanged:
  immediate ZERO on the triggering TX, then 100 Hz ZERO packets,
  fresh-odom stop confirmation, HOLD_STOP, and automatic recovery.

No base_driver.h change is required.
'''

from pathlib import Path
import shutil
import sys

path = Path(sys.argv[1] if len(sys.argv) > 1
            else "src/ucar_controller/src/base_driver.cpp")

if not path.exists():
    raise SystemExit(f"file not found: {path}")

text = path.read_text(encoding="utf-8")

if "BASE-FAILSAFE V4.1" in text:
    raise SystemExit("BASE-FAILSAFE V4.1 is already installed; aborting.")

if "BASE-FAILSAFE V4" not in text:
    raise SystemExit(
        "This patch expects BASE-FAILSAFE V4. "
        "Restore/install V4 first, then run V4.1 patch."
    )

backup = path.with_name(path.name + ".bak_failsafe_v4")
if not backup.exists():
    shutil.copy2(path, backup)
    print(f"saved V4 backup: {backup}")


def replace_once(old, new, description):
    global text
    count = text.count(old)
    if count != 1:
        raise SystemExit(
            f"patch failed at {description}: expected exactly 1 match, got {count}"
        )
    text = text.replace(old, new, 1)


# 1) Faster but still structured defaults.
replace_once(
    "int g_fs_reverse_confirm_cycles = 6;  // roughly 300 ms at 20 Hz",
    "int g_fs_reverse_confirm_cycles = 5;  // roughly 250 ms at 20 Hz",
    "reverse confirmation default"
)

replace_once(
    "double g_fs_delay_eval_window_ms = 300.0;",
    "double g_fs_delay_eval_window_ms = 220.0;",
    "delay evaluation window default"
)

replace_once(
    "double g_fs_delay_fault_min_ms = 300.0;",
    "double g_fs_delay_fault_min_ms = 250.0;",
    "regular delay fault threshold default"
)

replace_once(
    "double g_fs_delay_vs_normal_ratio_max = 0.72;",
    "double g_fs_delay_vs_normal_ratio_max = 0.85;",
    "regular ratio default"
)

replace_once(
    "double g_fs_delay_improvement_min = 0.45;",
    "double g_fs_delay_improvement_min = 0.25;",
    "regular improvement default"
)


# 2) Add FAST_DELAY globals.
anchor = '''double g_fs_delay_improvement_min = 0.25;

// Brake/recovery state machine.
'''
replacement = '''double g_fs_delay_improvement_min = 0.25;

// Channel C: FAST high-confidence abnormal-delay trigger.
// Unlike the regular delay detector, this channel needs only one FRESH odom
// frame, but it also requires a large current cmd/odom mismatch.
bool g_fs_fast_delay_enable = true;
double g_fs_fast_delay_min_ms = 300.0;
double g_fs_fast_delay_score_max = 2.20;
double g_fs_fast_delay_ratio_max = 0.82;
double g_fs_fast_delay_improvement_min = 0.30;
double g_fs_fast_mismatch_linear = 0.30;
double g_fs_fast_mismatch_angular = 1.00;

// Brake/recovery state machine.
'''
replace_once(anchor, replacement, "FAST_DELAY global parameters")


# 3) ROS parameter defaults and FAST_DELAY parameters.
replace_once(
    'pravite_nh.param("failsafe_reverse_confirm_cycles", g_fs_reverse_confirm_cycles, 6);',
    'pravite_nh.param("failsafe_reverse_confirm_cycles", g_fs_reverse_confirm_cycles, 5);',
    "reverse parameter default"
)

replace_once(
    'pravite_nh.param("failsafe_delay_eval_window_ms", g_fs_delay_eval_window_ms, 300.0);',
    'pravite_nh.param("failsafe_delay_eval_window_ms", g_fs_delay_eval_window_ms, 220.0);',
    "eval window parameter default"
)

replace_once(
    'pravite_nh.param("failsafe_delay_fault_min_ms", g_fs_delay_fault_min_ms, 300.0);',
    'pravite_nh.param("failsafe_delay_fault_min_ms", g_fs_delay_fault_min_ms, 250.0);',
    "fault delay parameter default"
)

replace_once(
    'pravite_nh.param("failsafe_delay_vs_normal_ratio_max", g_fs_delay_vs_normal_ratio_max, 0.72);',
    'pravite_nh.param("failsafe_delay_vs_normal_ratio_max", g_fs_delay_vs_normal_ratio_max, 0.85);',
    "ratio parameter default"
)

old = '''  pravite_nh.param("failsafe_delay_improvement_min", g_fs_delay_improvement_min, 0.45);

  pravite_nh.param("failsafe_min_brake_ms", g_fs_min_brake_ms, 200.0);
'''
new = '''  pravite_nh.param("failsafe_delay_improvement_min", g_fs_delay_improvement_min, 0.25);

  // FAST_DELAY: high-confidence single-fresh-odom trigger.
  pravite_nh.param("failsafe_fast_delay_enable", g_fs_fast_delay_enable, true);
  pravite_nh.param("failsafe_fast_delay_min_ms", g_fs_fast_delay_min_ms, 300.0);
  pravite_nh.param("failsafe_fast_delay_score_max", g_fs_fast_delay_score_max, 2.20);
  pravite_nh.param("failsafe_fast_delay_ratio_max", g_fs_fast_delay_ratio_max, 0.82);
  pravite_nh.param("failsafe_fast_delay_improvement_min", g_fs_fast_delay_improvement_min, 0.30);
  pravite_nh.param("failsafe_fast_mismatch_linear", g_fs_fast_mismatch_linear, 0.30);
  pravite_nh.param("failsafe_fast_mismatch_angular", g_fs_fast_mismatch_angular, 1.00);

  pravite_nh.param("failsafe_min_brake_ms", g_fs_min_brake_ms, 200.0);
'''
replace_once(old, new, "FAST_DELAY ROS parameters")


# 4) New diagnostic variable.
old = '''      bool fs_delay_fault_candidate = false;
      double fs_current_mismatch_linear = -1.0;
'''
new = '''      bool fs_delay_fault_candidate = false;
      bool fs_fast_delay_candidate = false;
      double fs_current_mismatch_linear = -1.0;
'''
replace_once(old, new, "FAST_DELAY diagnostic variable")


# 5) Calculate FAST_DELAY after the V4 delay fit has been evaluated.
old = '''                fs_delay_fault_candidate =
                    best_delay >= g_fs_delay_fault_min_ms &&
                    best_score <= g_fs_delay_fault_score_max &&
                    best_score <= best_normal_score * g_fs_delay_vs_normal_ratio_max &&
                    fs_delay_improvement >= g_fs_delay_improvement_min;
'''
new = '''                fs_delay_fault_candidate =
                    best_delay >= g_fs_delay_fault_min_ms &&
                    best_score <= g_fs_delay_fault_score_max &&
                    best_score <= best_normal_score * g_fs_delay_vs_normal_ratio_max &&
                    fs_delay_improvement >= g_fs_delay_improvement_min;

                // FAST_DELAY is stricter about instantaneous mismatch, but does
                // not wait for a second odom confirmation.
                fs_fast_delay_candidate =
                    g_fs_fast_delay_enable &&
                    best_delay >= g_fs_fast_delay_min_ms &&
                    best_score <= g_fs_fast_delay_score_max &&
                    best_score <= best_normal_score * g_fs_fast_delay_ratio_max &&
                    fs_delay_improvement >= g_fs_fast_delay_improvement_min &&
                    (fs_current_mismatch_linear >= g_fs_fast_mismatch_linear ||
                     fs_current_mismatch_angular >= g_fs_fast_mismatch_angular);
'''
replace_once(old, new, "FAST_DELAY candidate calculation")


# 6) Trigger FAST_DELAY on one fresh odom frame.
old = '''        // Delay fault confirmation counts only NEW odom frames.
        if (fs_odom.header.stamp != g_fs_last_detection_odom_stamp)
        {
          g_fs_last_detection_odom_stamp = fs_odom.header.stamp;
          if (fs_delay_fault_candidate)
            ++g_fs_delay_confirm_count;
          else
            g_fs_delay_confirm_count = 0;
        }

        if (fs_trigger_reason == NULL &&
            g_fs_delay_confirm_count >= g_fs_delay_confirm_odom_frames)
          fs_trigger_reason = "ABNORMAL_COMMAND_TO_ODOM_DELAY";
'''
new = '''        // Delay fault confirmation counts only NEW odom frames.
        bool fs_new_detection_odom = false;
        if (fs_odom.header.stamp != g_fs_last_detection_odom_stamp)
        {
          fs_new_detection_odom = true;
          g_fs_last_detection_odom_stamp = fs_odom.header.stamp;

          if (fs_delay_fault_candidate)
            ++g_fs_delay_confirm_count;
          else
            g_fs_delay_confirm_count = 0;
        }

        // High-confidence path: one fresh odom frame is enough.
        if (fs_trigger_reason == NULL &&
            fs_new_detection_odom &&
            fs_fast_delay_candidate)
          fs_trigger_reason = "FAST_ABNORMAL_COMMAND_TO_ODOM_DELAY";

        // Medium-confidence path: retain two-frame confirmation.
        if (fs_trigger_reason == NULL &&
            g_fs_delay_confirm_count >= g_fs_delay_confirm_odom_frames)
          fs_trigger_reason = "ABNORMAL_COMMAND_TO_ODOM_DELAY";
'''
replace_once(old, new, "FAST_DELAY trigger path")


# 7) Trigger log: expose fast flag and current mismatch.
old = '''        ROS_ERROR("[BASE-FAILSAFE][TRIGGER] reason=%s count=%llu cmd=(%.3f,%.3f,%.3f) odom=(%.3f,%.3f,%.3f) odom_age_ms=%.2f reverse_count=%d delay_count=%d best_delay_ms=%.1f best_score=%.3f normal_delay_ms=%.1f normal_score=%.3f improvement=%.3f eval_n=%d cmd_span=(%.3f,%.3f)",
'''
new = '''        ROS_ERROR("[BASE-FAILSAFE][TRIGGER] reason=%s count=%llu cmd=(%.3f,%.3f,%.3f) odom=(%.3f,%.3f,%.3f) odom_age_ms=%.2f reverse_count=%d delay_count=%d fast_delay=%d best_delay_ms=%.1f best_score=%.3f normal_delay_ms=%.1f normal_score=%.3f improvement=%.3f mismatch=(%.3f,%.3f) eval_n=%d cmd_span=(%.3f,%.3f)",
'''
replace_once(old, new, "trigger log format")

old = '''                  g_fs_reverse_count,
                  g_fs_delay_confirm_count,
                  fs_best_delay_ms,
                  fs_best_delay_score,
                  fs_best_normal_delay_ms,
                  fs_best_normal_score,
                  fs_delay_improvement,
                  fs_eval_sample_count,
                  fs_cmd_span_linear,
                  fs_cmd_span_angular);
'''
new = '''                  g_fs_reverse_count,
                  g_fs_delay_confirm_count,
                  fs_fast_delay_candidate ? 1 : 0,
                  fs_best_delay_ms,
                  fs_best_delay_score,
                  fs_best_normal_delay_ms,
                  fs_best_normal_score,
                  fs_delay_improvement,
                  fs_current_mismatch_linear,
                  fs_current_mismatch_angular,
                  fs_eval_sample_count,
                  fs_cmd_span_linear,
                  fs_cmd_span_angular);
'''
replace_once(old, new, "trigger log arguments")


# 8) tx_diag: preserve V4 fields [0..41], append [42].
old = '''        // [41] normal_score_minus_best_score
        std_msgs::Float64MultiArray diag_msg;
        diag_msg.data.resize(42);
'''
new = '''        // [41] normal_score_minus_best_score
        // [42] fast_delay_candidate
        std_msgs::Float64MultiArray diag_msg;
        diag_msg.data.resize(43);
'''
replace_once(old, new, "tx_diag layout")

old = '''        diag_msg.data[41] = fs_delay_improvement;
        g_tx_diag_pub.publish(diag_msg);
'''
new = '''        diag_msg.data[41] = fs_delay_improvement;
        diag_msg.data[42] = fs_fast_delay_candidate ? 1.0 : 0.0;
        g_tx_diag_pub.publish(diag_msg);
'''
replace_once(old, new, "tx_diag FAST_DELAY assignment")


# 9) SUMMARY: expose FAST_DELAY and mismatch.
old = '''fs_state=%d fs_triggers=%llu fs_rev=%d fs_delay=%d delay_fault=%d eval_ms=%.1f cmd_span=(%.2f,%.2f) best_delay_ms=%.1f best_score=%.2f normal_delay_ms=%.1f normal_score=%.2f improve=%.2f eval_n=%d odom_age_ms=%.1f'''
new = '''fs_state=%d fs_triggers=%llu fs_rev=%d fs_delay=%d delay_fault=%d fast_delay=%d eval_ms=%.1f cmd_span=(%.2f,%.2f) best_delay_ms=%.1f best_score=%.2f normal_delay_ms=%.1f normal_score=%.2f improve=%.2f mismatch=(%.2f,%.2f) eval_n=%d odom_age_ms=%.1f'''
replace_once(old, new, "SUMMARY format")

old = '''                            g_fs_delay_confirm_count,
                            fs_delay_fault_candidate ? 1 : 0,
                            fs_eval_span_ms,
                            fs_cmd_span_linear,
                            fs_cmd_span_angular,
                            fs_best_delay_ms,
                            fs_best_delay_score,
                            fs_best_normal_delay_ms,
                            fs_best_normal_score,
                            fs_delay_improvement,
                            fs_eval_sample_count,
                            fs_odom_age_ms);
'''
new = '''                            g_fs_delay_confirm_count,
                            fs_delay_fault_candidate ? 1 : 0,
                            fs_fast_delay_candidate ? 1 : 0,
                            fs_eval_span_ms,
                            fs_cmd_span_linear,
                            fs_cmd_span_angular,
                            fs_best_delay_ms,
                            fs_best_delay_score,
                            fs_best_normal_delay_ms,
                            fs_best_normal_score,
                            fs_delay_improvement,
                            fs_current_mismatch_linear,
                            fs_current_mismatch_angular,
                            fs_eval_sample_count,
                            fs_odom_age_ms);
'''
replace_once(old, new, "SUMMARY arguments")


# 10) Version labels last.
text = text.replace("BASE-FAILSAFE V4", "BASE-FAILSAFE V4.1")

required = (
    "FAST_ABNORMAL_COMMAND_TO_ODOM_DELAY",
    "failsafe_fast_delay_enable",
    "fs_fast_delay_candidate",
    "diag_msg.data.resize(43)",
    'failsafe_delay_eval_window_ms", g_fs_delay_eval_window_ms, 220.0',
    'failsafe_delay_fault_min_ms", g_fs_delay_fault_min_ms, 250.0',
)
for token in required:
    if token not in text:
        raise SystemExit(f"V4.1 patch incomplete, missing token: {token}")

path.write_text(text, encoding="utf-8")

print(f"BASE-FAILSAFE V4.1 written to: {path}")
print(f"V4 backup: {backup}")
print("Next:")
print("  catkin_make --pkg ucar_controller")
print("  source devel/setup.bash")