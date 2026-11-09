#!/usr/bin/env python3
"""把 results/results.json 汇总成 Markdown 表格片段（供人工报告引用）。"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results", "results_all.json")

NODE_CASES = {
    # 订阅
    "SubGameStatus": ["sub_game_status_pos", "is_game_time_neg_window"],
    "SubRobotStatus": ["sub_robot_status_pos", "is_status_ok_neg_hp"],
    "SubAllRobotHP": ["sub_all_robot_hp_pos", "is_outpost_ok_neg"],
    "SubAutoAimStatus": ["sub_auto_aim_status_pos", "is_auto_aim_ok_neg"],
    "SubChassisEnergy": ["sub_chassis_energy_pos", "is_chassis_energy_ok_neg"],
    "SubRuneStatus": ["sub_rune_status_pos", "is_rune_status_neg"],
    "SubRemoteStatus": ["sub_remote_status_pos", "is_remote_status_neg"],
    "SubDecisionNum": ["sub_decision_num_action_selector"],
    "SubTargetPos": ["sub_target_pos_pos", "is_detect_enemy_neg_range"],
    "SubGimbalTarget": ["sub_gimbal_target_pos", "is_gimbal_target_valid_timeout"],
    # 发布
    "PubRobotState": ["pub_robot_state", "e2e_action02_defend_together"],
    "PubChassisMode": ["pub_chassis_mode"],
    "PubAutoAimMode": ["pub_auto_aim_mode", "e2e_action03_home_outpost"],
    "PubSuperCapacitor": ["pub_super_capacitor"],
    "PubEnemyStatus": ["pub_enemy_status"],
    "PubAutoaimTarget": ["pub_autoaim_target"],
    "RobotControl": ["robot_control"],
    # 导航/定位
    "SendGoal": ["send_goal_arrive", "send_goal_abort", "send_goal_bad_format"],
    "MoveAround": ["get_current_location_move_around"],
    "PursueGoal": ["pursue_goal_static_enemy"],
    "PursueEnemy": ["pursue_enemy_sends_goal"],
    "GetCurrentLocation": ["get_current_location_move_around"],
    # 工具/控制
    "WaitDuration": ["wait_duration_800ms"],
    "PrintMessage": ["print_message_literal"],
    "RMSetBlackboard": ["set_get_blackboard_true", "rm_set_blackboard_missing_key"],
    "GetBlackboard": ["set_get_blackboard_true", "set_get_blackboard_false", "get_blackboard_missing"],
    "ActionSelector": ["sub_decision_num_action_selector", "action_selector_out_of_range",
                       "action_selector_cooldown"],
    "DecisionSwitch": ["decision_switch_1", "decision_switch_3"],
    "RateController": ["rate_controller_2hz"],
    # 条件
    "IsGameTime": ["sub_game_status_pos", "is_game_time_neg_window"],
    "IsStatusOK": ["sub_robot_status_pos", "is_status_ok_neg_hp", "is_status_ok_missing_message_quirk"],
    "IsOutpostOK": ["sub_all_robot_hp_pos", "is_outpost_ok_neg"],
    "IsEnemyOutpostOK": ["is_enemy_outpost_ok_pos", "is_enemy_outpost_ok_neg"],
    "IsAutoAimOK": ["sub_auto_aim_status_pos", "is_auto_aim_ok_neg"],
    "IsRemoteStatus": ["sub_remote_status_pos", "is_remote_status_neg"],
    "IsChassisEnergyOK": ["sub_chassis_energy_pos", "is_chassis_energy_ok_neg"],
    "IsRuneStatus": ["sub_rune_status_pos", "is_rune_status_neg"],
    "IsAttacked": ["is_attacked_sawtooth"],
    "IsBaseAttacked": ["is_base_attacked_alive"],
    "IsBaseStatusOK": ["is_base_status_ok_pos", "is_base_status_ok_neg"],
    "IsDetectEnemy": ["sub_target_pos_pos", "is_detect_enemy_neg_range", "is_detect_enemy_no_target"],
    "IsFriendOK": ["is_friend_ok_missing_message"],
    "IsRfidDetected": ["is_rfid_detected_missing"],
    "IsSemantic": ["is_semantic_pos", "is_semantic_neg"],
    "IsTimeToBullet": ["is_time_to_bullet_pos", "is_time_to_bullet_neg"],
    "IsTargetInZone": ["is_target_in_zone_pos", "is_target_in_zone_neg"],
    "IsGimbalTargetInZone": ["is_gimbal_target_in_zone_pos", "is_gimbal_target_in_zone_neg"],
    "IsGimbalTargetValid": ["sub_gimbal_target_pos", "is_gimbal_target_valid_timeout"],
    # 未注册
    "Not": [],
    "SubArmors": [],
    "IsBatteryOK": [],
}

PARTIAL = {"IsFriendOK", "IsRfidDetected"}
UNREGISTERED = {"Not", "SubArmors", "IsBatteryOK"}


def main():
    data = json.load(open(RES, encoding="utf-8"))
    by_name = {r["name"]: r for r in data}

    print("## 用例结果\n")
    print("| # | 用例 | 结果 | 耗时(s) | 备注 |")
    print("| --- | --- | --- | --- | --- |")
    for i, r in enumerate(data, 1):
        note = "" if not r["errors"] else "<br>".join(r["errors"])
        print(f"| {i} | `{r['name']}` | {r['status']} | {r['duration']} | {note} |")

    print("\n## 节点结果\n")
    print("| 节点 | 覆盖用例 | 结果 |")
    print("| --- | --- | --- |")
    for node, cases in NODE_CASES.items():
        if node in UNREGISTERED:
            print(f"| `{node}` | 未注册 | ⚪ 未注册（加载失败符合预期） |")
            continue
        if not cases:
            print(f"| `{node}` | — | ⚪ 未覆盖 |")
            continue
        rs = [by_name.get(c) for c in cases if by_name.get(c)]
        bad = [r for r in rs if r["status"] != "PASS"]
        if bad:
            print(f"| `{node}` | {', '.join(cases)} | ❌ FAIL |")
        elif node in PARTIAL:
            print(f"| `{node}` | {', '.join(cases)} | 🟡 部分（仅失败路径） |")
        else:
            print(f"| `{node}` | {', '.join(cases)} | ✅ PASS |")

    vals = [r["status"] for r in data]
    print(f"\n合计：PASS {vals.count('PASS')} / FAIL {vals.count('FAIL')} / 共 {len(data)}")


if __name__ == "__main__":
    main()
