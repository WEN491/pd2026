## 用例结果

| # | 用例 | 结果 | 耗时(s) | 备注 |
| --- | --- | --- | --- | --- |
| 1 | `sub_game_status_pos` | PASS | 5.17 |  |
| 2 | `is_game_time_neg_window` | PASS | 5.17 |  |
| 3 | `sub_robot_status_pos` | PASS | 5.17 |  |
| 4 | `is_status_ok_neg_hp` | PASS | 5.17 |  |
| 5 | `is_status_ok_missing_message_quirk` | PASS | 5.17 |  |
| 6 | `pub_robot_state` | PASS | 5.17 |  |
| 7 | `pub_chassis_mode` | PASS | 5.27 |  |
| 8 | `pub_auto_aim_mode` | PASS | 5.27 |  |
| 9 | `pub_super_capacitor` | PASS | 5.17 |  |
| 10 | `pub_enemy_status` | PASS | 5.22 |  |
| 11 | `pub_autoaim_target` | PASS | 5.17 |  |
| 12 | `robot_control` | PASS | 5.17 |  |
| 13 | `get_current_location_move_around` | PASS | 5.17 |  |
| 14 | `send_goal_arrive` | PASS | 5.17 |  |
| 15 | `send_goal_abort` | PASS | 5.27 |  |
| 16 | `sub_all_robot_hp_pos` | PASS | 5.17 |  |
| 17 | `is_outpost_ok_neg` | PASS | 5.17 |  |
| 18 | `sub_auto_aim_status_pos` | PASS | 5.27 |  |
| 19 | `is_auto_aim_ok_neg` | PASS | 5.17 |  |
| 20 | `sub_chassis_energy_pos` | PASS | 5.22 |  |
| 21 | `is_chassis_energy_ok_neg` | PASS | 5.17 |  |
| 22 | `sub_remote_status_pos` | PASS | 5.17 |  |
| 23 | `is_remote_status_neg` | PASS | 5.17 |  |
| 24 | `sub_rune_status_pos` | PASS | 5.17 |  |
| 25 | `is_rune_status_neg` | PASS | 5.22 |  |
| 26 | `sub_target_pos_pos` | PASS | 5.17 |  |
| 27 | `is_detect_enemy_neg_range` | PASS | 5.27 |  |
| 28 | `is_detect_enemy_no_target` | PASS | 5.17 |  |
| 29 | `sub_gimbal_target_pos` | PASS | 5.17 |  |
| 30 | `is_gimbal_target_valid_timeout` | PASS | 5.27 |  |
| 31 | `is_gimbal_target_in_zone_pos` | PASS | 5.17 |  |
| 32 | `is_gimbal_target_in_zone_neg` | PASS | 5.17 |  |
| 33 | `is_target_in_zone_pos` | PASS | 5.27 |  |
| 34 | `is_target_in_zone_neg` | PASS | 5.17 |  |
| 35 | `is_base_status_ok_pos` | PASS | 5.22 |  |
| 36 | `is_base_status_ok_neg` | PASS | 5.22 |  |
| 37 | `is_enemy_outpost_ok_pos` | PASS | 5.17 |  |
| 38 | `is_enemy_outpost_ok_neg` | PASS | 5.17 |  |
| 39 | `is_base_attacked_alive` | PASS | 5.17 |  |
| 40 | `is_attacked_sawtooth` | PASS | 5.17 |  |
| 41 | `is_semantic_pos` | PASS | 5.17 |  |
| 42 | `is_semantic_neg` | PASS | 5.17 |  |
| 43 | `is_time_to_bullet_pos` | PASS | 5.27 |  |
| 44 | `is_time_to_bullet_neg` | PASS | 5.22 |  |
| 45 | `is_friend_ok_missing_message` | PASS | 5.17 |  |
| 46 | `is_rfid_detected_missing` | PASS | 5.22 |  |
| 47 | `sub_decision_num_action_selector` | PASS | 5.17 |  |
| 48 | `action_selector_out_of_range` | PASS | 5.17 |  |
| 49 | `action_selector_cooldown` | PASS | 5.17 |  |
| 50 | `decision_switch_1` | PASS | 5.27 |  |
| 51 | `decision_switch_3` | PASS | 5.22 |  |
| 52 | `rate_controller_2hz` | FAIL | 5.22 | count('RATE_TICK')=410，期望 [5, 20] |
| 53 | `wait_duration_800ms` | PASS | 5.17 |  |
| 54 | `print_message_literal` | PASS | 5.22 |  |
| 55 | `set_get_blackboard_true` | PASS | 5.22 |  |
| 56 | `set_get_blackboard_false` | PASS | 5.27 |  |
| 57 | `get_blackboard_missing` | PASS | 5.27 |  |
| 58 | `rm_set_blackboard_missing_key` | PASS | 5.22 |  |
| 59 | `pursue_goal_static_enemy` | PASS | 5.22 |  |
| 60 | `pursue_enemy_sends_goal` | PASS | 5.17 |  |
| 61 | `send_goal_bad_format` | PASS | 5.01 |  |
| 62 | `unregistered_not` | PASS | 5.01 |  |
| 63 | `unregistered_sub_armors` | PASS | 5.01 |  |
| 64 | `unregistered_is_battery_ok` | PASS | 5.01 |  |
| 65 | `e2e_action01_attack_together` | FAIL | 6.52 | goal 未命中 (2.21, 0.89) ±0.35; 实际 [(3.2, 0.0)] |
| 66 | `e2e_action02_defend_together` | PASS | 5.22 |  |
| 67 | `e2e_action03_home_outpost` | PASS | 5.27 |  |
| 68 | `e2e_action04_away_outpost` | PASS | 5.22 |  |
| 69 | `e2e_action05_fortress` | PASS | 5.22 |  |
| 70 | `e2e_action06_hold_home_outpost` | PASS | 5.22 |  |
| 71 | `e2e_action07_hold_away_outpost` | PASS | 5.22 |  |
| 72 | `e2e_action08_last_stand` | PASS | 5.27 |  |
| 73 | `e2e_action09_pursue` | PASS | 5.27 |  |
| 74 | `e2e_action10_supply` | PASS | 5.22 |  |
| 75 | `e2e_action11_home_outpost_heal` | PASS | 5.22 |  |
| 76 | `e2e_action12_away_outpost_heal` | PASS | 5.27 |  |
| 77 | `e2e_fallback_action0` | PASS | 5.27 |  |
| 78 | `e2e_fallback_action99` | PASS | 5.27 |  |

## 节点结果

| 节点 | 覆盖用例 | 结果 |
| --- | --- | --- |
| `SubGameStatus` | sub_game_status_pos, is_game_time_neg_window | ✅ PASS |
| `SubRobotStatus` | sub_robot_status_pos, is_status_ok_neg_hp | ✅ PASS |
| `SubAllRobotHP` | sub_all_robot_hp_pos, is_outpost_ok_neg | ✅ PASS |
| `SubAutoAimStatus` | sub_auto_aim_status_pos, is_auto_aim_ok_neg | ✅ PASS |
| `SubChassisEnergy` | sub_chassis_energy_pos, is_chassis_energy_ok_neg | ✅ PASS |
| `SubRuneStatus` | sub_rune_status_pos, is_rune_status_neg | ✅ PASS |
| `SubRemoteStatus` | sub_remote_status_pos, is_remote_status_neg | ✅ PASS |
| `SubDecisionNum` | sub_decision_num_action_selector | ✅ PASS |
| `SubTargetPos` | sub_target_pos_pos, is_detect_enemy_neg_range | ✅ PASS |
| `SubGimbalTarget` | sub_gimbal_target_pos, is_gimbal_target_valid_timeout | ✅ PASS |
| `PubRobotState` | pub_robot_state, e2e_action02_defend_together | ✅ PASS |
| `PubChassisMode` | pub_chassis_mode | ✅ PASS |
| `PubAutoAimMode` | pub_auto_aim_mode, e2e_action03_home_outpost | ✅ PASS |
| `PubSuperCapacitor` | pub_super_capacitor | ✅ PASS |
| `PubEnemyStatus` | pub_enemy_status | ✅ PASS |
| `PubAutoaimTarget` | pub_autoaim_target | ✅ PASS |
| `RobotControl` | robot_control | ✅ PASS |
| `SendGoal` | send_goal_arrive, send_goal_abort, send_goal_bad_format | ✅ PASS |
| `MoveAround` | get_current_location_move_around | ✅ PASS |
| `PursueGoal` | pursue_goal_static_enemy | ✅ PASS |
| `PursueEnemy` | pursue_enemy_sends_goal | ✅ PASS |
| `GetCurrentLocation` | get_current_location_move_around | ✅ PASS |
| `WaitDuration` | wait_duration_800ms | ✅ PASS |
| `PrintMessage` | print_message_literal | ✅ PASS |
| `RMSetBlackboard` | set_get_blackboard_true, rm_set_blackboard_missing_key | ✅ PASS |
| `GetBlackboard` | set_get_blackboard_true, set_get_blackboard_false, get_blackboard_missing | ✅ PASS |
| `ActionSelector` | sub_decision_num_action_selector, action_selector_out_of_range, action_selector_cooldown | ✅ PASS |
| `DecisionSwitch` | decision_switch_1, decision_switch_3 | ✅ PASS |
| `RateController` | rate_controller_2hz | ❌ FAIL |
| `IsGameTime` | sub_game_status_pos, is_game_time_neg_window | ✅ PASS |
| `IsStatusOK` | sub_robot_status_pos, is_status_ok_neg_hp, is_status_ok_missing_message_quirk | ✅ PASS |
| `IsOutpostOK` | sub_all_robot_hp_pos, is_outpost_ok_neg | ✅ PASS |
| `IsEnemyOutpostOK` | is_enemy_outpost_ok_pos, is_enemy_outpost_ok_neg | ✅ PASS |
| `IsAutoAimOK` | sub_auto_aim_status_pos, is_auto_aim_ok_neg | ✅ PASS |
| `IsRemoteStatus` | sub_remote_status_pos, is_remote_status_neg | ✅ PASS |
| `IsChassisEnergyOK` | sub_chassis_energy_pos, is_chassis_energy_ok_neg | ✅ PASS |
| `IsRuneStatus` | sub_rune_status_pos, is_rune_status_neg | ✅ PASS |
| `IsAttacked` | is_attacked_sawtooth | ✅ PASS |
| `IsBaseAttacked` | is_base_attacked_alive | ✅ PASS |
| `IsBaseStatusOK` | is_base_status_ok_pos, is_base_status_ok_neg | ✅ PASS |
| `IsDetectEnemy` | sub_target_pos_pos, is_detect_enemy_neg_range, is_detect_enemy_no_target | ✅ PASS |
| `IsFriendOK` | is_friend_ok_missing_message | 🟡 部分（仅失败路径） |
| `IsRfidDetected` | is_rfid_detected_missing | 🟡 部分（仅失败路径） |
| `IsSemantic` | is_semantic_pos, is_semantic_neg | ✅ PASS |
| `IsTimeToBullet` | is_time_to_bullet_pos, is_time_to_bullet_neg | ✅ PASS |
| `IsTargetInZone` | is_target_in_zone_pos, is_target_in_zone_neg | ✅ PASS |
| `IsGimbalTargetInZone` | is_gimbal_target_in_zone_pos, is_gimbal_target_in_zone_neg | ✅ PASS |
| `IsGimbalTargetValid` | sub_gimbal_target_pos, is_gimbal_target_valid_timeout | ✅ PASS |
| `Not` | 未注册 | ⚪ 未注册（加载失败符合预期） |
| `SubArmors` | 未注册 | ⚪ 未注册（加载失败符合预期） |
| `IsBatteryOK` | 未注册 | ⚪ 未注册（加载失败符合预期） |

合计：PASS 76 / FAIL 2 / 共 78
