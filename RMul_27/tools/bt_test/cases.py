#!/usr/bin/env python3
"""逐节点运行测试用例。

每个 case = 一棵最小树 + mock 配置 + 断言。XML 由本文件生成，结果在 results/。
约定：
  - PASS/FAIL 探针：Fallback{ Sequence{前置节点, 被测条件, PrintMessage PASS}, PrintMessage FAIL }
  - 断言只看 warmup 之后的输出，避开启动瞬态
  - 动作节点用 Sequence 包一层 PrintMessage 作为“执行到了”的证据
"""


def doc(body):
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<root BTCPP_format="4" main_tree_to_execute="T">\n'
        f'  <BehaviorTree ID="T">\n{body}\n  </BehaviorTree>\n'
        '</root>\n'
    )


def probe(pre, cond):
    """前置节点 + 条件，成功打 PASS、失败打 FAIL。"""
    return doc(f"""    <Fallback>
      <Sequence>
{pre}
{cond}
        <PrintMessage message_to_print="CASE_RESULT:PASS"/>
      </Sequence>
      <PrintMessage message_to_print="CASE_RESULT:FAIL"/>
    </Fallback>""")


def action_probe(node_xml):
    return doc(f"""    <Sequence>
{node_xml}
      <PrintMessage message_to_print="CASE_RESULT:PASS"/>
    </Sequence>""")


CASES = []


def add(name, xml, config=None, **kw):
    CASES.append({"name": name, "xml": xml, "config": config or {}, **kw})


# ---------------------------------------------------------------- 订阅 + 消费者
add(
    "sub_game_status_pos",
    probe(
        '        <SubGameStatus topic_name="/game_status" game_status="{game_status}"/>',
        '        <IsGameTime message="{game_status}" game_progress="4" '
        'lower_remain_time="0" higher_remain_time="420"/>',
    ),
    stdout_all=["CASE_RESULT:PASS"],
    stdout_none=["CASE_RESULT:FAIL"],
    warmup=1.2,
)

add(
    "is_game_time_neg_window",
    probe(
        '        <SubGameStatus topic_name="/game_status" game_status="{game_status}"/>',
        '        <IsGameTime message="{game_status}" game_progress="4" '
        'lower_remain_time="0" higher_remain_time="100"/>',
    ),
    stdout_all=["CASE_RESULT:FAIL"],
    stdout_none=["CASE_RESULT:PASS"],
    warmup=1.2,
)

add(
    "sub_robot_status_pos",
    probe(
        '        <SubRobotStatus topic_name="/robot_status" robot_status="{robot_status}"/>',
        '        <IsStatusOK message="{robot_status}" hp_threshold="100" '
        'heat_threshold="10" barrel_heat_limit_threshold="400"/>',
    ),
    stdout_all=["CASE_RESULT:PASS"],
    stdout_none=["CASE_RESULT:FAIL"],
    warmup=1.2,
)

add(
    "is_status_ok_neg_hp",
    probe(
        '        <SubRobotStatus topic_name="/robot_status" robot_status="{robot_status}"/>',
        '        <IsStatusOK message="{robot_status}" hp_threshold="500" '
        'heat_threshold="10" barrel_heat_limit_threshold="400"/>',
    ),
    stdout_all=["CASE_RESULT:FAIL"],
    stdout_none=["CASE_RESULT:PASS"],
    warmup=1.2,
)

add(
    "is_status_ok_missing_message_quirk",
    probe(
        '',
        '        <IsStatusOK hp_threshold="100" heat_threshold="10" '
        'barrel_heat_limit_threshold="400"/>',
    ),
    stdout_all=["CASE_RESULT:PASS"],
    stdout_none=["CASE_RESULT:FAIL"],
    warmup=1.0,
)

# ---------------------------------------------------------------- 发布节点
add(
    "pub_robot_state",
    action_probe(
        '      <PubRobotState topic_name="/robot_state" robot_state="3"/>'
    ),
    topics=[{"topic": "/robot_state", "field": "state", "equals": 3}],
    stdout_all=["CASE_RESULT:PASS"],
)

add(
    "pub_chassis_mode",
    action_probe(
        '      <PubChassisMode topic_name="/chassis_mode" chassis_mode="1"/>'
    ),
    topics=[{"topic": "/chassis_mode", "field": "mode", "equals": 1}],
)

add(
    "pub_auto_aim_mode",
    action_probe(
        '      <PubAutoAimMode topic_name="/auto_aim_mode" auto_aim_mode="3"/>'
    ),
    topics=[{"topic": "/auto_aim_mode", "field": "mode", "equals": 3}],
)

add(
    "pub_super_capacitor",
    action_probe(
        '      <PubSuperCapacitor topic_name="/super_capacitor" super_capacitor_state="1"/>'
    ),
    topics=[{"topic": "/super_capacitor", "field": "state", "equals": 1}],
)

add(
    "pub_enemy_status",
    action_probe(
        '      <PubEnemyStatus topic_name="/enemy_status" invincible_enemy_ids="1,3,5"/>'
    ),
    topics=[{"topic": "/enemy_status", "field": "invincible_enemy_ids", "equals": [1, 3, 5]}],
)

add(
    "pub_autoaim_target",
    action_probe(
        '      <PubAutoaimTarget topic_name="/autoaim_target" target_ids="2,4"/>'
    ),
    topics=[{"topic": "/autoaim_target", "field": "target_ids", "equals": [2, 4]}],
)

add(
    "robot_control",
    action_probe(
        '      <RobotControl stop_gimbal_scan="true" chassis_spin_vel="2.5"/>'
    ),
    topics=[
        {"topic": "/robot_control", "field": "stop_gimbal_scan", "equals": True},
        {"topic": "/robot_control", "field": "chassis_spin_vel", "approx": 2.5, "tol": 0.01},
    ],
)

# ---------------------------------------------------------------- 导航 / 定位
add(
    "get_current_location_move_around",
    doc("""    <Sequence>
      <GetCurrentLocation current_location="{current_location}"
                          target_frame="map"
                          source_frame="gimbal_yaw"/>
      <MoveAround message="{current_location}"
                  expected_dis="0.5"
                  map_topic="/map"
                  goal_clearance="0.0"
                  max_resample="20"
                  allow_unknown="false"
                  goal_pose="{goal_pose}"/>
      <SendGoal goal_pose="{goal_pose}" action_name="navigate_to_pose"/>
      <PrintMessage message_to_print="CASE_RESULT:PASS"/>
    </Sequence>"""),
    goals=[{"x": 0.0, "y": 0.0, "tol": 0.6}],
    stdout_all=["CASE_RESULT:PASS"],
    warmup=1.5,
)

add(
    "send_goal_arrive",
    doc("""    <Sequence>
      <SendGoal goal_pose="1.5;-2.5;0;0;0;0;1" action_name="navigate_to_pose"/>
      <PrintMessage message_to_print="CASE_RESULT:PASS"/>
    </Sequence>"""),
    goals=[{"x": 1.5, "y": -2.5, "tol": 0.05}],
    stdout_all=["CASE_RESULT:PASS", r"\[SendGoal\] Sending goal: \[1\.5, -2\.5\]"],
    warmup=1.2,
)

add(
    "send_goal_abort",
    doc("""    <Fallback>
      <Sequence>
        <SendGoal goal_pose="1.5;-2.5;0;0;0;0;1" action_name="navigate_to_pose"/>
        <PrintMessage message_to_print="CASE_RESULT:PASS"/>
      </Sequence>
      <PrintMessage message_to_print="CASE_RESULT:FAIL"/>
    </Fallback>"""),
    config={"nav": {"mode": "abort"}},
    stdout_all=["CASE_RESULT:FAIL"],
    stdout_none=["CASE_RESULT:PASS"],
    warmup=1.2,
)


# ================================================================ 订阅节点二
SUB_GS = '        <SubGameStatus topic_name="/game_status" game_status="{game_status}"/>'
SUB_HP = '        <SubAllRobotHP topic_name="/robot_hp" robot_hp="{robot_hp}"/>'
SUB_RS = '        <SubRobotStatus topic_name="/robot_status" robot_status="{robot_status}"/>'
SUB_AS = '        <SubAutoAimStatus topic_name="/auto_aim_status" auto_aim_status="{auto_aim_status}"/>'
SUB_CE = '        <SubChassisEnergy topic_name="/chassis_energy" chassis_energy="{chassis_energy}"/>'
SUB_RM = '        <SubRemoteStatus topic_name="/remote_status" remote_status="{remote_status}"/>'
SUB_RUNE = '        <SubRuneStatus topic_name="/rune_status" rune_status="{rune_status}"/>'
SUB_DN = '        <SubDecisionNum topic_name="/decision_num" decision_num="{decision_num}"/>'
SUB_TP = ('        <ForceSuccess>\n'
          '          <SubTargetPos topic_name="/auto_aim_target_pos" target_frame="map" '
          'source_frame="odom" armors="{armors}"/>\n'
          '        </ForceSuccess>')
SUB_GT = ('        <SubGimbalTarget topic_name="/gimbal_target_point" gimbal_frame="gimbal_field" '
          'gimbal_origin_x="0" gimbal_origin_y="0" gimbal_origin_z="0" gimbal_yaw="0" '
          'target_pose="{gimbal_pose}" target_valid="{gimbal_valid}" '
          'target_x="{gimbal_x}" target_y="{gimbal_y}"/>')
GCL = ('        <GetCurrentLocation current_location="{current_location}" '
       'target_frame="map" source_frame="gimbal_yaw"/>')


def add_pair(name, pre, cond, config=None, neg_config=None, extra=None):
    add(name + "_pos", probe(pre, cond), config=config,
        stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"],
        warmup=1.2, **(extra or {}))
    if neg_config is not None or extra is not None:
        pass


def cond_neg(name, pre, cond, config=None, warmup=1.2, duration=4.0):
    add(name, probe(pre, cond), config=config,
        stdout_all=["CASE_RESULT:FAIL"], stdout_none=["CASE_RESULT:PASS"],
        warmup=warmup, duration=duration)


add(
    "sub_all_robot_hp_pos",
    probe(SUB_HP, '        <IsOutpostOK robot_hp="{robot_hp}" hp_threshold="100"/>'),
    stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"], warmup=1.2,
)
cond_neg("is_outpost_ok_neg", SUB_HP,
         '        <IsOutpostOK robot_hp="{robot_hp}" hp_threshold="600"/>')

add(
    "sub_auto_aim_status_pos",
    probe(SUB_AS, '        <IsAutoAimOK message="{auto_aim_status}" target_status="0"/>'),
    stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"], warmup=1.2,
)
cond_neg("is_auto_aim_ok_neg", SUB_AS,
         '        <IsAutoAimOK message="{auto_aim_status}" target_status="1"/>')

add(
    "sub_chassis_energy_pos",
    probe(SUB_CE, '        <IsChassisEnergyOK message="{chassis_energy}"/>'),
    config={"chassis_energy": {"chassis_energy": 1}},
    stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"], warmup=1.2,
)
cond_neg("is_chassis_energy_ok_neg", SUB_CE,
         '        <IsChassisEnergyOK message="{chassis_energy}"/>',
         config={"chassis_energy": {"chassis_energy": 0}})

add(
    "sub_remote_status_pos",
    probe(SUB_RM, '        <IsRemoteStatus message="{remote_status}" target_status="1"/>'),
    stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"], warmup=1.2,
)
cond_neg("is_remote_status_neg", SUB_RM,
         '        <IsRemoteStatus message="{remote_status}" target_status="2"/>')

add(
    "sub_rune_status_pos",
    probe(SUB_RUNE, '        <IsRuneStatus rune_status="{rune_status}" target_count="1"/>'),
    stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"], warmup=1.2,
)
cond_neg("is_rune_status_neg", SUB_RUNE,
         '        <IsRuneStatus rune_status="{rune_status}" target_count="1"/>',
         config={"rune_status": {"rune_count": 1}})

add(
    "sub_target_pos_pos",
    probe(SUB_TP + '\n' + GCL,
          '        <IsDetectEnemy message="{armors}" max_range="12.0" '
          'current_location="{current_location}" debounce_ms="200"/>'),
    stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"], warmup=1.5,
)
cond_neg("is_detect_enemy_neg_range", SUB_TP + '\n' + GCL,
         '        <IsDetectEnemy message="{armors}" max_range="1.0" '
         'current_location="{current_location}" debounce_ms="200"/>',
         warmup=1.5)
cond_neg("is_detect_enemy_no_target", SUB_TP + '\n' + GCL,
         '        <IsDetectEnemy message="{armors}" max_range="12.0" '
         'current_location="{current_location}" debounce_ms="200"/>',
         config={"armors": {"count": 0}}, warmup=1.5)

add(
    "sub_gimbal_target_pos",
    probe(SUB_GT,
          '        <IsGimbalTargetValid message="{gimbal_valid}" target_x="{gimbal_x}" '
          'target_y="{gimbal_y}" target_timeout_ms="10000"/>'),
    stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"], warmup=1.5,
)
cond_neg("is_gimbal_target_valid_timeout", SUB_GT,
         '        <IsGimbalTargetValid message="{gimbal_valid}" target_x="{gimbal_x}" '
         'target_y="{gimbal_y}" target_timeout_ms="500"/>',
         warmup=1.5, duration=3.5)

add(
    "is_gimbal_target_in_zone_pos",
    probe(SUB_GT,
          '        <IsGimbalTargetInZone target_x="{gimbal_x}" target_y="{gimbal_y}" '
          'x1="2" y1="3" x2="4" y2="3" x3="4" y3="5" x4="2" y4="5"/>'),
    stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"], warmup=1.5,
)
cond_neg("is_gimbal_target_in_zone_neg", SUB_GT,
         '        <IsGimbalTargetInZone target_x="{gimbal_x}" target_y="{gimbal_y}" '
         'x1="0" y1="0" x2="1" y2="0" x3="1" y3="1" x4="0" y4="1"/>',
         warmup=1.5)

add(
    "is_target_in_zone_pos",
    probe(SUB_TP,
          '        <IsTargetInZone armors="{armors}" '
          'x1="4" y1="1" x2="6" y2="1" x3="6" y3="3" x4="4" y4="3"/>'),
    stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"], warmup=1.5,
)
cond_neg("is_target_in_zone_neg", SUB_TP,
         '        <IsTargetInZone armors="{armors}" '
         'x1="0" y1="0" x2="1" y2="0" x3="1" y3="1" x4="0" y4="1"/>',
         warmup=1.5)

# ---------------------------------------------------------------- 条件节点二
add(
    "is_base_status_ok_pos",
    probe(SUB_HP, '        <IsBaseStatusOK robot_hp="{robot_hp}" hp_threshold="100"/>'),
    stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"], warmup=1.2,
)
cond_neg("is_base_status_ok_neg", SUB_HP,
         '        <IsBaseStatusOK robot_hp="{robot_hp}" hp_threshold="6000"/>')

add(
    "is_enemy_outpost_ok_pos",
    probe(SUB_HP, '        <IsEnemyOutpostOK robot_hp="{robot_hp}" hp_threshold="100"/>'),
    stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"], warmup=1.2,
)
cond_neg("is_enemy_outpost_ok_neg", SUB_HP,
         '        <IsEnemyOutpostOK robot_hp="{robot_hp}" hp_threshold="500"/>')

add(
    "is_base_attacked_alive",
    probe(SUB_HP.replace('robot_hp="{robot_hp}"', 'robot_hp="{all_robot_hp}"'),
          '        <IsBaseAttacked all_robot_hp="{all_robot_hp}" hp_threshold="10" '
          'timeout_msec="10000"/>'),
    stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"], warmup=1.5,
)

add(
    "is_attacked_sawtooth",
    probe(SUB_RS, '        <IsAttacked message="{robot_status}" hp_threshold="10" '
                  'timeout_msec="10000"/>'),
    config={"robot_status": {"hp_pattern": "sawtooth"}},
    min_count_any={"pattern": "CASE_RESULT:PASS", "min": 1},
    warmup=1.0, duration=5.0,
)

add(
    "is_semantic_pos",
    probe('', '        <IsSemantic topic_name="/terrain_zone" semantic="tunnel" '
              'received="{semantic_now}"/>'),
    config={"terrain_zone": {"data": "tunnel"}},
    stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"], warmup=1.2,
)
cond_neg("is_semantic_neg", '',
         '        <IsSemantic topic_name="/terrain_zone" semantic="high_ground" '
         'received="{semantic_now}"/>',
         config={"terrain_zone": {"data": "tunnel"}})

add(
    "is_time_to_bullet_pos",
    probe(SUB_GS, '        <IsTimeToBullet message="{game_status}" game_progress="4" '
                  'time_to_bullet="300 200 100"/>'),
    stdout_all=["CASE_RESULT:PASS"], stdout_none=["CASE_RESULT:FAIL"], warmup=1.2,
)
cond_neg("is_time_to_bullet_neg", SUB_GS,
         '        <IsTimeToBullet message="{game_status}" game_progress="4" '
         'time_to_bullet="299 201"/>')

add(
    "is_friend_ok_missing_message",
    probe('', '        <IsFriendOK message="{robot_hp}" friend_color="red"/>'),
    stdout_all=["CASE_RESULT:FAIL"], stdout_none=["CASE_RESULT:PASS"], warmup=0.8,
)

add(
    "is_rfid_detected_missing",
    probe('', '        <IsRfidDetected key_port="{rfid}" center_gain_point="true"/>'),
    stdout_all=["CASE_RESULT:FAIL"], stdout_none=["CASE_RESULT:PASS"], warmup=0.8,
)

# ---------------------------------------------------------------- 控制/装饰/工具
add(
    "sub_decision_num_action_selector",
    doc("""    <Sequence>
{SUB_DN}
      <ActionSelector decision_num="{decision_num}" cooldown_ms="1000">
        <PrintMessage message_to_print="CHILD1"/>
        <PrintMessage message_to_print="CHILD2"/>
      </ActionSelector>
    </Sequence>""".replace("{SUB_DN}", SUB_DN)),
    config={"decision_num": {"decision_num": 2}},
    stdout_all=["CHILD2"], stdout_none=["CHILD1"], warmup=1.2,
)

add(
    "action_selector_out_of_range",
    doc("""    <Fallback>
      <Sequence>
{SUB_DN}
        <ActionSelector decision_num="{decision_num}" cooldown_ms="500">
          <PrintMessage message_to_print="CHILD1"/>
          <PrintMessage message_to_print="CHILD2"/>
        </ActionSelector>
        <PrintMessage message_to_print="CASE_RESULT:PASS"/>
      </Sequence>
      <PrintMessage message_to_print="CASE_RESULT:FAIL"/>
    </Fallback>""".replace("{SUB_DN}", SUB_DN)),
    config={"decision_num": {"decision_num": 9}},
    stdout_all=["CASE_RESULT:FAIL", "无有效动作号"],
    stdout_none=["CASE_RESULT:PASS"], warmup=1.2,
)

add(
    "action_selector_cooldown",
    doc("""    <Sequence>
{SUB_DN}
      <ActionSelector decision_num="{decision_num}" cooldown_ms="1500">
        <PrintMessage message_to_print="CHILD1"/>
      </ActionSelector>
    </Sequence>""".replace("{SUB_DN}", SUB_DN)),
    config={"decision_num": {"decision_num": 1}},
    counts=[{"pattern": "CHILD1", "min": 2, "max": 6}],
    warmup=1.0, duration=5.0,
)

add(
    "decision_switch_1",
    doc("""    <Sequence>
{SUB_DN}
      <DecisionSwitch decision_num="{decision_num}">
        <PrintMessage message_to_print="CHILD1"/>
        <PrintMessage message_to_print="CHILD2"/>
      </DecisionSwitch>
    </Sequence>""".replace("{SUB_DN}", SUB_DN)),
    config={"decision_num": {"decision_num": 1}},
    stdout_all=["CHILD1"], stdout_none=["CHILD2"], warmup=1.2,
)
cond_neg("decision_switch_3", SUB_DN,
         '        <DecisionSwitch decision_num="{decision_num}">'
         '<PrintMessage message_to_print="CHILD1"/>'
         '<PrintMessage message_to_print="CHILD2"/></DecisionSwitch>',
         config={"decision_num": {"decision_num": 3}}, warmup=1.2)

add(
    "rate_controller_2hz",
    doc("""    <KeepRunningUntilFailure>
      <RateController hz="2">
        <PrintMessage message_to_print="RATE_TICK"/>
      </RateController>
    </KeepRunningUntilFailure>"""),
    counts=[{"pattern": "RATE_TICK", "min": 5, "max": 20}],
    warmup=0.8, duration=5.0,
)

add(
    "wait_duration_800ms",
    doc("""    <Sequence>
      <PrintMessage message_to_print="CASE_START"/>
      <WaitDuration duration_msec="800"/>
      <PrintMessage message_to_print="CASE_PASS"/>
    </Sequence>"""),
    timing={"start": "CASE_START", "end": "CASE_PASS", "min_ms": 650, "max_ms": 1100},
    warmup=0.8, duration=4.0,
)

add(
    "print_message_literal",
    doc('    <PrintMessage message_to_print="HELLO_BT_TEST"/>'),
    stdout_all=["HELLO_BT_TEST"], warmup=0.5, duration=2.0,
)

add(
    "set_get_blackboard_true",
    doc("""    <Sequence>
      <RMSetBlackboard key="flag" value="true"/>
      <GetBlackboard key="flag"/>
      <PrintMessage message_to_print="CASE_RESULT:PASS"/>
    </Sequence>"""),
    stdout_all=["CASE_RESULT:PASS"], warmup=0.8, duration=2.5,
)
cond_neg("set_get_blackboard_false",
         '        <RMSetBlackboard key="flag" value="false"/>',
         '        <GetBlackboard key="flag"/>', warmup=0.8, duration=2.5)
cond_neg("get_blackboard_missing", '', '        <GetBlackboard key="nope"/>',
         warmup=0.8, duration=2.5)

add(
    "rm_set_blackboard_missing_key",
    doc("""    <Fallback>
      <Sequence>
        <RMSetBlackboard value="true"/>
        <PrintMessage message_to_print="CASE_RESULT:PASS"/>
      </Sequence>
      <PrintMessage message_to_print="CASE_RESULT:FAIL"/>
    </Fallback>"""),
    stdout_all=["CASE_RESULT:FAIL"], stdout_none=["CASE_RESULT:PASS"], warmup=0.8,
)

# ---------------------------------------------------------------- 导航节点二
add(
    "pursue_goal_static_enemy",
    doc("""    <ReactiveSequence>
{SUB_TP}
{GCL}
      <PursueGoal armors="{armors}" current_location="{current_location}"
                  pursue_radius="2.5" update_interval="0.5" min_goal_change="1.0"
                  target_frame="map" map_topic="/map" wall_check_clearance="0.0"
                  goal_pose="{pursue_goal}"/>
      <SendGoal goal_pose="{pursue_goal}" action_name="navigate_to_pose"/>
      <PrintMessage message_to_print="CASE_RESULT:PASS"/>
    </ReactiveSequence>""".replace("{SUB_TP}", SUB_TP).replace("{GCL}", GCL)),
    goals=[{"x": 2.68, "y": 1.07, "tol": 0.25}],
    stdout_all=["CASE_RESULT:PASS"], warmup=1.5, duration=5.0,
)

add(
    "pursue_enemy_sends_goal",
    doc("""    <Sequence>
{SUB_TP}
{GCL}
      <PursueEnemy armors="{armors}" current_location="{current_location}"
                   pursue_radius="2.0" max_pursuit_points="1" update_interval="0.2"
                   action_name="navigate_to_pose"/>
      <PrintMessage message_to_print="CASE_RESULT:PASS"/>
    </Sequence>""".replace("{SUB_TP}", SUB_TP).replace("{GCL}", GCL)),
    goals=[{"x": 3.14, "y": 1.26, "tol": 0.3}],
    warmup=1.5, duration=5.0,
)

add(
    "send_goal_bad_format",
    doc("""    <Sequence>
      <SendGoal goal_pose="abc" action_name="navigate_to_pose"/>
    </Sequence>"""),
    expect_load_failure=r"(invalid goal_pose format|RuntimeError|terminate)",
    duration=2.0,
)

# ---------------------------------------------------------------- 未注册节点
add(
    "unregistered_not",
    doc("    <Not><AlwaysSuccess/></Not>"),
    expect_load_failure=r"(not registered|Could not find|Building tree failed|RuntimeError)",
    duration=2.0,
)
add(
    "unregistered_sub_armors",
    doc('    <SubArmors topic_name="/auto_aim_target_pos" armors="{armors}"/>'),
    expect_load_failure=r"(not registered|Could not find|Building tree failed|RuntimeError)",
    duration=2.0,
)
add(
    "unregistered_is_battery_ok",
    doc('    <IsBatteryOK message="{battery}"/>'),
    expect_load_failure=r"(not registered|Could not find|Building tree failed|RuntimeError)",
    duration=2.0,
)

# ---------------------------------------------------------------- 2027 主树 12 动作
MAIN_XML = "rm_behavior_tree/rm_behavior_tree/config/rmul_2027_sentry.xml"


def e2e(name, num, goals=None, topics=None, no_goal=False, stdout_all=None, duration=5.0):
    add(
        name,
        "",  # xml_file 优先
        xml_file=MAIN_XML,
        config={"decision_num": {"decision_num": num}},
        goals=goals or [],
        topics=topics or [],
        no_goal=no_goal,
        stdout_all=stdout_all or [],
        warmup=1.6,
        duration=duration,
    )


E2E_COMMON_1 = [
    {"topic": "/robot_state", "field": "state", "equals": 3},
    {"topic": "/auto_aim_mode", "field": "mode", "equals": 0},
    {"topic": "/chassis_mode", "field": "mode", "equals": 1},
    {"topic": "/robot_control", "field": "stop_gimbal_scan", "equals": False},
    {"topic": "/robot_control", "field": "chassis_spin_vel", "approx": 3.0, "tol": 0.01},
]

e2e("e2e_action01_attack_together", 1,
    goals=[{"x": 2.21, "y": 0.89, "tol": 0.35}], topics=E2E_COMMON_1)

e2e("e2e_action02_defend_together", 2,
    goals=[{"x": -1.80, "y": 2.60, "tol": 0.1}],
    topics=[
        {"topic": "/robot_state", "field": "state", "equals": 2},
        {"topic": "/auto_aim_mode", "field": "mode", "equals": 0},
        {"topic": "/chassis_mode", "field": "mode", "equals": 1},
        {"topic": "/robot_control", "field": "chassis_spin_vel", "approx": 3.0, "tol": 0.01},
    ])

e2e("e2e_action03_home_outpost", 3,
    goals=[{"x": -3.02, "y": 3.90, "tol": 0.1}],
    topics=[
        {"topic": "/auto_aim_mode", "field": "mode", "equals": 3},
        {"topic": "/robot_state", "field": "state", "equals": 3},
        {"topic": "/chassis_mode", "field": "mode", "equals": 1},
        {"topic": "/robot_control", "field": "stop_gimbal_scan", "equals": True},
        {"topic": "/robot_control", "field": "chassis_spin_vel", "approx": 2.0, "tol": 0.01},
    ])

e2e("e2e_action04_away_outpost", 4,
    goals=[{"x": 3.02, "y": -3.90, "tol": 0.1}],
    topics=[{"topic": "/auto_aim_mode", "field": "mode", "equals": 3},
            {"topic": "/robot_state", "field": "state", "equals": 3}])

e2e("e2e_action05_fortress", 5,
    goals=[{"x": 7.43, "y": 0.0, "tol": 0.1}],
    topics=[{"topic": "/robot_state", "field": "state", "equals": 2}])

e2e("e2e_action06_hold_home_outpost", 6,
    goals=[{"x": -3.02, "y": 3.90, "tol": 0.1}],
    topics=[{"topic": "/robot_state", "field": "state", "equals": 2},
            {"topic": "/auto_aim_mode", "field": "mode", "equals": 0}])

e2e("e2e_action07_hold_away_outpost", 7,
    goals=[{"x": 3.02, "y": -3.90, "tol": 0.1}],
    topics=[{"topic": "/robot_state", "field": "state", "equals": 2},
            {"topic": "/auto_aim_mode", "field": "mode", "equals": 0}])

e2e("e2e_action08_last_stand", 8,
    no_goal=True,
    topics=[{"topic": "/robot_state", "field": "state", "equals": 2},
            {"topic": "/chassis_mode", "field": "mode", "equals": 1},
            {"topic": "/robot_control", "field": "chassis_spin_vel", "approx": 3.0, "tol": 0.01}])

e2e("e2e_action09_pursue", 9,
    goals=[{"x": 2.21, "y": 0.89, "tol": 0.35}], topics=E2E_COMMON_1)

e2e("e2e_action10_supply", 10,
    goals=[{"x": -12.60, "y": 5.50, "tol": 0.1}],
    topics=[{"topic": "/robot_state", "field": "state", "equals": 1},
            {"topic": "/robot_state", "field": "state", "equals": 2},
            {"topic": "/chassis_mode", "field": "mode", "equals": 0}])

e2e("e2e_action11_home_outpost_heal", 11,
    goals=[{"x": -3.02, "y": 3.90, "tol": 0.1}],
    topics=[{"topic": "/robot_state", "field": "state", "equals": 1},
            {"topic": "/robot_state", "field": "state", "equals": 2}])

e2e("e2e_action12_away_outpost_heal", 12,
    goals=[{"x": 3.02, "y": -3.90, "tol": 0.1}],
    topics=[{"topic": "/robot_state", "field": "state", "equals": 1},
            {"topic": "/robot_state", "field": "state", "equals": 2}])

e2e("e2e_fallback_action0", 0,
    goals=[{"x": -1.80, "y": 2.60, "tol": 0.1}],
    topics=[{"topic": "/robot_state", "field": "state", "equals": 2}],
    stdout_all=["无有效动作号"])

e2e("e2e_fallback_action99", 99,
    goals=[{"x": -1.80, "y": 2.60, "tol": 0.1}],
    topics=[{"topic": "/robot_state", "field": "state", "equals": 2}],
    stdout_all=["无有效动作号"])
