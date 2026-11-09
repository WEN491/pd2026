#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONTROL__ACTION_SELECTOR_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONTROL__ACTION_SELECTOR_HPP_

#include <chrono>
#include <cstddef>

#include "behaviortree_cpp/control_node.h"
#include "rm_decision_interfaces/msg/decision_num.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 12 路「宏动作选择器」——按动作号选一个子树执行，并把子树当 option / 宏来用。
 *
 * 与同目录 DecisionSwitch 的区别：
 *   - DecisionSwitch 只写了 case 1/2，其余 default → FAILURE，且每 tick 重新 tick 子节点，
 *     本质是「一次性分支」；本节点支持 1..N（N = 子节点数），并处理 RUNNING。
 *   - 子树返回 SUCCESS 时**不向上返回 SUCCESS**，而是保持 RUNNING 并在 cooldown_ms 后再重入，
 *     这样「驻守 / 回血 / 死战」这类没有终点的宏会一直保持，不会被上层每个 tick 重开一次；
 *     也避免 SendGoal 在 10 ms 级循环里反复取消/重发。
 *   - 动作号变化时 halt 旧子树再进新子树，旧子树的 RUNNING 状态（导航目标等）会被正确清理。
 *
 * 动作号来源（按优先级）：
 *   1. input "action_id"（int，>0 生效）——留给雷达 0x0210 的 action_id；
 *   2. input "decision_num"（rm_decision_interfaces/DecisionNum）——拨杆/本地调试。
 *   两者都没有 → FAILURE，由上层 Fallback 落到本地安全策略。
 *
 * 端口：
 *   - action_id    : int，默认 -1；>0 时优先
 *   - decision_num : rm_decision_interfaces/DecisionNum；可选
 *   - cooldown_ms  : int，默认 1000；同一宏完成后保持多久再重入
 */
class ActionSelector : public BT::ControlNode
{
public:
  ActionSelector(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("action_id", -1, "动作号 1..N；>0 时优先于 decision_num"),
      BT::InputPort<rm_decision_interfaces::msg::DecisionNum>(
        "decision_num", "拨杆消息（备选动作号来源）"),
      BT::InputPort<int>("cooldown_ms", 1000, "同一宏完成后保持的时长 / ms，之后重新进入")};
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  /// 解析动作号；返回 -1 表示没有可用来源
  int resolveAction();

  int current_action_ = -1;
  bool waiting_ = false;
  std::chrono::steady_clock::time_point next_tick_at_{};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONTROL__ACTION_SELECTOR_HPP_
