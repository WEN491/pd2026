#include "rm_behavior_tree/plugins/control/action_selector.hpp"

#include <iostream>

#include "behaviortree_cpp/bt_factory.h"

namespace rm_behavior_tree
{

ActionSelector::ActionSelector(const std::string & name, const BT::NodeConfig & config)
: BT::ControlNode(name, config)
{
}

int ActionSelector::resolveAction()
{
  int action_id = -1;
  // 端口声明了默认值 -1，所以 getInput 一般会成功；>0 才认为雷达给了动作号
  if (getInput<int>("action_id", action_id) && action_id > 0) {
    return action_id;
  }

  // 备选：拨杆消息。没收到过消息时黑板里没有这个 key → getInput 失败 → 返回 -1
  auto msg = getInput<rm_decision_interfaces::msg::DecisionNum>("decision_num");
  if (msg) {
    return static_cast<int>(msg->decision_num);
  }
  return -1;
}

BT::NodeStatus ActionSelector::tick()
{
  const int action = resolveAction();
  const int n = static_cast<int>(childrenCount());

  if (action < 1 || action > n) {
    std::cout << "[ActionSelector] 无有效动作号（action_id/decision_num 缺失或越界: " << action
              << "，子节点数 " << n << "）→ FAILURE" << '\n';
    return BT::NodeStatus::FAILURE;
  }

  // 动作号变化：halt 旧子树（清掉导航目标等 RUNNING 状态）再切到新子树
  if (action != current_action_) {
    if (current_action_ >= 1 && current_action_ <= n) {
      haltChild(static_cast<size_t>(current_action_ - 1));
    }
    current_action_ = action;
    waiting_ = false;
  }

  const auto now = std::chrono::steady_clock::now();
  if (waiting_ && now < next_tick_at_) {
    return BT::NodeStatus::RUNNING;
  }
  waiting_ = false;

  int cooldown_ms = 1000;
  getInput<int>("cooldown_ms", cooldown_ms);
  if (cooldown_ms < 0) {
    cooldown_ms = 0;
  }

  const auto status = children_nodes_[static_cast<size_t>(action - 1)]->executeTick();
  switch (status) {
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;

    case BT::NodeStatus::SUCCESS:
      // 宏跑完一轮：保持 RUNNING，冷却后再重入（见头文件说明）
      waiting_ = true;
      next_tick_at_ = now + std::chrono::milliseconds(cooldown_ms);
      return BT::NodeStatus::RUNNING;

    case BT::NodeStatus::FAILURE:
    default:
      // 交给上层 Fallback（本地安全策略）。不缓存 FAILURE，下个 tick 允许重试。
      return BT::NodeStatus::FAILURE;
  }
}

void ActionSelector::halt()
{
  current_action_ = -1;
  waiting_ = false;
  // 统一由基类 halt 掉全部子节点（含 RUNNING 的）
  BT::ControlNode::halt();
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::ActionSelector>("ActionSelector");
}
