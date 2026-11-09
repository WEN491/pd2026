#include "rm_behavior_tree/plugins/condition/is_semantic.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>

namespace rm_behavior_tree
{
namespace
{
/// 去掉首尾空白：Groot2 里手填时很容易多带空格（如 " tunnel "）
std::string trim(const std::string & s)
{
  const auto not_space = [](unsigned char c) { return std::isspace(c) == 0; };
  auto begin = std::find_if(s.begin(), s.end(), not_space);
  auto end = std::find_if(s.rbegin(), s.rend(), not_space).base();
  return (begin < end) ? std::string(begin, end) : std::string();
}
}  // namespace

IsSemanticCondition::IsSemanticCondition(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<std_msgs::msg::String>(name, conf, params)
{
}

BT::NodeStatus IsSemanticCondition::onTick(
  const std::shared_ptr<std_msgs::msg::String> & last_msg)
{
  if (!started_) {
    first_tick_ = std::chrono::steady_clock::now();
    started_ = true;
  }

  // 有新消息就刷新缓存；latched 话题在订阅建立后很快就能拿到当前语义
  if (last_msg) {
    current_semantic_ = trim(last_msg->data);
    got_message_ = true;
    setOutput("received", current_semantic_);
  }

  // 还没收到过任何消息：判 FAILURE（不能拿空串去比，否则没起发布者时会误判成功）
  // latched 话题在订阅建立后通常几十毫秒内就会收到历史值，所以只在订阅满 2 秒仍无消息时告警一次
  if (!got_message_) {
    const auto waited = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - first_tick_).count();
    if (!warned_no_msg_ && waited > 2.0) {
      warned_no_msg_ = true;
      RCLCPP_WARN(
        logger(),
        "[%s] 订阅语义话题已 %.1f 秒仍未收到消息：检查发布者( tunnel_mode_manager )是否在跑、"
        "话题名/命名空间是否正确、QoS 是否需要 transient_local", name().c_str(), waited);
    }
    return BT::NodeStatus::FAILURE;
  }

  // 每个 tick 重新读端口：支持在 Groot2 里写死字面量，也支持 {blackboard_key} 动态值
  std::string expected;
  if (!getInput("semantic", expected)) {
    RCLCPP_ERROR(logger(), "[%s] 缺少输入端口 [semantic]", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  expected = trim(expected);

  const bool matched = (current_semantic_ == expected);
  RCLCPP_DEBUG(
    logger(), "[%s] 当前语义='%s' 期望='%s' → %s", name().c_str(),
    current_semantic_.c_str(), expected.c_str(), matched ? "SUCCESS" : "FAILURE");

  return matched ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

// 注册为 ROS 节点插件（需要 RosNodeParams 提供底层 rclcpp 节点，XML 里用 "IsSemantic"）
#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsSemanticCondition, "IsSemantic");
