#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_SEMANTIC_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_SEMANTIC_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace rm_behavior_tree
{

/**
 * @brief BT Condition 节点：把「当前语义」与「Groot2 里设定的语义」对比。
 *
 * 语义来源：tunnel_mode_manager 发布的 std_msgs/String 话题
 *   /terrain_zone              （根命名空间）
 *   /<ns>/terrain_zone         （导航跑在命名空间下时，如 /red_standard_robot1/terrain_zone）
 *   该话题 QoS = RELIABLE + TRANSIENT_LOCAL（latched）。本节点继承
 *   BT::RosTopicSubNode<T>，而本仓库这个基类在创建订阅时就是
 *   transient_local + reliable，所以第一个 tick 就能拿到当前值，不会漏历史。
 *
 * 行为：
 *  - 收到的语义 == semantic 端口的值（两者都去掉首尾空白后比较） → SUCCESS
 *  - 不相等                                                      → FAILURE
 *  - 还没收到任何消息                                            → FAILURE
 *
 * 端口：
 *  - topic_name : 语义话题名，默认 terrain_zone（相对名 → 跟随节点命名空间解析，
 *                 所以行为树用 -r __ns:=/<ns> 启动时会自动对齐到导航命名空间）
 *  - semantic   : 期望的语义，在 Groot2 里填，如 tunnel / high_ground / slope ...
 *                 留空串表示「当前不在任何 zone」（tunnel_mode_manager 此时发 ""）
 *  - received   : 输出，当前收到的语义（调试 / 串联到黑板用）
 */
class IsSemanticCondition : public BT::RosTopicSubNode<std_msgs::msg::String>
{
public:
  IsSemanticCondition(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {// 期望语义；留空 = 断言「不在任何 zone」
       BT::InputPort<std::string>("semantic", "", "期望的语义（在 Groot2 中填写）"),
       BT::OutputPort<std::string>("received", "当前收到的语义（调试用）")});
  }

  BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::String> & last_msg) override;

private:
  /// 最近一次收到的语义。latched 话题首 tick 即可拿到；之后只在变化时更新
  std::string current_semantic_;
  /// 是否至少收到过一次消息（没收到时判 FAILURE，而不是拿空串去比）
  bool got_message_ = false;
  /// 缺消息的告警只打一次，避免 100Hz 刷屏
  bool warned_no_msg_ = false;
  /// 首个 tick 的时间戳，用于"订阅多久还没消息"的判定
  std::chrono::steady_clock::time_point first_tick_{};
  bool started_ = false;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_SEMANTIC_HPP_
