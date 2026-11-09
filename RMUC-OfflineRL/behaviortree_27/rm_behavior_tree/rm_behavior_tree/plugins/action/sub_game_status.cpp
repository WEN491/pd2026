#include "rm_behavior_tree/plugins/action/sub_game_status.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 构造函数：继承自 BT::RosTopicSubNode，订阅逻辑由基类完成
 *
 * 基类会根据端口 topic_name（默认取自节点参数中的默认话题）创建订阅，
 * 每次 tick 时将最近收到的消息传入 onTick 回调。
 *
 * @param name   行为树节点实例名
 * @param conf   行为树节点配置（端口绑定等信息）
 * @param params ROS 节点参数（提供底层 rclcpp 节点）
 */
SubGameStatusAction::SubGameStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<robot_msgs::msg::GameStatus>(name, conf, params)
{
}

/**
 * @brief 节点主逻辑：收到新消息时将其写入 game_status 输出端口
 *
 * @param last_msg 自上次 tick 以来收到的最新消息；
 *                 为 nullptr 表示这段时间内没有新消息
 * @return 始终返回 SUCCESS（无新消息不算失败）
 */
BT::NodeStatus SubGameStatusAction::onTick(
  const std::shared_ptr<robot_msgs::msg::GameStatus> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    // 仅在有新消息时输出调试日志并转发到输出端口，
    // 供下游节点（如 IsGameTime）读取使用
    RCLCPP_DEBUG(
      logger(), "[%s] new message, game_progress: %s", name().c_str(),
      std::to_string(last_msg->game_progress).c_str());
    setOutput("game_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

// 将该节点注册为行为树插件，XML 中通过 "SubGameStatus" 引用
#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubGameStatusAction, "SubGameStatus");
