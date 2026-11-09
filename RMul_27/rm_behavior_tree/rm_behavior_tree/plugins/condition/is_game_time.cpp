#include "rm_behavior_tree/plugins/condition/is_game_time.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 构造函数：初始化条件节点并订阅比赛状态话题
 *
 * @param name   行为树节点实例名
 * @param config 行为树节点配置
 * @param params ROS 节点参数（提供底层 rclcpp 节点）
 */
IsGameTimeCondition::IsGameTimeCondition(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
    : BT::ConditionNode(name, config),
      node_(params.nh)
{
    // 订阅 /game_status，缓存最新一条消息供 tick 判断使用
    sub_ = node_->create_subscription<robot_msgs::msg::GameStatus>(
        "/game_status", 10,
        [this](const robot_msgs::msg::GameStatus::SharedPtr msg)
        {
            last_msg_ = *msg;
            has_msg_ = true;
        });
}

/**
 * @brief 节点主逻辑：判断比赛阶段与剩余时间是否落在期望区间内
 *
 * @return SUCCESS 阶段匹配且剩余时间在 [lower, higher] 区间内
 *         FAILURE 阶段或时间不匹配
 *         RUNNING  尚未获得任何比赛状态消息（等待中）
 */
BT::NodeStatus IsGameTimeCondition::tick()
{
    // 读取期望的比赛阶段与剩余时间上下限（来自行为树端口）
    int game_progress, lower_remain_time, higher_remain_time;
    getInput("game_progress", game_progress);
    getInput("lower_remain_time", lower_remain_time);
    getInput("higher_remain_time", higher_remain_time);

    // XML message 优先：优先使用端口传入的 message，
    // 否则回退到内部订阅缓存的消息
    auto msg = getInput<robot_msgs::msg::GameStatus>("message");
    if (!msg && !has_msg_)
    {
        // 两种来源都没有消息（常见于刚启动阶段）
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "IsGameTime: waiting for /game_status ...");
        return BT::NodeStatus::RUNNING;  // 启动阶段返回 RUNNING，避免误判
    }

    // 选用可用的消息源（端口消息优先，其次为订阅缓存）
    const auto & game_msg = msg ? *msg : last_msg_;

    // 阶段完全匹配，且剩余时间处于 [lower_remain_time, higher_remain_time] 闭区间
    if (game_msg.game_progress == game_progress &&
        game_msg.stage_remain_time >= lower_remain_time &&
        game_msg.stage_remain_time <= higher_remain_time)
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

// 将该 Condition 节点注册为行为树插件，XML 中通过 "IsGameTime" 引用
#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsGameTimeCondition, "IsGameTime");
