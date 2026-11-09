#include "rm_behavior_tree/plugins/action/get_current_location.hpp"

#include <rclcpp/logging.hpp>

namespace rm_behavior_tree
{
// 全局递增计数器，用于给每次 tick 分配序号，
// 便于在日志中区分和追踪不同时刻的调用记录。
static std::atomic<int> g_getloc_tick(0);

/**
 * @brief 构造函数：初始化行为树同步节点与 TF 监听器
 *
 * @param name   行为树中该节点的实例名
 * @param config 行为树节点配置（端口绑定等信息）
 * @param params ROS 节点参数，提供底层 rclcpp 节点和日志器
 */
GetCurrentLocationAction::GetCurrentLocationAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config), node_(params.nh), logger_(params.nh->get_logger())
{
  // 基于节点时钟创建 TF 缓存缓冲区
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  // 创建 TF 监听器，持续接收并缓存 TF 变换数据
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // 监听器创建失败时直接抛出异常，避免后续空指针使用
  if (!tf_listener_) {
    throw std::runtime_error("Failed to create tf2_ros::TransformListener");
  }
}

/**
 * @brief 节点主逻辑：查询目标坐标系到源坐标系的 TF 变换并输出
 *
 * @return SUCCESS 查询到变换并成功写入 current_location 端口
 *         FAILURE TF 查询失败
 */
BT::NodeStatus GetCurrentLocationAction::tick()
{
  // 本次调用的唯一序号，用于日志追踪
  int my_tick = ++g_getloc_tick;

  // 默认目标坐标系为 map，源坐标系为 gimbal_yaw
  std::string target_frame = "map";
  std::string source_frame = "gimbal_yaw";

  // 若行为树中显式配置了 target_frame 端口，则覆盖默认值
  if (auto target_res = getInput<std::string>("target_frame")) {
    target_frame = target_res.value();
  }
  // 若行为树中显式配置了 source_frame 端口，则覆盖默认值
  if (auto source_res = getInput<std::string>("source_frame")) {
    source_frame = source_res.value();
  }

  try {
    // 用 rclcpp::Time(0) 查最新可用 TF，有一定容差
    // 查询 source_frame -> target_frame 的变换，超时 100ms
    geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
      target_frame,
      source_frame,
      rclcpp::Time(0),
      std::chrono::milliseconds(100));

    // 将查询到的变换写入 current_location 输出端口
    setOutput("current_location", t);
    return BT::NodeStatus::SUCCESS;
  } catch (const tf2::TransformException & e) {
    // 查询失败时以 3 秒节流的方式打印警告，避免刷屏
    RCLCPP_WARN_THROTTLE(
      logger_,
      *node_->get_clock(),
      3000,
      "GetCurrentLocation tick#%d: FAILURE - TF %s->%s failed: %s",
      my_tick,
      source_frame.c_str(),
      target_frame.c_str(),
      e.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

// 将该 Action 节点注册为行为树插件，XML 中通过 "GetCurrentLocation" 引用
#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::GetCurrentLocationAction, "GetCurrentLocation");
