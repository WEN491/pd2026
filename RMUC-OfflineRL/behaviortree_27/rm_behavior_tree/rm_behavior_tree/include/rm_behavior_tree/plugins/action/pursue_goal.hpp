#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PURSUE_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PURSUE_GOAL_HPP_

#include <deque>
#include <mutex>
#include <string>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rm_behavior_tree/map_accessor.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{

/**
 * @brief 追击敌人【规划器】—— MoveAround 模式
 *
 * 与老的 PursueEnemy 的区别（老节点同时管「算点 + 发导航 + 取消导航」）：
 *   本节点【只算点】：读黑板 → 算一个交战点 → 写成 "x;y;z;qx;qy;qz;qw" 输出到 goal_pose，
 *   导航/取消/重发全部交给 SendGoal（RosActionNode）。因此：
 *     - 不需要 action client、不需要自己 spin、不持有 goal_handle；
 *     - 不会出现「自建节点没进 executor → 回调不执行、取消无效」那类问题；
 *     - 与 MoveAround + SendGoal 的行为完全统一，便于调试。
 *
 * 算法（沿用老节点已验证的部分）：
 *   1) 从 armors 里挑距自身最近的装甲板作为敌人位置；
 *   2) 滑动窗口均值滤波（kEnemyHistorySize 帧）压掉装甲板随云台旋转的抖动；
 *   3) 在「敌人 → 自身」连线上取距敌人 pursue_radius 米的点（z 用自身高度，朝向敌人）；
 *   4) 可选贴墙检查：机器人与敌人连线上被地图挡住 → FAILURE。
 *
 * 防抖（与 SendGoal 配合的关键）：
 *   SendGoal 发现 goal_pose 端口字符串变化就会「取消旧目标 + 按新点重发」。
 *   所以本节点只在**点真的移动超过 min_goal_change** 时才改写输出，
 *   并用 update_interval 限速；目标没变时保持字符串不变 → SendGoal 不会抖动。
 *
 * 返回：
 *   SUCCESS —— 已经给出一个可用的追击点（可能是本次新算的，也可能沿用上次的）
 *   FAILURE —— 没有敌人 / 缺自身位姿 / 贴墙检查发现被挡
 *
 * 典型用法（注意必须用 ReactiveSequence：每 tick 重算，才能跟得上敌人移动）：
 *   <ReactiveSequence>
 *     <PursueGoal armors="{armors}" current_location="{current_location}"
 *                 pursue_radius="2.5" goal_pose="{pursue_goal}"/>
 *     <SendGoal goal_pose="{pursue_goal}" action_name="navigate_to_pose"/>
 *   </ReactiveSequence>
 */
class PursueGoalAction : public BT::SyncActionNode
{
public:
  PursueGoalAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<auto_aim_interfaces::msg::Armors>("armors", "敌人装甲板（来自 SubTargetPos）"),
      BT::InputPort<geometry_msgs::msg::TransformStamped>(
        "current_location", "自身位姿（来自 GetCurrentLocation）"),
      BT::InputPort<double>("pursue_radius", 2.0, "保持的交战距离 / m"),
      BT::InputPort<double>("update_interval", 0.5, "重算间隔 / s，0 = 每 tick 都算"),
      BT::InputPort<double>(
        "min_goal_change", 0.3, "追击点移动超过该距离才改写 goal_pose / m（防 SendGoal 抖动）"),
      BT::InputPort<std::string>("target_frame", "map", "输出目标坐标系"),
      BT::InputPort<std::string>("map_topic", "/map", "贴墙检查用的栅格地图话题"),
      BT::InputPort<double>("wall_check_clearance", 0.0, "贴墙检查安全间隔 / m，0 = 关闭"),
      BT::OutputPort<std::string>(
        "goal_pose", "输出目标 \"x;y;z;qx;qy;qz;qw\"，交给 SendGoal 执行"),
      BT::OutputPort<double>("enemy_distance", "自身到敌人的距离 / m（调试用）")};
  }

  BT::NodeStatus tick() override;

private:
  /// 挑距自身最近的装甲板，返回其位置（map 系）
  geometry_msgs::msg::Point getEnemyPosition(const auto_aim_interfaces::msg::Armors & armors,
                                             const geometry_msgs::msg::Point & robot_pos);
  /// 滑窗均值滤波后的敌人位置
  geometry_msgs::msg::Point getFilteredEnemyPos() const;
  /// 在「敌人→自身」连线上取距敌人 radius 米的点，朝向敌人
  geometry_msgs::msg::PoseStamped generatePursuePose(
    const geometry_msgs::msg::Point & enemy_pos, const geometry_msgs::msg::Point & robot_pos,
    double radius);
  /// 与 MoveAround / SendGoal 一致的字符串格式（固定 4 位小数、单位四元数按 yaw 旋转）
  static std::string formatGoalPose(const geometry_msgs::msg::PoseStamped & pose);
  /// 订阅地图（构造期一次；/map 多为 latched，必须 transient_local）
  void initMapSubscription();
  /// 机器人与敌人之间是否被墙挡住
  bool hasObstacleBetween(const geometry_msgs::msg::Point & robot_pos,
                          const geometry_msgs::msg::Point & enemy_pos);
  /// 清掉一次追击会话的中间状态（敌人丢失时用）
  void resetSession();

  rclcpp::Node::SharedPtr node_;

  // 本次追击会话的状态（SyncActionNode 实例整棵树生命周期只构造一次，成员会跨 tick 保留）
  static constexpr size_t kEnemyHistorySize = 3;
  std::deque<geometry_msgs::msg::Point> enemy_pos_history_;
  std::string last_goal_pose_;                  ///< 已输出到黑板的字符串，用于「没变就不改写」
  geometry_msgs::msg::Point last_goal_point_{};  ///< 上次输出的点，用于比较跳变距离
  bool has_last_goal_ = false;
  rclcpp::Time last_update_time_;      ///< 上次真正重算的时刻
  bool has_last_update_ = false;

  // 地图订阅（贴墙检查用）
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  std::mutex map_mutex_;
  bool map_sub_created_ = false;

  // 端口缓存（构造期读一次；运行期若需要动态值请在 tick 里重新 getInput）
  std::string target_frame_{"map"};
  std::string map_topic_{"/map"};
  double pursue_radius_{2.0};
  double update_interval_{0.5};
  double min_goal_change_{0.3};
  double wall_check_clearance_{0.0};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PURSUE_GOAL_HPP_
