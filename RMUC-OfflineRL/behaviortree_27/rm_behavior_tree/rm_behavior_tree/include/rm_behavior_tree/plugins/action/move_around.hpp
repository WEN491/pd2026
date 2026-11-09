#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_AROUND_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_AROUND_HPP_

#include <cmath>     // M_PI（角度用）
#include <mutex>     // std::mutex（保护地图数据）
#include <random>    // std::mt19937、std::uniform_real_distribution（随机数）
#include <string>

#include "behaviortree_cpp/action_node.h"            // BT::SyncActionNode
#include "behaviortree_ros2/ros_node_params.hpp"     // BT::RosNodeParams
#include "geometry_msgs/msg/transform_stamped.hpp"   // 输入：机器人当前位置
#include "nav_msgs/msg/occupancy_grid.hpp"           // 订阅 /map 用
#include "rm_behavior_tree/map_accessor.hpp"         // 私有函数签名里用到 MapAccessor
#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{

/**
 * @brief 随机游走【规划器】
 *
 * 只干三件事：
 *   1) 以机器人当前位置为圆心、expected_dis 为半径，随机撒一个点
 *   2) 校验这个点能不能用（在图内 / 不是障碍 / 不是未知区域 / 直线可达）
 *   3) 把选中的点写成 "x;y;z;qx;qy;qz;qw" 输出到 goal_pose 端口
 *
 * 它【不】负责导航 —— 导航由 XML 里的 SendGoal 负责。
 * 因此本节点不需要 action client、不需要回调注册、不需要考虑 spin。
 *
 * 典型用法：
 *   <Repeat num_cycles="5">
 *     <Fallback>
 *       <RetryUntilSuccessful num_attempts="3">
 *         <Sequence>
 *           <MoveAround goal_pose="{goal_pose}" message="{current_location}" expected_dis="0.3"/>
 *           <SendGoal   goal_pose="{goal_pose}" action_name="navigate_to_pose"/>
 *         </Sequence>
 *       </RetryUntilSuccessful>
 *       <AlwaysSuccess/>
 *     </Fallback>
 *   </Repeat>
 */
class MoveAroundAction : public BT::SyncActionNode
{
public:
  MoveAroundAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::TransformStamped>("message"),
      BT::InputPort<double>("expected_dis", 0.3, "撒点半径 / m"),
      BT::InputPort<std::string>("map_topic", "/map", "用于落点校验的栅格地图话题"),
      BT::InputPort<double>("goal_clearance", 0.0, "直线可达检查的安全间隔 / m，0 = 只检查栅格本身"),
      BT::InputPort<int>("max_resample", 20, "单次 tick 内最多重抽几次"),
      BT::InputPort<bool>("allow_unknown", false, "是否允许落在未知区域（栅格值 -1）"),
      BT::OutputPort<std::string>(
        "goal_pose", "输出目标，格式 x;y;z;qx;qy;qz;qw，交给 SendGoal 执行")};
  }

  BT::NodeStatus tick() override;

private:
  // 抽点：成功返回 true，并把坐标写进 out_x / out_y
  bool sampleUsablePoint(const MapAccessor & map, double cx, double cy,
                         double & out_x, double & out_y);
  // 判点：从 (cx,cy) 到 (x,y) 这个落点能不能用
  bool isPointUsable(const MapAccessor & map, double cx, double cy, double x, double y);
  // 订阅地图（在构造函数里调用一次，不能等到 tick 才订阅）
  void initMapSubscription();
  // 把坐标拼成 SendGoal 能解析的字符串
  static std::string formatGoalPose(double x, double y, double z);

  rclcpp::Node::SharedPtr node_;   // 外部传进来的 ROS 节点（已被 executor spin）

  std::string map_topic_;
  double expected_dis_;
  double goal_clearance_;
  int max_resample_;
  bool allow_unknown_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  std::mutex map_mutex_;

  std::mt19937 rng_;                                        // 随机数引擎（成员，只建一次）
  std::uniform_real_distribution<double> angle_dist_{0.0, 2.0 * M_PI};  // 角度分布 [0, 2π)
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_AROUND_HPP_
