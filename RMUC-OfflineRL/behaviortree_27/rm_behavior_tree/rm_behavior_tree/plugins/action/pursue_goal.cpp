#include "rm_behavior_tree/plugins/action/pursue_goal.hpp"

#include "rm_behavior_tree/visibility_checker.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace rm_behavior_tree
{

PursueGoalAction::PursueGoalAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config), node_(params.nh)
{
  // 端口在构造期读一次（XML 里的静态配置）。若某个端口写成 {blackboard}，请改成在 tick 里读。
  getInput("target_frame", target_frame_);
  getInput("map_topic", map_topic_);
  getInput("pursue_radius", pursue_radius_);
  getInput("update_interval", update_interval_);
  getInput("min_goal_change", min_goal_change_);
  getInput("wall_check_clearance", wall_check_clearance_);

  if (pursue_radius_ < 0.0) pursue_radius_ = 0.0;
  if (update_interval_ < 0.0) update_interval_ = 0.0;
  if (min_goal_change_ < 0.0) min_goal_change_ = 0.0;

  // 只有需要贴墙检查时才订阅地图；构造期订阅，避免漏掉 latched 的 /map
  if (wall_check_clearance_ > 0.0) {
    initMapSubscription();
  }
}

BT::NodeStatus PursueGoalAction::tick()
{
  // ---------- 1) 读黑板 ----------
  auto armors = getInput<auto_aim_interfaces::msg::Armors>("armors");
  if (!armors || armors->armors.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000, "PursueGoal: 没有敌人数据 → FAILURE");
    resetSession();
    return BT::NodeStatus::FAILURE;
  }
  auto loc = getInput<geometry_msgs::msg::TransformStamped>("current_location");
  if (!loc) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "PursueGoal: 缺少输入端口 [current_location] → FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::Point robot_pos;
  robot_pos.x = loc->transform.translation.x;
  robot_pos.y = loc->transform.translation.y;
  robot_pos.z = loc->transform.translation.z;

  // ---------- 2) 选最近敌人 + 滑窗滤波 ----------
  const auto enemy_pos = getEnemyPosition(armors.value(), robot_pos);
  if (std::isnan(enemy_pos.x)) {  // 全是无效装甲板（位置为 0,0,0）
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000, "PursueGoal: 装甲板位置无效 → FAILURE");
    resetSession();
    return BT::NodeStatus::FAILURE;
  }

  enemy_pos_history_.push_back(enemy_pos);
  if (enemy_pos_history_.size() > kEnemyHistorySize) {
    enemy_pos_history_.pop_front();
  }
  const auto filtered = getFilteredEnemyPos();

  setOutput("enemy_distance", std::hypot(enemy_pos.x - robot_pos.x, enemy_pos.y - robot_pos.y));

  // ---------- 3) 可选：贴墙检查 ----------
  if (wall_check_clearance_ > 0.0 && hasObstacleBetween(robot_pos, enemy_pos)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "PursueGoal: 机器人与敌人之间被障碍挡住 → FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  // ---------- 4) 节流：没到间隔就沿用上次结果（不改写字符串 → SendGoal 不重发）----------
  const auto now = node_->now();
  if (has_last_goal_ && has_last_update_ && update_interval_ > 0.0 &&
      (now - last_update_time_).seconds() < update_interval_)
  {
    return BT::NodeStatus::SUCCESS;
  }

  // ---------- 5) 算点，只在「真的移动了」才改写输出 ----------
  const auto pose = generatePursuePose(filtered, robot_pos, pursue_radius_);
  last_update_time_ = now;
  has_last_update_ = true;

  const double jump =
    std::hypot(pose.pose.position.x - last_goal_point_.x, pose.pose.position.y - last_goal_point_.y);
  if (has_last_goal_ && jump < min_goal_change_) {
    return BT::NodeStatus::SUCCESS;
  }

  const auto text = formatGoalPose(pose);
  setOutput("goal_pose", text);

  last_goal_pose_ = text;
  last_goal_point_.x = pose.pose.position.x;
  last_goal_point_.y = pose.pose.position.y;
  last_goal_point_.z = pose.pose.position.z;
  has_last_goal_ = true;

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "PursueGoal: 交战点 (%.3f, %.3f)　敌人 (%.3f, %.3f)　半径 %.2f m　跳变 %.2f m",
    pose.pose.position.x, pose.pose.position.y, filtered.x, filtered.y, pursue_radius_, jump);
  return BT::NodeStatus::SUCCESS;
}

geometry_msgs::msg::Point PursueGoalAction::getEnemyPosition(
  const auto_aim_interfaces::msg::Armors & armors, const geometry_msgs::msg::Point & robot_pos)
{
  geometry_msgs::msg::Point best{};
  bool found = false;
  double best_dist = std::numeric_limits<double>::max();

  for (const auto & armor : armors.armors) {
    const auto & p = armor.pose.position;
    // SubTargetPos 已把装甲板变到 map 系；位置全零视为无效装甲板
    if (std::abs(p.x) < 1e-6 && std::abs(p.y) < 1e-6 && std::abs(p.z) < 1e-6) {
      continue;
    }
    const double d = std::hypot(p.x - robot_pos.x, p.y - robot_pos.y);
    if (d < best_dist) {
      best_dist = d;
      best = p;
      found = true;
    }
  }

  if (!found) {
    geometry_msgs::msg::Point invalid;
    invalid.x = std::numeric_limits<double>::quiet_NaN();
    invalid.y = std::numeric_limits<double>::quiet_NaN();
    return invalid;
  }
  return best;
}

geometry_msgs::msg::Point PursueGoalAction::getFilteredEnemyPos() const
{
  if (enemy_pos_history_.empty()) {
    return geometry_msgs::msg::Point();
  }
  geometry_msgs::msg::Point sum{};
  for (const auto & p : enemy_pos_history_) {
    sum.x += p.x;
    sum.y += p.y;
    sum.z += p.z;
  }
  const double n = static_cast<double>(enemy_pos_history_.size());
  geometry_msgs::msg::Point mean;
  mean.x = sum.x / n;
  mean.y = sum.y / n;
  mean.z = sum.z / n;
  return mean;
}

geometry_msgs::msg::PoseStamped PursueGoalAction::generatePursuePose(
  const geometry_msgs::msg::Point & enemy_pos, const geometry_msgs::msg::Point & robot_pos,
  double radius)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = target_frame_;
  pose.header.stamp = node_->now();

  const double dx = robot_pos.x - enemy_pos.x;
  const double dy = robot_pos.y - enemy_pos.y;
  const double dist = std::hypot(dx, dy);

  if (dist < 1e-6) {
    // 与敌人重合：退到敌人 +x 方向，避免除零
    pose.pose.position.x = enemy_pos.x + radius;
    pose.pose.position.y = enemy_pos.y;
    pose.pose.position.z = robot_pos.z;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  // 从敌人朝自身方向退 radius 米 → 得到「保持交战距离」的点
  const double ratio = radius / dist;
  pose.pose.position.x = enemy_pos.x + dx * ratio;
  pose.pose.position.y = enemy_pos.y + dy * ratio;
  pose.pose.position.z = robot_pos.z;

  // 朝向敌人：yaw = atan2(dy, dx)，转成只绕 z 轴的四元数
  const double yaw = std::atan2(dy, dx);
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

std::string PursueGoalAction::formatGoalPose(const geometry_msgs::msg::PoseStamped & pose)
{
  std::ostringstream oss;
  // 与 MoveAround / SendGoal 完全一致的格式：固定 4 位小数、分号分隔 x;y;z;qx;qy;qz;qw
  oss << std::fixed << std::setprecision(4) << pose.pose.position.x << ";" << pose.pose.position.y
      << ";" << pose.pose.position.z << ";" << pose.pose.orientation.x << ";"
      << pose.pose.orientation.y << ";" << pose.pose.orientation.z << ";"
      << pose.pose.orientation.w;
  return oss.str();
}

void PursueGoalAction::initMapSubscription()
{
  if (map_sub_created_ || !node_) {
    return;
  }
  // /map 一般是 latched（只发一次）→ 必须 transient_local + reliable，
  // 否则订阅晚了永远收不到（老 PursueEnemy 用 best_effort 就踩了这个坑）
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, qos, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(map_mutex_);
      latest_map_ = msg;
    });
  map_sub_created_ = true;

  RCLCPP_INFO(
    node_->get_logger(), "PursueGoal: 订阅地图 '%s'（贴墙检查 clearance=%.2f m）",
    map_topic_.c_str(), wall_check_clearance_);
}

bool PursueGoalAction::hasObstacleBetween(
  const geometry_msgs::msg::Point & robot_pos, const geometry_msgs::msg::Point & enemy_pos)
{
  if (wall_check_clearance_ <= 0.0) {
    return false;
  }

  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!latest_map_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "PursueGoal: 还没收到地图 '%s'，跳过贴墙检查（放行）", map_topic_.c_str());
    return false;
  }

  MapAccessor map(*latest_map_);
  double resolution = map.getResolution();
  if (resolution <= 0.0) {
    resolution = 0.05;
  }
  const int clearance_cells = static_cast<int>(wall_check_clearance_ / resolution);

  KeyPoint from;
  from.x = robot_pos.x;
  from.y = robot_pos.y;
  KeyPoint to;
  to.x = enemy_pos.x;
  to.y = enemy_pos.y;

  return !VisibilityChecker::isVisible(from, to, map, clearance_cells);
}

void PursueGoalAction::resetSession()
{
  enemy_pos_history_.clear();
  last_goal_pose_.clear();
  last_goal_point_ = geometry_msgs::msg::Point();
  has_last_goal_ = false;
  has_last_update_ = false;
}

}  // namespace rm_behavior_tree

// 注册为 ROS 节点插件（需要 RosNodeParams 提供底层 rclcpp 节点；XML 里用 "PursueGoal"）
#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::PursueGoalAction, "PursueGoal");
