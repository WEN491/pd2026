#include "rm_behavior_tree/plugins/action/sub_target_pos.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <sstream>
#include <algorithm>
#include <cmath>

namespace rm_behavior_tree
{

/**
 * @brief 构造函数：初始化同步行为树节点、TF 缓冲与状态变量
 *
 * @param name   行为树节点实例名
 * @param config 行为树节点配置
 * @param params ROS 节点参数（提供底层 rclcpp 节点）
 */
SubTargetPosAction::SubTargetPosAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config), node_(params.nh)
{
  // 默认目标坐标系 map，源坐标系 chassis（可在 tick 中通过端口覆盖）
  target_frame_ = "map";
  source_frame_ = "chassis";

  // 创建 TF 缓存与监听器（用于把视觉数据从源坐标系变换到目标坐标系）
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 初始化输出消息与时间戳记录
  last_armors_.armors.clear();
  last_armors_.header.frame_id = target_frame_;
  last_armors_.header.stamp = node_->now();
  last_msg_time_ = node_->now();

  // 订阅尚未创建，延迟到首次 tick 时创建
  subscription_created_ = false;
}

/**
 * @brief 解析目标位置字符串，格式为 "x,y,z,w"（至少 4 个逗号分隔的浮点数）
 *
 * @param data 待解析的字符串
 * @param x,y,z,w 解析结果（输出参数）
 * @return true 解析成功；false 解析失败或数值不足
 */
bool SubTargetPosAction::parseTargetPos(
  const std::string & data, double & x, double & y, double & z, double & w)
{
  std::istringstream iss(data);
  std::string token;
  std::vector<double> values;

  // 按逗号切分字符串并逐个转换为 double
  while (std::getline(iss, token, ',')) {
    try {
      values.push_back(std::stod(token));
    } catch (const std::exception &) {
      // 存在无法转换的字段，视为解析失败
      return false;
    }
  }

  // 至少需要 x, y, z, w 四个值
  if (values.size() < 4) {
    return false;
  }

  x = values[0];
  y = values[1];
  z = values[2];
  w = values[3];
  return true;
}

/**
 * @brief 订阅话题回调：解析目标位置并缓存到 raw_armor_
 *
 * @param msg 收到的字符串消息
 */
void SubTargetPosAction::targetPosCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_DEBUG(node_->get_logger(), "SubTargetPos CB: Received raw msg: '%s'", msg->data.c_str());

  double x, y, z, w;

  // 解析失败则丢弃本条消息
  if (!parseTargetPos(msg->data, x, y, z, w)) {
    RCLCPP_WARN(node_->get_logger(), "SubTargetPos CB: Failed to parse target pos");
    return;
  }

  // x、y 均接近 0，表示敌人消失（约定用 (0,0) 表示无目标）
  if (std::abs(x) < 1e-6 && std::abs(y) < 1e-6) {
    // 显式清空 raw_armor_ 表示敌人消失
    raw_armor_.pose.position.x = 0.0;
    raw_armor_.pose.position.y = 0.0;
    raw_armor_.pose.position.z = 0.0;
    last_armors_.armors.clear();
    last_armors_.header.frame_id = target_frame_;
    last_armors_.header.stamp = node_->now();
    last_msg_time_ = node_->now();
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "SubTargetPos CB: Detected zero armor, clearing data");
    return;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "SubTargetPos CB: raw=(%.3f, %.3f, %.3f)", x, y, z);

  // 保存原始位置（源坐标系下），朝向暂用单位四元数占位
  raw_armor_.pose.position.x = x;
  raw_armor_.pose.position.y = y;
  raw_armor_.pose.position.z = z;
  raw_armor_.pose.orientation.x = 0.0;
  raw_armor_.pose.orientation.y = 0.0;
  raw_armor_.pose.orientation.z = 0.0;
  raw_armor_.pose.orientation.w = 1.0;

  // 记录最新消息时间，供超时检测使用
  last_msg_time_ = node_->now();
}

/**
 * @brief 将装甲板位姿从源坐标系变换到目标坐标系
 *
 * @param in_pose  输入的源坐标系位姿
 * @param out_pose 输出的目标坐标系位姿（失败时被置为单位位姿）
 * @param stamp    消息时间戳（当前实现使用最新 TF，未直接使用该值）
 * @return true 变换成功；false TF 不可用或查询失败
 */
bool SubTargetPosAction::transformArmorToTargetFrame(
  const geometry_msgs::msg::Pose & in_pose,
  geometry_msgs::msg::Pose & out_pose,
  const rclcpp::Time & stamp)
{
  // TF 缓冲未初始化，直接返回空位姿
  if (!tf_buffer_) {
    out_pose = geometry_msgs::msg::Pose();
    return false;
  }

  const std::string & actual_src = source_frame_;
  const std::string & actual_tgt = target_frame_;

  // 源与目标坐标系相同时无需变换，直接透传
  if (actual_src == actual_tgt) {
    out_pose = in_pose;
    return true;
  }

  try {
    // 用当前时间查 TF（消息时间戳可能是旧时间，导致 extrapolation 失败）
    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
      actual_tgt, actual_src, rclcpp::Time(0), tf2::durationFromSec(0.5));

    geometry_msgs::msg::PoseStamped pose_in, pose_out;
    pose_in.header.stamp = rclcpp::Time(0);  // 用当前时间保持一致
    pose_in.header.frame_id = actual_src;
    pose_in.pose = in_pose;

    // 执行坐标系变换
    tf2::doTransform(pose_in, pose_out, tf_stamped);
    out_pose = pose_out.pose;

    return true;
  } catch (const tf2::TransformException & e) {
    // 变换失败：告警并返回空位姿
    RCLCPP_WARN(node_->get_logger(),
                "SubTargetPos: [TF FAIL] %s->%s failed: %s  → returning empty",
                actual_src.c_str(), actual_tgt.c_str(), e.what());
    out_pose = geometry_msgs::msg::Pose();
    return false;
  }
}

/**
 * @brief 节点主逻辑：创建订阅、检测超时/丢失、变换坐标并输出 armors
 *
 * @return SUCCESS 成功输出有效装甲板；FAILURE 无有效目标或订阅未就绪
 */
BT::NodeStatus SubTargetPosAction::tick()
{
  // 每 100 次 tick 打印一次调试信息，避免日志过多
  static int tick_count = 0;
  if (tick_count++ % 100 == 0) {
    RCLCPP_INFO(node_->get_logger(), "SubTargetPos tick #%d: raw=(%.3f,%.3f,%.3f) sub_ready=%d",
        tick_count,
        raw_armor_.pose.position.x, raw_armor_.pose.position.y, raw_armor_.pose.position.z,
        sub_ != nullptr);
  }

  // 从端口读取话题名（默认 auto_aim_target_pos）
  std::string topic_name = "auto_aim_target_pos";
  if (auto topic_res = getInput<std::string>("topic_name")) {
    topic_name = topic_res.value();
  }

  // 从端口读取目标坐标系（默认 map）
  if (auto target_res = getInput<std::string>("target_frame")) {
    target_frame_ = target_res.value();
  }

  // 从端口读取源坐标系（默认 chassis）
  if (auto source_res = getInput<std::string>("source_frame")) {
    source_frame_ = source_res.value();
  }

  // 首次 tick 时创建订阅（使用可靠 QoS）
  if (!sub_) {
    rclcpp::QoS qos(10);
    qos.reliable();

    RCLCPP_INFO(node_->get_logger(), "SubTargetPos: Creating subscription to '%s'", topic_name.c_str());
    sub_ = node_->create_subscription<std_msgs::msg::String>(
      topic_name, qos,
      std::bind(&SubTargetPosAction::targetPosCallback, this, std::placeholders::_1));

    subscription_created_ = true;
    RCLCPP_INFO(node_->get_logger(), "SubTargetPos: Subscription created successfully");
  }

  // 超过 10 秒没收到消息，认为数据过期并清空
  if (subscription_created_ && !last_armors_.armors.empty()) {
    auto dt = (node_->now() - last_msg_time_).seconds();
    if (dt > 10.0) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "SubTargetPos: No message for %.1f seconds, clearing armor data", dt);
      last_armors_.armors.clear();
      last_valid_armor_ = auto_aim_interfaces::msg::Armor();
    }
  }

  // 每帧重建输出消息头与列表
  last_armors_.armors.clear();
  last_armors_.header.frame_id = target_frame_;
  last_armors_.header.stamp = node_->now();

  // 记录本帧起始时间，用于统计耗时（ms_now 返回相对 t0 的毫秒数）
  auto t0 = std::chrono::steady_clock::now();
  auto ms_now = [this, &t0]() {
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
  };

  // raw_armor_ 全为 0 表示当前无目标
  if (!raw_armor_.pose.position.x && !raw_armor_.pose.position.y && !raw_armor_.pose.position.z) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "SubTargetPos: raw_armor is (0,0,0), enemy lost");
    // 敌人消失时清空有效数据，输出空数组让行为树能检测到敌人丢失
    last_valid_armor_ = auto_aim_interfaces::msg::Armor();
    last_armors_.armors.clear();
    setOutput("armors", last_armors_);
    return subscription_created_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::FAILURE;
  }

  // 将原始数据变换到目标坐标系
  geometry_msgs::msg::Pose transformed_pose;
  bool transform_ok = transformArmorToTargetFrame(
    raw_armor_.pose, transformed_pose, last_msg_time_);

  if (!transform_ok) {
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "SubTargetPos: [%05.1fms] TF FAIL %s->%s raw=(%.3f,%.3f,%.3f), keeping last valid",
        ms_now(), source_frame_.c_str(), target_frame_.c_str(),
        raw_armor_.pose.position.x, raw_armor_.pose.position.y, raw_armor_.pose.position.z);
    // TF失败时保留上一帧有效数据（不输出0）
    if (last_valid_armor_.pose.position.x || last_valid_armor_.pose.position.y || last_valid_armor_.pose.position.z) {
      transformed_pose.position = last_valid_armor_.pose.position;
      transformed_pose.orientation = last_valid_armor_.pose.orientation;
      transform_ok = true;
    }
  } else {
    // 成功时更新last_valid_armor_
    last_valid_armor_.pose.position = transformed_pose.position;
    last_valid_armor_.pose.orientation = transformed_pose.orientation;
    last_valid_armor_.number = "0";
    last_valid_armor_.type = "unknown";
    last_valid_armor_.distance_to_image_center = 0.0f;
  }

  last_armors_.header.frame_id = target_frame_;

  // 组装输出用的装甲板消息（仅含位置与朝向）
  auto_aim_interfaces::msg::Armor armor;
  armor.pose.position = transformed_pose.position;
  armor.pose.orientation = transformed_pose.orientation;
  armor.number = "0";
  armor.type = "unknown";
  armor.distance_to_image_center = 0.0f;

  last_armors_.armors.push_back(armor);

  // 写入输出端口并记录本帧耗时
  setOutput("armors", last_armors_);
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "SubTargetPos: [%05.1fms] done -> (%.3f,%.3f,%.3f)",
      ms_now(),
      transformed_pose.position.x, transformed_pose.position.y, transformed_pose.position.z);
  return subscription_created_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

// 将节点注册为行为树插件，XML 中通过 "SubTargetPos" 引用
#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubTargetPosAction, "SubTargetPos");
