#include "rm_behavior_tree/plugins/action/pub_autoaim_target.hpp"
#include <sstream>
#include <algorithm>

namespace rm_behavior_tree
{

PubAutoaimTargetAction::PubAutoaimTargetAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<sp_msgs::msg::AutoaimTargetMsg>(name, conf, params)
{
}

// 从字符串解析ID列表，支持格式："1,3,5" 或 "[1,3,5]" 或 "1 3 5"
std::vector<int8_t> parseIdList(const std::string & str)
{
  std::vector<int8_t> ids;
  if (str.empty()) {
    return ids;
  }
  
  std::string cleaned = str;
  // 移除方括号和空格
  cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '['), cleaned.end());
  cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ']'), cleaned.end());
  cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ' '), cleaned.end());
  
  if (cleaned.empty()) {
    return ids;
  }
  
  std::istringstream iss(cleaned);
  std::string token;
  
  // 按逗号或空格分割
  while (std::getline(iss, token, ',')) {
    if (!token.empty()) {
      try {
        int id = std::stoi(token);
        ids.push_back(static_cast<int8_t>(id));
      } catch (const std::exception &) {
        // 忽略无效的token
      }
    }
  }
  
  return ids;
}

bool PubAutoaimTargetAction::setMessage(sp_msgs::msg::AutoaimTargetMsg & msg)
{
  // 先按字符串取：端口声明就是 std::string，且 BT.CPP 没有 string→vector<int8_t>
  // 的转换器；若先 getInput<std::vector<int8_t>>，每个 tick 都会打印
  // "convertFromString() ... can't find the template specialization" 并抛错一次。
  auto ids_str = getInput<std::string>("target_ids");
  if (ids_str) {
    msg.target_ids = parseIdList(ids_str.value());
  } else {
    // 端口没给字符串时（例如从黑板传进来的是向量），再尝试向量
    auto target_ids_in = getInput<std::vector<int8_t>>("target_ids");
    if (target_ids_in) {
      msg.target_ids = target_ids_in.value();
    } else {
      // 都没有提供，使用空列表（无目标）
      msg.target_ids.clear();
    }
  }

  // 设置时间戳
  msg.timestamp = node_->now();

  return true;
}

BT::PortsList PubAutoaimTargetAction::providedPorts()
{
  BT::PortsList ports = {
    BT::InputPort<std::string>(
      "target_ids",
      "",
      "List of target IDs as string (e.g., '1,3,5' or '[1,3,5]') or vector")
  };

  return providedBasicPorts(ports);
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::PubAutoaimTargetAction, "PubAutoaimTarget");

