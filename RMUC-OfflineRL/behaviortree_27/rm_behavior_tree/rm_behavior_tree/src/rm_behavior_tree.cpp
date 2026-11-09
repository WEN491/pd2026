#include "rm_behavior_tree/rm_behavior_tree.h"
#include "rm_behavior_tree/bt_conversions.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <thread>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;

  std::string bt_xml_path;
  auto node = std::make_shared<rclcpp::Node>("rm_behavior_tree");
  node->declare_parameter<std::string>("bt_xml", "");
  node->get_parameter_or<std::string>("bt_xml", bt_xml_path, "");

  std::cout << "Start RM_Behavior_Tree" << '\n';
  RCLCPP_INFO(node->get_logger(), "Load bt_xml: \e[1;42m %s \e[0m", bt_xml_path.c_str());

  BT::RosNodeParams params_update_msg;
  params_update_msg.nh = std::make_shared<rclcpp::Node>("update_msg");

  BT::RosNodeParams params_robot_control;
  params_robot_control.nh = std::make_shared<rclcpp::Node>("robot_control");
  params_robot_control.default_port_value = "robot_control";

  BT::RosNodeParams params_robot_state;
  params_robot_state.nh = std::make_shared<rclcpp::Node>("robot_state");
  params_robot_state.default_port_value = "robot_state";

  BT::RosNodeParams params_send_goal;
  params_send_goal.nh = std::make_shared<rclcpp::Node>("send_goal");
  params_send_goal.default_port_value = "goal_pose";

  // clang-format off
  const std::vector<std::string> msg_update_plugin_libs = {
    "sub_all_robot_hp",
    "sub_robot_status",
    "sub_game_status",
    // "sub_armors",
    "sub_decision_num",
    "sub_target_pos",
    "sub_gimbal_target",
    "sub_remote_status",
    "sub_auto_aim_status",
    "sub_chassis_energy",
    "sub_rune_status",
  };

  const std::vector<std::string> bt_plugin_libs = {
    "rate_controller",
    "decision_switch",
    "is_friend_ok",
    "is_detect_enemy",
    "is_rfid_detected",
    "is_target_in_zone",
    "is_gimbal_target_in_zone",
    // "move_around" 已改成"需要 RosNodeParams 的插件"（CreateRosNodePlugin），
    // 不能用 registerFromPlugin 注册了，改到下面用 RegisterRosNode 注册
    "print_message",
    "is_time_to_bullet",
    "is_gimbal_target_valid",
    "wait_duration",
    "rm_set_blackboard",
    "get_blackboard",
  };
  // clang-format on

  for (const auto & p : msg_update_plugin_libs) {
    RegisterRosNode(factory, BT::SharedLibrary::getOSName(p), params_update_msg);
  }

  // 注册需要 ROS2 节点参数的节点
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_game_time"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_status_ok"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_remote_status"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_auto_aim_ok"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_outpost_ok"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_enemy_outpost_ok"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_attacked"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_base_attacked"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_base_status_ok"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_chassis_energy_ok"), params_update_msg);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_rune_status"), params_update_msg);

  // 注册 IsSemantic 节点（对比当前语义与 Groot2 设定值）
  // topic_name 默认给相对名 "terrain_zone"：行为树用 -r __ns:=/<ns> 启动时
  // 会自动解析到 /<ns>/terrain_zone（tunnel_mode_manager 在导航命名空间里发布）
  BT::RosNodeParams params_semantic;
  params_semantic.nh = params_update_msg.nh;
  params_semantic.default_port_value = "terrain_zone";
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("is_semantic"), params_semantic);

  // 注册 PursueEnemy 节点
  BT::RosNodeParams params_pursue_enemy;
  params_pursue_enemy.nh = std::make_shared<rclcpp::Node>("pursue_enemy");
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pursue_enemy"), params_pursue_enemy);

  // 注册 GetCurrentLocation 节点
  BT::RosNodeParams params_get_current_location;
  params_get_current_location.nh = std::make_shared<rclcpp::Node>("get_current_location");
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("get_current_location"), params_get_current_location);

  // 注册 PursueGoal 节点（MoveAround 模式的追击点规划器：只算点，导航交给 SendGoal）
  // ⚠ 它直接 node_->create_subscription 订阅 /map（贴墙检查用），属于「裸订阅」，
  //   所以必须复用已在 executor 里的节点，不能自建节点
  BT::RosNodeParams params_pursue_goal;
  params_pursue_goal.nh = params_update_msg.nh;
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pursue_goal"), params_pursue_goal);

  // 注册 MoveAround 节点（随机游走规划器）
  // 复用 update_msg 节点：它已经在下面的 executor.add_node 名单里，
  // 所以 /map 的订阅回调才会真的被执行（回调没被 spin = 永远收不到地图）
  BT::RosNodeParams params_move_around;
  params_move_around.nh = params_update_msg.nh;
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("move_around"), params_move_around);

  for (const auto & p : bt_plugin_libs) {
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(p));
  }

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_goal"), params_send_goal);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("robot_control"), params_robot_control);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_robot_state"), params_robot_state);
  
  // 注册发布节点（使用默认参数，不需要特殊的 RosNodeParams）
  BT::RosNodeParams params_pub_default;
  params_pub_default.nh = std::make_shared<rclcpp::Node>("pub_default");
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_auto_aim_mode"), params_pub_default);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_chassis_mode"), params_pub_default);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_super_capacitor"), params_pub_default);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_enemy_status"), params_pub_default);
  RegisterRosNode(factory, BT::SharedLibrary::getOSName("pub_autoaim_target"), params_pub_default);

  auto tree = factory.createTreeFromFile(bt_xml_path);

  // Connect the Groot2Publisher. This will allow Groot2 to get the tree and poll status updates.
  const unsigned port = 1667;
  BT::Groot2Publisher publisher(tree, port);

  // 创建一个 executor 来处理主节点回调
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(params_update_msg.nh);
  executor.add_node(params_get_current_location.nh);
  executor.add_node(params_send_goal.nh);
  executor.add_node(params_pub_default.nh);

  while (rclcpp::ok()) {
    // 关键：不能用 tree.tickWhileRunning()。它对 RUNNING 的树永不返回，
    // 会让 executor 只在启动时 spin 一次，之后所有走主 executor 的订阅
    // （例如 SubTargetPos 的 /auto_aim_target_pos）永远收不到数据 →
    // 前出/追击分支失效。正确做法是"spin → 单次 tick → 睡一下"循环。
    executor.spin_some(std::chrono::milliseconds(5));
    tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  rclcpp::shutdown();
  return 0;
}