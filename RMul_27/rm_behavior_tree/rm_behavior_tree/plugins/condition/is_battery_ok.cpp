#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/robot_batterystatus.hpp"
#include "robot_msgs/msg/robot_batterystatus.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace rm_behavior_tree
{
class IsBatteryOKAction : public BT::SimpleConditionNode
{
public:
  IsBatteryOKAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SimpleConditionNode(name, std::bind(&IsBatteryOKAction::checkRobotBattery, this), config)
   {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<robot_msgs::msg::RobotBatteryStatus>("message"),
      BT::InputPort<int>("bt_threshold")
      };
  }
private:
  BT::NodeStatus checkRobotBattery()
  {
    int bt_threshold;
    auto msg = getInput<robot_msgs::msg::RobotBattery>("message");
    /*
    getInput<T>() 返回的是一个 BT::Expected<T> 类型（类似于 std::optional<T>）。
    如果取成功，msg 就是一个 包含消息对象的“智能封装”
    */
    getInput("bt_threshold",bt_threshold);

    if (!msg)//==if (msg.RobotBattery() == false)
    {
        std::cerr << "Missing input [message]" << std::endl;
        return BT::FAILURE;
    }

    if (msg->current_bt < bt_threshold)
    {
        std::cout<<"The battery is not enough!!!"<<std::endl;
        return BT::NodeBattery::FAILURE;
    }
    else{
        std::cout<<"The battery is enough right now"<<std::endl;
        return BT::NodeBattery::SUCCESS;
    }
    
  }
};
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsBatteryOKAction>("IsBatteryOK");
}
