#include "rm_behavior_tree/plugins/control/decision_switch.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace rm_behavior_tree
{
    DecisionSwitch::DecisionSwitch(const std::string & name, const BT::NodeConfig & config)
    : BT::ControlNode(name, config)
    {
    }

    BT::NodeStatus DecisionSwitch::tick()
    {
        auto msg = getInput<rm_decision_interfaces::msg::DecisionNum>("decision_num");

        if(!msg)
        {
            std::cout << "missing required input [decision_num]" << '\n';
            if(active_child_ >= 0)
            {
                haltChild(static_cast<size_t>(active_child_));
                active_child_ = -1;
            }
            return BT::NodeStatus::FAILURE;
        }

        // decision_num: 1..N -> children_nodes_[0..N-1]
        const int index = static_cast<int>(msg->decision_num) - 1;
        if(index < 0 || static_cast<size_t>(index) >= childrenCount())
        {
            std::cout << "[DecisionSwitch] invalid decision_num="
                      << static_cast<int>(msg->decision_num)
                      << " children=" << childrenCount() << '\n';
            if(active_child_ >= 0)
            {
                haltChild(static_cast<size_t>(active_child_));
                active_child_ = -1;
            }
            return BT::NodeStatus::FAILURE;
        }

        // 切换意图时 halt 旧子树，避免上一个 SendGoal / PursueEnemy 还在跑
        if(active_child_ != index)
        {
            if(active_child_ >= 0)
            {
                haltChild(static_cast<size_t>(active_child_));
            }
            active_child_ = index;
        }

        // 原实现只在子节点 SUCCESS 时返回 SUCCESS，RUNNING 会被压成 FAILURE；
        // 长时动作（SendGoal/PursueEnemy）必须把 RUNNING 原样传出去。
        return children_nodes_[static_cast<size_t>(index)]->executeTick();
    }

    void DecisionSwitch::halt()
    {
        if(active_child_ >= 0)
        {
            haltChild(static_cast<size_t>(active_child_));
            active_child_ = -1;
        }
        BT::ControlNode::halt();
    }
}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::DecisionSwitch>("DecisionSwitch");
}