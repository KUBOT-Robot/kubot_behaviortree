#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

// Simple function that return a NodeStatus
BT::NodeStatus CheckBattery()
{
    std::cout << "[ CheckBattery OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}
