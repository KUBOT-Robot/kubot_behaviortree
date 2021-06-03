
#pragma once

#include <behaviortree_cpp_v3/action_node.h>

using namespace BT;

// We want to wrap into an ActionNode the methods open() and close()
class GripperInterface{
    public:
        GripperInterface(): _open(true) {}

        BT::NodeStatus open(){
            _open = true;
            std::cout << "GriperInterface::open" << std::endl;
            return NodeStatus::SUCCESS;
        }

        BT::NodeStatus close(){
            std::cout << "GripperInterface::close" << std::endl;
            _open = false;
            return NodeStatus::SUCCESS;
        }
    private:
        bool _open; // shared information
};