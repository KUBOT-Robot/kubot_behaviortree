#pragma once

#include <behaviortree_cpp_v3/action_node.h>

using namespace BT;

class PassThroughDoor : public SyncActionNode{
    public:
        PassThroughDoor(const std::string& name):
            SyncActionNode(name,{})
            {
            }

        NodeStatus tick() override
        {
            std::cout << "Pass throug door" << std::endl;
            return NodeStatus::SUCCESS;
        }
};