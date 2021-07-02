#pragma once

#include <behaviortree_cpp_v3/action_node.h>

using namespace BT;

class PassThroughWindow : public SyncActionNode{
    public:
        PassThroughWindow(const std::string& name):
            SyncActionNode(name,{})
            {
            }

        NodeStatus tick() override
        {
            std::cout << "Pass throug window" << std::endl;
            return NodeStatus::SUCCESS;
        }
};