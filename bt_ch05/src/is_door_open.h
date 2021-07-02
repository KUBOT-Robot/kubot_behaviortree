#pragma once

#include <behaviortree_cpp_v3/action_node.h>

using namespace BT;

class IsDoorOpen : public SyncActionNode{
    public:
        IsDoorOpen(const std::string& name):
            SyncActionNode(name,{})
            {
                _open = false;
            }
        
        NodeStatus tick() override
        {
            if(_open == true){
                std::cout << "Door is open" << std::endl;
                return NodeStatus::SUCCESS;
            }
            std::cout << "Door is close"<<std::endl;
            return NodeStatus::FAILURE;
        }
        

    private:
        bool _open;
};