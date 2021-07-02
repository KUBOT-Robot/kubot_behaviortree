#pragma once

#include <behaviortree_cpp_v3/action_node.h>

using namespace BT;

class OpenDoor : public SyncActionNode{
    public:
        OpenDoor(const std::string& name):
            SyncActionNode(name,{})
            {
                open_door_num = 0;
            }

        NodeStatus tick() override
        {
            if(open_door_num < 3){
                open_door_num++;
                std::cout << "Open door failure" << std::endl;
                std::cout << "No :" << open_door_num <<std::endl;
                return NodeStatus::FAILURE;
            }
            else if(open_door_num >= 3){
                open_door_num = 0;
                std::cout << "Open door SUCCESS" << std::endl;
            return NodeStatus::SUCCESS;
            }
        }
    
    private:
        int open_door_num;
};