#pragma once

#include <behaviortree_cpp_v3/action_node.h>

using namespace BT;

class MoveBase : public SyncActionNode{
    public:
        MoveBase(const std::string& name,const NodeConfiguration& config)
        : SyncActionNode(name,config)
        {
        }

        static PortsList providedPorts()
        {
            return {InputPort<std::string>("goal")};
        }

        NodeStatus tick() override
        {
            std::string move_goal;
            if(!getInput<std::string>("goal",move_goal))
            {
                throw RuntimeError("missing move goal");
            }
            std::cout<<"Move goal : "<< move_goal.c_str()<<std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};