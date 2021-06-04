#pragma once
#include <behaviortree_cpp_v3/action_node.h>

// SyncActionNode (synchronous action) with an output port.
class ThinkWhatToSay: public BT::SyncActionNode
{
    public:
         ThinkWhatToSay (const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
        {}

        static BT::PortsList providedPorts()
        {
            return {BT::OutputPort<std::string>("text")};
        }

        // This Action writes a value into the port "text"
        BT::NodeStatus tick() override
        {
            // the output may change at each tick(). Here we keep it simple.
            setOutput("text","The answer is 42");
            return BT::NodeStatus::SUCCESS;
        }
};