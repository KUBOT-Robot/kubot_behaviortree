#pragma once
#include <behaviortree_cpp_v3/action_node.h>

// Example of custom SyncActionNode (synchronous action)
// without ports.
class ApproachObject : public BT::SyncActionNode
{
  public:
    ApproachObject(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    // tick()是母節點對子節點下達命令，到達這個節點時，會使用這個節點的tick()成員，同樣這個動作也叫做tick()
    BT::NodeStatus tick() override
    {
        std::cout << "ApproachObject: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};