#include <behaviortree_cpp_v3/action_node.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

// Simple function that return a NodeStatus
BT::NodeStatus SaySomethingSimple2(BT::TreeNode& self)
{
    
    std::string expect_message;
    // 檢查是否有拿到數值，如果沒有，警告錯誤
    if(!self.getInput<std::string>("message",expect_message))
    {
        throw BT::RuntimeError("missing required input [message]: ");
    }
    // 顯示message內容
    std::cout << "Robot says " << expect_message.c_str() << std::endl;
    return BT::NodeStatus::SUCCESS;
}