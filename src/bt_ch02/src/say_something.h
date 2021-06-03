// SyncActionNode (synchronous action) with an input port.
#pragma once
#include <behaviortree_cpp_v3/action_node.h>


class SaySomething : public BT::SyncActionNode
{
    public:
        // 如果我今天要建立的是通訊端口(port)，就必須使用這樣子的建構子
        SaySomething(const std::string& name,const BT::NodeConfiguration& config):BT::SyncAcionNode(name,config)
        {}

        // 必須宣告成靜態
        static BT::PortsList providedPorts()
        {
            // 此動作是在命名一個端口名叫""message"
            // 所有的端口都有名字，而端口的型態可以選擇
            return{ InputPort<std::string>("message")};
        }

        // 同樣的tick()必須override
        BT::NodeStatus tick() override
        {
            Optional<std::string> msg = getInput<std::string>("message");
            // 檢查Optional是否運行正常，如果沒有，警告錯誤
            if(!msg)
            {
                throw BT::RuntimeError("missing required input [message] : ",msg.error());
            }
            // 用value()的方法去提取input的內容
            std::cout << "Robot says : " msg.value() << std::endl;
            return NodeStatus::SUCCESS;
        }
}