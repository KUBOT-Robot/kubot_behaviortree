
#pragma once
#include <behaviortree_cpp_v3/action_node.h>

// SyncActionNode (synchronous action) with an input port.
class SaySomething : public BT::SyncActionNode
{
    public:
        // 如果我今天要建立的是通訊端口(port)，就必須使用這樣子的建構子
        SaySomething (const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
        {}

        // 必須宣告成靜態
        static BT::PortsList providedPorts()
        {
            // 此動作是在命名一個端口名叫""message"
            // 所有的端口都有名字，而端口的型態可以選擇
            return{ BT::InputPort<std::string>("message")};
        }

        // 同樣的tick()必須override
        /* * *
          *     override的中文翻譯叫做複寫，虛擬函式的安全措施，
          *     是用來告訴編譯器這個函式現在是要複寫虛擬函式，若未有虛擬函式宣告，
          *     編譯器會報錯
        * * */
        // 呼叫到這個節點的時候，把message裡面的內容顯示到終端機上面
        BT::NodeStatus tick() override
        {
            std::string expect_message;
            // 檢查是否運行正常，如果沒有，警告錯誤
            if(!getInput<std::string>("message",expect_message))
            {
                throw BT::RuntimeError("missing required input [message] : ");
            }
            // 顯示從input port裡拿到的內容
            std::cout << "Robot says : " << expect_message.c_str()<< std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};