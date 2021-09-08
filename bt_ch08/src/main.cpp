#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "action_a.h"
#include "action_b.h"

using namespace BT;
int main(int argc,char **argv){
    ros::init(argc,argv,"bt_ch08_node");
    ros::NodeHandle nh("~");
    std::string xml_filename;

    nh.param<std::string>("file",xml_filename,"/home/behaviortree_test/src/bt_ch08/xml/bt_ch08_xml.xml");
    ROS_INFO("LoadingXML : %s",xml_filename.c_str());

    BehaviorTreeFactory factory;

    //node builder只是一個函式指針用來創建std::unique_ptr<TreeNode>
    //使用lambdas或std::bind可以輕鬆注入額外的參數
    NodeBuilder builder_A =[](const std::string& name, const NodeConfiguration& config) 
    {
        return std::make_unique<Action_A>( name, config, 42, 3.14, "hello world, action_a" );
    };

    factory.registerBuilder<Action_A>( "Action_A", builder_A);
    
    // Action_B用正常註冊就行，但我們依舊需要先呼叫Action_B::init()
    factory.registerNodeType<Action_B>( "Action_B" );

    auto tree = factory.createTreeFromFile(xml_filename);

    for( auto& node: tree.nodes )
    {
        // Not a typo: it is "=", not "=="
        if( auto action_B = dynamic_cast<Action_B*>( node.get() ))
        {
            action_B->init( 42, 3.14, "hello world, action_b");
        }
    }

<<<<<<< HEAD
    
    // 建立監控者
    StdCoutLogger logger_cout(tree);
    
    NodeStatus status = NodeStatus::RUNNING;//預設節點robot_sequeue狀態目前為RUNNING

    // Keep on ticking until you get either a SUCCESS or FAILURE state
  while (ros::ok() && status == NodeStatus::RUNNING) {  // ROS運行OK以及robot_squeue狀態為RUNNING 就開始運行
    status = tree.rootNode()->executeTick();  // status變成現在樹正在運行的節點，並顯示它目前運行的狀態
    // Sleep 100 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等個0.1秒
    }
=======
    tree.tickRoot();
>>>>>>> 08614cbd957be768068a29b653a0ccb21903905a
    return 0;
}