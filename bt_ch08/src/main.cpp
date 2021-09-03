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

    nh.param<std::string>("file",xml_filename,"/home/behaviortree_test/src/bt_ch08/bt_ch08/bt_ch08_xml.xml");
    ROS_INFO("LoadingXML : %s",xml_filename.c_str());

    BehaviorTreeFactory factory;

    //  node builder只是一個函式指針用來創建std::unique_ptr<TreeNode>
    //  使用lambdas或std::bind可以輕鬆注入額外的參數
    NodeBuilder builder_A =[](const std::string& name, const NodeConfiguration& config) 
    {
        return std::make_unique<Action_A>( name, config, 42, 3.14, "hello world" );
    };

    factory.registerBuilder<Action_A>( "Action_A", builder_A);
    
    // Action_B用正常註冊就行，但我們依舊需要先呼叫Action_B::init()
    //factory.registerNodeType<Action_B>( "Action_B" );

    auto tree = factory.createTreeFromText(xml_filename);

    NodeStatus status = NodeStatus::RUNNING;

    while (ros::ok() && status == NodeStatus::RUNNING) { 
        status = tree.rootNode()->executeTick();
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }
    return 0;
}