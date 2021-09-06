#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "my_async_action.h"

using namespace BT;
int main(int argc,char **argv){
    ros::init(argc,argv,"bt_ch09_node");
    ros::NodeHandle nh("~");
    std::string xml_filename;

    nh.param<std::string>("file",xml_filename,"/home/behaviortree_test/src/bt_ch09/xml/bt_ch09_xml.xml");
    ROS_INFO("LoadingXML : %s",xml_filename.c_str());

    BehaviorTreeFactory factory;

    factory.registerNodeType<MyAsyncAction>("MyAsyncAction");

    auto tree = factory.createTreeFromText(xml_filename);

    //---------------------------------------
    // keep executin tick until it returns etiher SUCCESS or FAILURE
    // 持續執行tick直到他回傳SUCCESS或FAILURE
    /*while (tree.tickRoot() == NodeStatus::RUNNING)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }*/

    NodeStatus status = NodeStatus::RUNNING;//預設節點robot_sequeue狀態目前為RUNNING
    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while (ros::ok() && status == NodeStatus::RUNNING) {  // ROS運行OK以及robot_squeue狀態為RUNNING 就開始運行
        status = tree.rootNode()->executeTick();  // status變成現在樹正在運行的節點，並顯示它目前運行的狀態
        // Sleep 100 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等個0.1秒
    }
    

    return 0;
}

/* Expected output:

action_A: Started. Send Request to server.
action_A: Waiting Reply...
action_A: Done. 'Waiting Reply' loop repeated 11 times
action_A: cleaning up after SUCCESS
action_B: Started. Send Request to server.
action_B: Waiting Reply...
action_B: Halted.
action_B: cleaning up after an halt()

*/