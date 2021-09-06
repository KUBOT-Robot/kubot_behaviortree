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

    BehaviorTreeFatory factory;

    factory.registerNodeType<MyAsyncAction>("MyAsyncAction");

    auto tree = factory.createTreeFromText(xml_filename);

    //---------------------------------------
    // keep executin tick until it returns etiher SUCCESS or FAILURE
    // 持續執行tick直到他回傳SUCCESS或FAILURE
    while (tree.tickRoot() == NodeStatus::RUNNING)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
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