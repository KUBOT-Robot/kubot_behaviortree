#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "action_test.h"
#include "my_async_action.h"

using namespace BT;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bt_ch09_node");

    ros::NodeHandle nh("~");
    std::string xml_filename;

    nh.param<std::string>("file", xml_filename, "/home/catkin_ws/src/bt_test/bt_test_xml/bt_test_xml.xml");
    ROS_INFO("LoadingXML : %s", xml_filename.c_str());

    BehaviorTreeFactory factory;

    // 一般建立Node時是建議使用繼承
    // 此為ActionNode，屬於葉，不會有子節點，負責執行動作
    factory.registerNodeType<MyAsyncAction>("ApproachObject"); // 這個是class的版本

    // 建立樹
    auto tree = factory.createTreeFromFile(xml_filename);

    // 建立監控者
    //StdCoutLogger logger_cout(tree);

    NodeStatus status = NodeStatus::RUNNING; //預設節點robot_sequeue狀態目前為RUNNING

    while( tree.tickRoot() == NodeStatus::RUNNING)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 等個0.1秒
    }

    return 0;
}