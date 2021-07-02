/*
Subtree的建立，並與Behaviors結合
樹可以建立非常大一顆樹，同時也可以將數分割成好幾個小樹後再做結合
被分割出去的小樹被稱作Subtree
此章節就在教說要如何建立Subtree與結合Subtree
在bt_ch05_xml.xml中，樹被分成MainTree與DoorClosed兩顆樹
MainTree中又呼叫了DoorClosed這棵樹：<SubTree ID="DoorClosed">就是在主程式中呼叫SubTree的方法

註：分別將is_door_open.h中IsDoorOpen建構子中的_open變數修改成true
                       open_door.h中OpenDoor中的line 18：open_door_num++; 註解掉，可以得到不同運行結果
*/
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "open_door.h"
#include "pass_through_window.h"
#include "pass_through_door.h"
#include "is_door_open.h"

using namespace BT;

int main(int argc,char **argv){
    ros::init(argc,argv,"bt_ch05_node");
    ros::NodeHandle nh("~");
    std::string xml_filename;

    nh.param<std::string>("file",xml_filename,"/home/catkin_ws/src/bt_test/bt_test_xml/bt_test_xml.xml");
    ROS_INFO("LoadingXML : %s",xml_filename.c_str());

    BehaviorTreeFactory factory;

    factory.registerNodeType<OpenDoor>("OpenDoor");
    factory.registerNodeType<PassThroughDoor>("PassThroughDoor");
    factory.registerNodeType<PassThroughWindow>("PassThroughWindow");
    factory.registerNodeType<IsDoorOpen>("IsDoorOpen");

    auto tree = factory.createTreeFromFile(xml_filename);

    NodeStatus status = NodeStatus::RUNNING;//預設節點robot_sequeue狀態目前為RUNNING

    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while (ros::ok() && status == NodeStatus::RUNNING) {  // ROS運行OK以及robot_squeue狀態為RUNNING 就開始運行
        status = tree.rootNode()->executeTick();  // status變成現在樹正在運行的節點，並顯示它目前運行的狀態
        // Sleep 100 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等個0.1秒
    }

    
    return 0;
}