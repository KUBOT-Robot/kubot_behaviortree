/*
behaviortree_cpp_v3實做，input與output
input port與output port的宣告
input port又分成class版(SaySomething)與一般function版(SaySomethingSimple2)

此範例的運行順序
1. input port -> SaySomething 讀取在xml已經設定好內容的字串，名叫message，內容是start thinking...，
    會在終端機顯示start thinking...
    xml語法：
    <SaySomething     message="start thinking..." />

2. output port -> ThinkWhatToSay 將一些內容寫入blackboard，寫入名叫the_answer的blackboard位置，
    寫入內容為The answer is 42，不會在終端機上顯示
    xml語法：
    <ThinkWhatToSay   text="{the_answer}"/>

3. input port -> SaySomething 讀取黑板內容數值，將已經有數值的黑板裡面的變數the_answer寫入message，
    此時的message是The answer is 42，會在終端機顯示The answer is 42
    xml語法：
    <SaySomething     message="{the_answer}" />

4. input port -> SaySometing2 讀取在xml已經設定好內容的字串，名叫message，內容是SaySomething2 works too...，
    會在終端機顯示SaySomething2 works too...
    xml語法：
    <SaySomething2    message="SaySomething2 works too..." />

5. input port -> SaySomething2 讀取黑板內容數值，將已經有數值的黑板裡面的變數the_answer寫入message，
    此時的message是The answer is 42，會在終端機顯示The answer is 42
    xml語法：
    <SaySomething2    message="{the_answer}" />


***重點***
input port應該是說，可以從黑板上拿到某數值後，寫入自身宣告的變數
output port可以更改黑板內容

***使用範例***
input port -> 拿黑板數值寫入變數
output port -> 寫入黑板數值
*/
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "say_something.h"
#include "say_something2.h"
#include "Think_what_to_say.h"

using namespace BT;

int main(int argc,char **argv){
    ros::init(argc,argv,"bt_ch02_node");

    ros::NodeHandle nh("~");
    std::string xml_filename;

    nh.param<std::string>("file",xml_filename,"/home/behaviortree_test/src/bt_ch02/bt_ch02_xml/bt_ch02_xml.xml");
    ROS_INFO("LoadingXML : %s",xml_filename.c_str());

    BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething"); // ActionNode宣告，input port，class版
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay"); // ActionNode宣告，output port
    
    // ActionNode宣告，input port，function版
    PortsList say_something_ports = {InputPort<std::string>("message")};
    factory.registerSimpleAction("SaySomething2",SaySomethingSimple2,say_something_ports);

    auto tree = factory.createTreeFromFile(xml_filename);

    StdCoutLogger logger_cout(tree);


    NodeStatus status = NodeStatus::RUNNING;//預設節點robot_sequeue狀態目前為RUNNING
    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while (ros::ok() && status == NodeStatus::RUNNING) {  // ROS運行OK以及robot_squeue狀態為RUNNING 就開始運行
        status = tree.rootNode()->executeTick();  // status變成現在樹正在運行的節點，並顯示它目前運行的狀態
        // Sleep 100 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等個0.1秒
    }

  return 0;
}