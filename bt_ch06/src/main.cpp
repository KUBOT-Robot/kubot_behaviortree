/*
樹(tree)與子樹(subtre)之間的端口映射(remapping port)
為了避免名稱衝突，所以主樹與子樹之間的blackboard會是不同的存在，
主樹在呼叫子樹時，同時能夠與子樹之間的blackboard連結，
連結的方式就在xml而非c++中，語法如下：

主樹
<BehaviorTree ID="MainTree">
    <Sequence name="main_sequence">
        <SetBlackboard output_key="move_goal" value="1;2;3" />
        <SubTree ID="MoveRobot" target="move_goal" output="move_result" />
        <SaySomething message="{move_result}"/>
    </Sequence>

</BehaviorTree>

子樹
<BehaviorTree ID="MoveRobot">
    <Fallback name="move_robot_main">
        <SequenceStar>
            <MoveBase       goal="{target}"/>
            <SetBlackboard output_key="output" value="mission accomplished" />
        </SequenceStar>
        <ForceFailure>
            <SetBlackboard output_key="output" value="mission failed" /
        </ForceFailure>
    </Fallback>
</BehaviorTree>


可以看見主樹宣告了一個Blackeboard名字叫move_goal，數值是1,2,3
呼叫了子樹，傳入Blackboard參數move_goal，並開了一個output port，傳出來的參數命名為move_result
呼叫子樹，並將move_goal傳入，並在子樹內命名為target
然後主樹呼叫SaySomething節點，並傳入move_result變數

子樹則接收target這個變數，並放入MoveBase節點，並宣告Blackboard，放入output_key指向的變數，並宣告數值

*/
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "move_base.h"
#include "say_something.h"

using namespace BT;

int main(int argc,char **argv){
    ros::init(argc,argv,"bt_ch06_node");
    ros::NodeHandle nh("~");
    std::string xml_filename;

    nh.param<std::string>("file",xml_filename,"/home/catkin_ws/src/bt_test/bt_test_xml/bt_test_xml.xml");
    ROS_INFO("LoadingXML : %s",xml_filename.c_str());

    BehaviorTreeFactory factory;

    factory.registerNodeType<MoveBase>("MoveBase");
    factory.registerNodeType<SaySomething>("SaySomething");

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