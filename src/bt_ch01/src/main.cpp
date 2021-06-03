/* 
    behaviortree_cpp_v3.cpp實做，建立第一棵Behavior Tree
    主要有：
    ．ros節點建立behaviortree_cpp_v3之中AcionNode的建立
    ．SimpleCondition兩種建立方式
    ．xml格式與建立方式
    ．樹的監控方式
    ．roslaunch啟動檔案內容與啟動方式
*/
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "action_test.h"
#include "check_battery.h"
#include "gripper_interface.h"

using namespace BT;

int main(int argc,char **argv){
    ros::init(argc,argv,"bt_ch01_node");

    ros::NodeHandle nh("~");
    std::string xml_filename;

    nh.param<std::string>("file",xml_filename,"/home/catkin_ws/src/bt_test/bt_test_xml/bt_test_xml.xml");
    ROS_INFO("LoadingXML : %s",xml_filename.c_str());

    BehaviorTreeFactory factory;
    
    // 一般建立Node時是建議使用繼承
    // 此為ActionNode，屬於葉，不會有子節點，負責執行動作
    factory.registerNodeType<ApproachObject>("ApproachObject");

    // 在建立SimpleActionNode時使用函式指標(function pointer)
    // 如果使用的是C++11，可以考慮使用lambdas來代替std::bind()
    /*  此為ConditionNode，類似於ActionNode，
          但不同的是，這類的節點必須是不可被分割、運行中不可中斷，同時他們會與主程式並行處理，
          狀態不可回傳RUNNING，也不應該改變系統狀態*/
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

    //用另外一種方式創造SimpleActionNode
    GripperInterface gripper;
    factory.registerSimpleCondition("OpenGripper",std::bind(&GripperInterface::open,&gripper));
    factory.registerSimpleCondition("CloseGripper",std::bind(&GripperInterface::close,&gripper));

    // 建立樹
    auto tree = factory.createTreeFromFile(xml_filename);
    
    // 建立監控者
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