/*
Sequeue與ReactiveSequeue兩種不同框架下的Sequeue節點運行狀態
Sequeue的運作原理是，tick子節點時，
若子節點回傳FAILURE   ，則在下次運行的時候，從頭開始運行
若子節點回傳RUNNING，則在下次運行的時候，從剛剛回傳RUNNING的節點開始運行

ReactiveSequeue的運作原理是，tick子節點時，
若子節點回傳FAILURE   ，則在下次運行的時候，從頭開始運行
若子節點回傳RUNNING，則在下次運行的時候，從頭開始運行

因此這章節有兩個xml檔，
bt_ch04_xml.xml為ReactiveSequeue
bt_ch04_1_xml.xml為Sequeue

兩者運行的結果可以看出其不同

Sequeue的運行結果會顯示：
  -No:1
  -[ CheckBattery OK ]
  -Robot says : mission started...
  -[ MoveBase: STARTED ]. goal: x=1 y=2.0 theta=3.00
  -No:2
  -[ MoveBase: FINISHED ]
  -No:3
  -Robot says : mission copleted!
可以注意到No:1與No:2的時候，MoveBase回傳了RUNNING，No:3的時候才回傳了SUCCESS，BatteryOK只執行了一次

ReactiveSequeue的運行結果會顯示：
-No:1
-[ CheckBattery OK ]
-Robot says : mission started...
-[ MoveBase: STARTED ]. goal: x=1 y=2.0 theta=3.00
-No:2
-[ CheckBattery OK ]
-Robot says : mission started...
-[ MoveBase: FINISHED ]
-No:3
-[ CheckBattery OK ]
-Robot says : mission started...
-Robot says : mission copleted!
可以注意到BatteryOK被執行的三次，意思是執行了三次，而且三次都是重頭開始執行

*/
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "move_base_action.h"
#include "say_something.h"
#include "check_battery.h"


using namespace BT;
int main(int argc,char **argv){
    ros::init(argc,argv,"bt_ch04_node");

    ros::NodeHandle nh("~");
    std::string xml_filename;

    nh.param<std::string>("file",xml_filename,"/home/behaviortree_test/src/bt_ch04/bt_ch04_xml/bt_ch04_xml.xml");
    ROS_INFO("LoadingXML : %s",xml_filename.c_str());


    BehaviorTreeFactory factory;
    factory.registerSimpleCondition("BatteryOK",std::bind(CheckBattery));
    factory.registerNodeType<MoveBaseAction>("MoveBase");
    factory.registerNodeType<SaySomething>("SaySomething");

    auto tree = factory.createTreeFromFile(xml_filename);

    //StdCoutLogger logger_cout(tree); //會將每個節點運行的狀態顯示在終端機上面，但這次的練習會造成畫面雜亂，所以先註解掉，需要的人再解除註解


    NodeStatus status = NodeStatus::RUNNING;
    int i = 1;
    while(ros::ok() && status == NodeStatus::RUNNING){
      std::cout << "No:" << i << std::endl;
      status = tree.rootNode()->executeTick();
      std::this_thread::sleep_for(std::chrono::milliseconds(150)); 
      i++;
      if(i > 3) i = 1;
    }

  return 0;
}