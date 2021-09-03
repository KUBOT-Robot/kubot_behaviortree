/*
包裝遺留代碼
這篇主要在講說如何將原本在程式中的class用behaviortree包裝起來，
因為是非侵入性的修改，因此能夠直接將class包裝起來後用behaviortree的方式呼叫

*/

#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

using namespace BT;

struct Point3D
{
    double x,y,z;
};

//  這是尚未加入behaviortree時的程式碼
//  我們要創造一個actionnode來呼叫MyLegazyMoveTo
class MyLegazyMoveTo{
    public:
        bool go(Point3D goal){
            printf("Going to %f %f %f \n",goal.x,goal.y,goal.z);
            return true;//  如果我終端機顯示出這段訊息，代表我呼叫成功
        }
};

namespace BT
{
template <> Point3D convertFromString(StringView key){
    auto parts = BT::splitString(key,';');//    用分號分割的三個數字
    if(parts.size() != 3){
        throw RuntimeError("invalid input");
    }
    else
    {
        Point3D output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        output.z = convertFromString<double>(parts[2]);
        return output;
    }
}
}// end namespace BT

int main(int argc,char **argv){
    ros::init(argc,argv,"bt_ch07_node");
    ros::NodeHandle nh("~");
    std::string xml_filename;

    nh.param<std::string>("file",xml_filename,"/home/behaviortree_test/src/bt_ch07/xml/bt_ch07_xml.xml");
    ROS_INFO("LoadingXML : %s",xml_filename.c_str());

    //  可以使用Lambda或是std::bind
    MyLegazyMoveTo move_to;
    auto MoveToWrapperWithLambda = [&move_to](TreeNode& parent_node) -> NodeStatus
    {
        Point3D goal;

        //  parent_node能夠輕鬆訪問輸入與輸出端口
        parent_node.getInput("goal",goal);

        bool res = move_to.go(goal);
        //  將布林數改成NodeStatus
        return res ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    };

    BehaviorTreeFactory factory;

    PortsList ports = {BT::InputPort<Point3D>("goal")};
    factory.registerSimpleAction("MoveTo",MoveToWrapperWithLambda,ports);

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