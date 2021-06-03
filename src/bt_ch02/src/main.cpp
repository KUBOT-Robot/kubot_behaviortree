/*
behaviortree_cpp_v3實做，input與output
*/
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
int main(int argc,char **argv){
    ros::init(argc,argv,"bt_ch02_node");

    ros::NodeHandle nh("~");
    std::string xml_filename;

    nh.param<std::string>("file",xml_filename,"/home/behaviortree_test/src/bt_ch02/bt_ch02_xml/bt_ch02_xml.xml");
    ROS_INFO("LoadingXML : %s",xml_filename.c_str());

  return 0;
}