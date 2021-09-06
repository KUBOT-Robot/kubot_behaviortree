#include "movebase_client.h"
#include "interrupt_event.h"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

using namespace BT;

int main(int argc, char **argv) {
   ros::init(argc, argv, "test_bt");

   ros::NodeHandle nh("~");
   std::string xml_filename;
   nh.param<std::string>
}