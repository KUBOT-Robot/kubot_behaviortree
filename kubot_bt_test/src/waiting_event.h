#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>


class WaitingEvent : public AsyncActionNode{
public:
   WaitingEvent(const std::string& name, const BT::NodeConfiguration& config):
      BT::AsyncActionNode(name, config)
   {

   }

   virtual BT::NodeStatus tick() override{
      ros::Duration(0.5).sleep();
      return BT::NodeStatus::SUCCESS;
   }
}