#include "ros/ros.h"
#include "esmacat_pkg/message.h"

void chatterCallback(const esmacat_pkg::message::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->name.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
