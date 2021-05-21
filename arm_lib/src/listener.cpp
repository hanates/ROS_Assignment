
#include "ros/ros.h"
#include "arm_lib/input.h"


void chatterCallback(const arm_lib::input &msg)
{
  ROS_INFO("I heard: %f, %f % f", msg.x, msg.y, msg.z);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "input_listener");

  
  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}