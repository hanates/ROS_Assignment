#include "arm_lib/arm_joint_angles.h"
#include "ros/ros.h"


void chatterCallback(const arm_lib::arm_joint_angles &msg)
{
  ROS_INFO("I heard: %f, %f, % f, %f", msg.z0, msg.x1, msg.x2, msg.x3);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "angle_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joint_angles", 1000, chatterCallback);

  ros::spin();

  return 0;
}