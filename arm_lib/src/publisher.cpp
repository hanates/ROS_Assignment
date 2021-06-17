
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "arm_lib/arm_joint_angles.h"
#include "arm_lib/input.h"

#include <stdlib.h>   

#include <sstream>

void send_message(ros::Publisher &, arm_lib::input);
void publishAngles();

int main(int argc, char **argv)
{
  publishAngles();
  return 0;
}

void send_message(ros::Publisher &chatter_pub, arm_lib::input inp_msg)
{
  chatter_pub.publish(inp_msg);
}

void send_input_msg()
{

  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "input_publisher");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Publisher chatter_pub = n.advertise<arm_lib::input>("chatter", 1000);
  double a = 0;
  while (ros::ok())
  {
    arm_lib::input input_message;
    input_message.x = 1 + a;
    input_message.y = 2;
    input_message.z = 3;
    input_message.alpha = 1;
    input_message.beta = 2;
    input_message.gamma = 3;
    input_message.d = 4;

    ROS_INFO("%f", input_message.x);
    send_message(chatter_pub, input_message);

    ros::spinOnce();
    loop_rate.sleep();
    a++;
  }
}

void publishAngles()
{
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "change_angles");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);
  ros::Publisher pub = n.advertise<arm_lib::arm_joint_angles>("change_angles", 1000);
  double a = 0;
  // while (ros::ok())
  // {
  //   // arm_lib::arm_joint_angles msg;
  //   // msg.z0 = rand() % 45;
  //   // msg.x1 = rand() % 45;
  //   // msg.x2 = rand() % 45;
  //   // msg.x3 = rand() % 45;

  //   // ROS_INFO("%f %f %f %f", msg.z0, msg.x1, msg.x2, msg.x3);
  //   // pub.publish(msg);
  //   // ros::Duration(3.0).sleep();
  //   ros::spinOnce();
    
  // }
    
  
}
