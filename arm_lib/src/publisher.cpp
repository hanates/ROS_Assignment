
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "arm_lib/input.h"


#include <sstream>

void send_message(ros::Publisher &, arm_lib::input);

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "input_publisher");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Publisher chatter_pub = n.advertise<arm_lib::input>("chatter", 1000);
  double a = 0;
  while (ros::ok()){
      arm_lib::input input_message;
      input_message.x  = 1 + a;
      input_message.y  = 2;
      input_message.z  = 3;
      input_message.alpha  = 1;
      input_message.beta  = 2;
      input_message.gamma  = 3;
      input_message.d  = 4;
      
      ROS_INFO("%f", input_message.x);
      send_message(chatter_pub, input_message);
      
      ros::spinOnce();
      loop_rate.sleep();
      a++;

     

  }
  
  


  return 0;
}

void send_message(ros::Publisher &chatter_pub, arm_lib::input inp_msg){
  chatter_pub.publish(inp_msg);

    
}