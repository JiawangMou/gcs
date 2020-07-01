#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <mav_comm_driver/MFPUnified.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  
  ros::Publisher pub_test = n.advertise<mav_comm_driver::MFPUnified>("/mav_download", 10);
  
  ros::Rate looprate(10);
  mav_comm_driver::MFPUnified msg;
  msg.msg_id = 0x16;
  msg.length = 1;
  msg.data.push_back(0x16);
  msg.data.push_back(0x01);
  msg.data.push_back(0x00);

  while(ros::ok()){
    msg.header.stamp = ros::Time::now();
    pub_test.publish(msg);
    ROS_INFO_STREAM("OK!");
    looprate.sleep();
  }

  return 0;
}
