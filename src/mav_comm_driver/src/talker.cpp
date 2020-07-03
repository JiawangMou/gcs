#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <mav_comm_driver/MFPUnified.h>

using namespace std;

void callback(const mav_comm_driver::MFPUnified::ConstPtr& msg){
  ROS_INFO("HelloWOrld");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  
  ros::Publisher pub_test = n.advertise<mav_comm_driver::MFPUnified>("/mav_download", 10);
  ros::Subscriber mav_sub = n.subscribe("/received_data", 10, callback);
  
  ros::Rate looprate(200);
  mav_comm_driver::MFPUnified msg;
  msg.msg_id = 0x16;
  msg.length = 6;
  msg.data.push_back(0x16);
  msg.data.push_back(0x06);
  msg.data.push_back(0x00);
  msg.data.push_back(0x00);
  msg.data.push_back(0x00);
  msg.data.push_back(0x00);
  msg.data.push_back(0x00);
  msg.data.push_back(0x00);

  while(ros::ok()){
    msg.header.stamp = ros::Time::now();
    pub_test.publish(msg);
    ros::spinOnce();
    looprate.sleep();
  }

  return 0;
}
