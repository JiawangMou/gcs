#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <mav_comm_driver/MFPUnified.h>
#include <std_msgs/Int32.h>
#include <queue>

using namespace std;

queue<uint64_t> last_sent_stamp;
queue<uint8_t> last_sent_check_sum;
double time_elapse = 0.0;
uint rec_count = 0;
std_msgs::Int32 height;
ros::Publisher height_pub;

void callback(const mav_comm_driver::MFPUnified::ConstPtr& msg){

  if(msg -> msg_id == mav_comm_driver::MFPUnified::UP_STATUS){
    
    height.data = (int32_t)((uint32_t)msg -> data[8] << 24 | (uint32_t)msg -> data[9] << 16 | (uint32_t)msg -> data[10] << 8 | (uint32_t)msg -> data[11]);
    height_pub.publish(height);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  
  ros::Publisher pub_test = n.advertise<mav_comm_driver::MFPUnified>("/mav_download", 10);
  height_pub = n.advertise<std_msgs::Int32>("/height", 10);
  ros::Subscriber mav_sub = n.subscribe("/received_data", 10, callback);

  ros::AsyncSpinner aspin(4);
  aspin.start();

  ros::Rate looprate(0.5);
  mav_comm_driver::MFPUnified msg;
  msg.msg_id = 0x16;
  msg.length = 6;
  uint16_t tmp_count = 0;
  uint8_t check_sum;

  while(ros::ok()){
    // check_sum = 0;
    // tmp_count ++;
    // msg.data.clear();
    // msg.data.push_back(0x16);
    // msg.data.push_back(0x06);
    // msg.data.push_back(tmp_count >> 8);
    // msg.data.push_back(tmp_count);
    // msg.data.push_back(0x00);
    // msg.data.push_back(0x00);
    // msg.data.push_back(0x00);
    // msg.data.push_back(0x00);
    // msg.header.stamp = ros::Time::now();
    // pub_test.publish(msg);

    // last_sent_stamp.push(msg.header.stamp.toNSec());
    // check_sum = msg.data[0] + msg.data[1] + msg.data[2] + msg.data[3] + 0x59;
    // last_sent_check_sum.push(check_sum);
    looprate.sleep();
  }

  aspin.stop();
  return 0;
}
