#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <mav_comm_driver/MFPUnified.h>
#include <queue>

using namespace std;

queue<uint64_t> last_sent_stamp;
queue<uint8_t> last_sent_check_sum;
double time_elapse = 0.0;
uint rec_count = 0;

void callback(const mav_comm_driver::MFPUnified::ConstPtr& msg){

  if(msg -> msg_id == mav_comm_driver::MFPUnified::UP_CHECK && msg -> data[2] == 0x16){
    
    while(!last_sent_check_sum.empty() && msg -> data[3] != last_sent_check_sum.front()){
      last_sent_check_sum.pop();
      last_sent_stamp.pop();
    }
    
    if(!last_sent_check_sum.empty()){
      time_elapse += (msg -> header.stamp.toNSec() - last_sent_stamp.front()) / 1000000.0;
      rec_count ++;
      last_sent_check_sum.pop();
      last_sent_stamp.pop();
      if(rec_count == 100){
        ROS_INFO_STREAM("AVG Receive Delay is " << time_elapse / rec_count << " ms");
        rec_count = 0;
        time_elapse = 0.0;
      }
    }

  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  
  ros::Publisher pub_test = n.advertise<mav_comm_driver::MFPUnified>("/mav_download", 10);
  ros::Subscriber mav_sub = n.subscribe("/received_data", 10, callback);

  ros::AsyncSpinner aspin(4);
  aspin.start();

  ros::Rate looprate(100);
  mav_comm_driver::MFPUnified msg;
  msg.msg_id = 0x16;
  msg.length = 6;
  uint16_t tmp_count = 0;
  uint8_t check_sum;

  while(ros::ok()){
    check_sum = 0;
    tmp_count ++;
    msg.data.clear();
    msg.data.push_back(0x16);
    msg.data.push_back(0x06);
    msg.data.push_back(tmp_count >> 8);
    msg.data.push_back(tmp_count);
    msg.data.push_back(0x00);
    msg.data.push_back(0x00);
    msg.data.push_back(0x00);
    msg.data.push_back(0x00);
    msg.header.stamp = ros::Time::now();
    pub_test.publish(msg);

    last_sent_stamp.push(msg.header.stamp.toNSec());
    check_sum = msg.data[0] + msg.data[1] + msg.data[2] + msg.data[3] + 0x59;
    last_sent_check_sum.push(check_sum);
    looprate.sleep();
  }

  aspin.stop();
  return 0;
}
