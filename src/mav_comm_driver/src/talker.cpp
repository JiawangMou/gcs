#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  //发布主题command
  uint8_t a;
  a = 0xff;
  #ifdef TWO_WING
  printf("%02x", a);
  #endif
  return 0;
}
