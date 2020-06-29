#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <mav_comm_driver/ModeConfig.h>
#include <string>
#include <iostream>

ros::Publisher to_mav_pub;

void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    
    tf::quaternionMsgToTF(msg -> transform.rotation, quat);
 
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
 
    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll * 180.0/M_PI;
    rpy.y = pitch * 180.0/M_PI;
    rpy.z = yaw * 180.0/M_PI;

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vicon_transfer");
    ros::NodeHandle n;

    if(argc < 2) return 0;

    to_mav_pub = n.advertise<mav_comm_driver::ModeConfig>("/mode_config", 100);
    ros::Subscriber vicon_subscriber = n.subscribe(argv[1], 100, viconCallback);
    
    ros::spin();
    return 0;
}
