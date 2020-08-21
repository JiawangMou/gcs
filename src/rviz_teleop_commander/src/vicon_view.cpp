#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <string>
#include <fstream>

ros::Publisher rpy_publisher;
ros::Publisher xyz_publisher;
ros::Subscriber vicon_subscriber;

//file out debug
std::ofstream fout;

void viconCallback(const geometry_msgs::TransformStamped msg){

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.transform.rotation, quat);
 
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
 
    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll * 180.0/M_PI;
    rpy.y = pitch * 180.0/M_PI;
    rpy.z = yaw * 180.0/M_PI;

    geometry_msgs::Vector3 xyz;
    xyz.x = msg.transform.translation.x;
    xyz.y = msg.transform.translation.y;
    xyz.z = msg.transform.translation.z;

    fout << rpy.x << "," << rpy.y << "," << rpy.z << "," << xyz.x << "," << xyz.y << "," << xyz.z << std::endl;

    rpy_publisher.publish(rpy);
    xyz_publisher.publish(xyz);
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vicon_view");
    ros::NodeHandle n;

    if(argc < 2){
        ROS_ERROR("No Topic Specified. Aborted.");
        return 0;
    }

    fout.open("vicon_data_save.txt");

    rpy_publisher = n.advertise<geometry_msgs::Vector3>("vicon_rpy_angles", 1000);
    xyz_publisher = n.advertise<geometry_msgs::Vector3>("vicon_position", 1000);
    vicon_subscriber = n.subscribe(argv[1], 1000, viconCallback);
 
    // check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("Waiting for data...");
    ros::spin();

    fout.close();

    return 0;
}
