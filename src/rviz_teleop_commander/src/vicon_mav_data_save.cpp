/****************************************************************************
Conversion from a quaternion to roll, pitch and yaw.
Nodes:
subscribed /rotation_quaternion (message of type geometry_msgs::Quaternion)
published /rpy_angles (message oftype geometry_msgs::Vector3.h)
****************************************************************************/
 
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <mav_comm_driver/MAVStatus.h>
#include <string>
#include <fstream>

//#define M_PI 3.14159265
 
// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Publisher rpy_publisher;
ros::Subscriber vicon_subscriber;
ros::Subscriber mav_subscriber;

float vicon_pitch = 0, vicon_roll = 0, vicon_yaw = 0;
float vicon_x = 0, vicon_y = 0, vicon_z = 0;
ros::Time vicon_time;
ulong vicon_count = 0;
bool vicon_received = false;

float mav_pitch = 0, mav_roll = 0, mav_yaw = 0;
float mav_pitch_rate = 0, mav_roll_rate = 0, mav_yaw_rate = 0;
float mav_ext_p = 0, mav_ext_i = 0, mav_ext_d = 0;
float mav_int_p = 0, mav_int_i = 0, mav_int_d = 0;
ros::Time mav_time;
ulong mav_count = 0;
bool mav_received = false;

// Function for conversion of quaternion to roll pitch and yaw. The angles
// are published here too.
void viconCallback(const geometry_msgs::TransformStamped msg){
    
    vicon_count ++;
    vicon_received = true;

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

    vicon_roll = roll;
    vicon_pitch = pitch;
    vicon_yaw = yaw;

    vicon_x = msg.transform.translation.x;
    vicon_y = msg.transform.translation.y;
    vicon_z = msg.transform.translation.z;

    vicon_time = msg.header.stamp;
 
    // this Vector is then published:
    rpy_publisher.publish(rpy);
    // ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);
}

void mavCallback(const mav_comm_driver::MAVStatus::ConstPtr& msg){

    mav_roll = msg -> roll_angle;
    mav_pitch = msg -> pitch_angle;
    mav_yaw = msg -> yaw_angle;
    mav_roll_rate = msg -> roll_rate;
    mav_pitch_rate = msg -> pitch_rate;
    mav_yaw_rate = msg -> yaw_rate;
    mav_time = msg -> header.stamp;
    
    mav_ext_p = msg -> ext_p_output;
    mav_ext_i = msg -> ext_i_output;
    mav_ext_d = msg -> ext_d_output;
    mav_int_p = msg -> int_p_output;
    mav_int_i = msg -> int_i_output;
    mav_int_d = msg -> int_d_output;

    mav_count ++;
    mav_received = true;
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vicon_mav_data_save");
    ros::NodeHandle n;

    std::string vicon_target = "";
    n.param<std::string>("/vicon_target", vicon_target, "vicon/test/test");

    std::string mav_topic = "";
    n.param<std::string>("/mav_topic", mav_topic, "/received_data");

    std::string mav_file_name = "mav.csv";
    n.param<std::string>("/mav_output_file_name", mav_file_name, "mav.csv");

    std::string vicon_file_name = "vicon.csv";
    n.param<std::string>("/vicon_output_file_name", vicon_file_name, "vicon.csv");

    rpy_publisher = n.advertise<geometry_msgs::Vector3>("vicon_rpy_angles", 1000);
    vicon_subscriber = n.subscribe(vicon_target, 1000, viconCallback);
    mav_subscriber = n.subscribe(mav_topic, 10, mavCallback);
 
    // check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("Waiting for data...");

    std::ofstream mav_out(mav_file_name);
    std::ofstream vicon_out(vicon_file_name);
    ROS_INFO_STREAM("Opened " << mav_file_name);
    ROS_INFO_STREAM("Opened " << vicon_file_name);

    ros::Rate loop_rate(400);
    uint mav_lost_count = 0, vicon_lost_count = 0;

    while(ros::ok()){
        ros::spinOnce();
        // ROS_INFO("Waiting for data...");
        if(mav_count && vicon_count){
            if(mav_received){
                mav_out << mav_roll << "," << mav_pitch << "," << mav_yaw << ","
                        << mav_roll_rate << "," << mav_pitch_rate << "," << mav_yaw_rate << ","
                        << mav_ext_p << "," << mav_ext_i << "," << mav_ext_d << ","
                        << mav_int_p << "," << mav_int_i << "," << mav_int_d << ","
                        << mav_time.toNSec() << std::endl;
                mav_received = false;
                mav_lost_count = 0;
            }
            else mav_lost_count ++;
            if(vicon_received){
                vicon_out << vicon_roll << "," << vicon_pitch << "," << vicon_yaw << ","
                          << vicon_x << "," << vicon_y << "," << vicon_z << ","
                          << vicon_time.toNSec() << std::endl;
                vicon_received = false;
                vicon_lost_count = 0;
            }
            else vicon_lost_count ++;
            // if(vicon_lost_count >= 400 || mav_received >= 400){
            //     ROS_INFO("Connection Lost.");
            //     mav_out.close();
            //     vicon_out.close();
            //     return 0;
            // }
        }
        loop_rate.sleep();
    }

    mav_out.close();
    vicon_out.close();
    return 0;
}
