#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <mav_comm_driver/MFPUnified.h>
#include <string>
#include <iostream>

#ifdef __cplusplus
extern "C" {
#endif
#include "attitude_pid.h"
#ifdef __cplusplus
}
#endif

#define VICON_FREQ 100.0

ros::Publisher to_mav_pub;
static attitude_t actual_angle;
static attitude_t desired_angle;
static attitude_t actual_rate;
static attitude_t desired_rate;
static control_t pid_output;

void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg -> transform.rotation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);   //In radians
 
    // get actual angles
    //TODO: radian 2 degree?
    actual_angle.pitch = pitch;
    actual_angle.roll = roll;
    actual_angle.yaw = yaw;

    // TODO: get actual rates


    //PID cal
    attitudeAnglePID(&actual_angle, &desired_angle, &desired_rate);
    attitudeRatePID(&actual_rate, &desired_rate, &pid_output);

    //publish
    
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vicon_transfer");
    ros::NodeHandle n;

    if(argc < 2) return 0;

    pidInit_t angle_roll, angle_pitch, angle_yaw;
    pidInit_t rate_roll, rate_pitch, rate_yaw;

    //angle PID
    angle_roll.kp = 0.0; angle_roll.kd = 0.0; angle_roll.ki = 0.0;
    angle_pitch.kp = 0.0; angle_pitch.kd = 0.0; angle_pitch.ki = 0.0;
    angle_yaw.kp = 0.0; angle_yaw.kd = 0.0; angle_yaw.ki = 0.0;

    //rate PID
    rate_roll.kp = 0.0; rate_roll.kd = 0.0; rate_roll.ki = 0.0;
    rate_pitch.kp = 0.0; rate_pitch.kd = 0.0; rate_pitch.ki = 0.0;
    rate_yaw.kp = 0.0; rate_yaw.kd = 0.0; rate_yaw.ki = 0.0;

    attitudeControlInit(angle_roll, angle_pitch, angle_yaw,
                        rate_roll, rate_pitch, rate_yaw,
                        1.0 / VICON_FREQ, 1.0 / VICON_FREQ);
    

    to_mav_pub = n.advertise<mav_comm_driver::MFPUnified>("/mode_config", 100);
    ros::Subscriber vicon_subscriber = n.subscribe(argv[1], 100, viconCallback);
    
    ros::spin();
    return 0;
}
