#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <mav_comm_driver/MFPUnified.h>
#include <string>
#include <iostream>

#include <Filter.hpp>

#ifdef __cplusplus
extern "C" {
#endif
#include "attitude_pid.h"
#ifdef __cplusplus
}
#endif

#define RAD2DEG (180 / M_PI)

#define VICON_FREQ 300.0
#define CONTROL_FREQ 200.0
#define ANGLE_FILTER_STOP_FREQ 60.0

ros::Publisher to_mav_pub;
static attitude_t actual_angle;
static attitude_t desired_angle;
static attitude_t actual_rate;
static attitude_t desired_rate;
static control_t pid_output;
static Sensor::BiquadFilter angle_roll_filter, angle_pitch_filter, angle_yaw_filter;

void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg -> transform.rotation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);   //In radians
 
    // get vicon angles and apply filter
    actual_angle.pitch = angle_pitch_filter.biquadFilterApply(pitch * RAD2DEG);
    actual_angle.roll = angle_roll_filter.biquadFilterApply(roll * RAD2DEG);
    actual_angle.yaw = angle_yaw_filter.biquadFilterApply(yaw * RAD2DEG);

    return;
}

void mavReceiveCallback(const mav_comm_driver::MFPUnified::ConstPtr& msg){

    if(msg -> msg_id == mav_comm_driver::MFPUnified::UP_SENSER){
        actual_rate.roll = (int16_t)(msg -> data[8] << 8 | msg -> data[9]) / 10;
        actual_rate.pitch = (int16_t)(msg -> data[10] << 8 | msg -> data[11]) / 10;
        actual_rate.yaw = (int16_t)(msg -> data[12] << 8 | msg -> data[13]) / 10;
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vicon_transfer");
    ros::NodeHandle n;

    if(argc < 2) return 0;

    pidInit_t angle_roll, angle_pitch, angle_yaw;
    pidInit_t rate_roll, rate_pitch, rate_yaw;

    //angle PID params
    angle_roll.kp = 8.0; angle_roll.kd = 0.0; angle_roll.ki = 0.0;
    angle_pitch.kp = 8.0; angle_pitch.kd = 0.0; angle_pitch.ki = 0.0;
    angle_yaw.kp = 20.0; angle_yaw.kd = 1.5; angle_yaw.ki = 0.0;

    //rate PID params
    rate_roll.kp = 300.0; rate_roll.kd = 6.5; rate_roll.ki = 0.0;
    rate_pitch.kp = 300.0; rate_pitch.kd = 6.5; rate_pitch.ki = 0.0;
    rate_yaw.kp = 200.0; rate_yaw.kd = 0.0; rate_yaw.ki = 18.5;

    //PID control init
    attitudeControlInit(angle_roll, angle_pitch, angle_yaw,
                        rate_roll, rate_pitch, rate_yaw,
                        1.0 / CONTROL_FREQ, 1.0 / CONTROL_FREQ);
    
    // Angle Filter Init
    angle_roll_filter.biquadFilterInitLPF(ANGLE_FILTER_STOP_FREQ, 1.0 / VICON_FREQ);
    angle_pitch_filter.biquadFilterInitLPF(ANGLE_FILTER_STOP_FREQ, 1.0 / VICON_FREQ);
    angle_yaw_filter.biquadFilterInitLPF(ANGLE_FILTER_STOP_FREQ, 1.0 / VICON_FREQ);
    
    //ROS publisher subscriber init
    to_mav_pub = n.advertise<mav_comm_driver::MFPUnified>("/mav_download", 100);
    ros::Subscriber vicon_sub = n.subscribe(argv[1], 100, viconCallback);
    ros::Subscriber mav_sub = n.subscribe("/received_data", 100, mavReceiveCallback);

    // Maintain level
    desired_angle.pitch = 0.0;
    desired_angle.roll = 0.0;
    desired_angle.yaw = 0.0;
    
    ros::Rate r(200);
    mav_comm_driver::MFPUnified control_msg;
    while(ros::ok()){

        ros::spinOnce();

        //PID cal
        attitudeAnglePID(&actual_angle, &desired_angle, &desired_rate);
        attitudeRatePID(&actual_rate, &desired_rate, &pid_output);

        //publish
        control_msg.msg_id = 0x16;
        control_msg.length = 0;
        control_msg.data.push_back(0x16);
        control_msg.data.push_back(0);

        to_mav_pub.publish(control_msg);
        r.sleep();
    }
    
    return 0;
}
