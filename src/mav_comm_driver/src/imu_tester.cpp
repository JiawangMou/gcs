#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string.h>
#include <tf/transform_datatypes.h>

#include <mav_comm_driver/MAVStatus.h>

serial::Serial ros_ser;

#define DEG2RAD (M_PI/180.0)
#define RAD2DEG (180.0/M_PI)
#define q30 1073741824.0f

using namespace std;

bool process_received_data(string& serial_readings, mav_comm_driver::MAVStatus& ros_msg){


    //----------------DMP Test ---------------//

    // if(serial_readings.size() != 18 ||
    //    serial_readings[serial_readings.size() - 1] != 0x0a){
    //         ROS_WARN("Bad frame end. Ignored.");
    //         return false;
    // }

    // ros_msg.mode_id = 0x00;

    // int32_t tmp_32;

    // tmp_32 = ((uint8_t)serial_readings[0]) | (((uint8_t)serial_readings[1]) << 8)
    //     | (((uint8_t)serial_readings[2]) << 16) | (((uint8_t)serial_readings[3]) << 24);
    // ros_msg.odom.header.stamp = ros::Time::now();
    // ros_msg.odom.pose.pose.orientation.w = tmp_32 / q30;

    // tmp_32 = ((uint8_t)serial_readings[4]) | (((uint8_t)serial_readings[5]) << 8)
    //     | (((uint8_t)serial_readings[6]) << 16) | (((uint8_t)serial_readings[7]) << 24);
    // ros_msg.odom.header.stamp = ros::Time::now();
    // ros_msg.odom.pose.pose.orientation.x = tmp_32 / q30;

    // tmp_32 = ((uint8_t)serial_readings[8]) | (((uint8_t)serial_readings[9]) << 8)
    //     | (((uint8_t)serial_readings[10]) << 16) | (((uint8_t)serial_readings[11]) << 24);
    // ros_msg.odom.header.stamp = ros::Time::now();
    // ros_msg.odom.pose.pose.orientation.y = tmp_32 / q30;

    // tmp_32 = ((uint8_t)serial_readings[12]) | (((uint8_t)serial_readings[13]) << 8)
    //     | (((uint8_t)serial_readings[14]) << 16) | (((uint8_t)serial_readings[15]) << 24);
    // ros_msg.odom.header.stamp = ros::Time::now();
    // ros_msg.odom.pose.pose.orientation.z = tmp_32 / q30;

    // double q0 = ros_msg.odom.pose.pose.orientation.w;
    // double q1 = ros_msg.odom.pose.pose.orientation.x;
    // double q2 = ros_msg.odom.pose.pose.orientation.y;
    // double q3 = ros_msg.odom.pose.pose.orientation.z;

    // ros_msg.yaw_angle = atan2(2*(q1*q2 - q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * RAD2DEG;
    // ros_msg.pitch_angle = asin(-2 * (q0*q2 + q1*q3)) * RAD2DEG;
    // ros_msg.roll_angle = atan2(2*(q2*q3 - q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3) * RAD2DEG;

    // ros_msg.header.stamp = ros::Time::now();

    // return true;



    //----------------Kalman Test ---------------//

    if(serial_readings.size() != 6 ||
       serial_readings[serial_readings.size() - 1] != 0x0a){
            ROS_WARN("Bad frame end. Ignored.");
            return false;
    }

    ros_msg.mode_id = 0x00;
    ros_msg.odom.header.stamp = ros::Time::now();

    int16_t tmp_16;
    tmp_16 = ((uint8_t)serial_readings[0]) | (((uint8_t)serial_readings[1]) << 8);
    ros_msg.pitch_angle = tmp_16 / 100.0;
    tmp_16 = ((uint8_t)serial_readings[2]) | (((uint8_t)serial_readings[3]) << 8);
    ros_msg.roll_angle = tmp_16 / 100.0;
    ros_msg.yaw_angle = 0;

    tf::Quaternion q;
    q.setRPY(ros_msg.roll_angle * DEG2RAD, ros_msg.pitch_angle * DEG2RAD, 0.0);
    ros_msg.odom.pose.pose.orientation.w = q.getW();
    ros_msg.odom.pose.pose.orientation.x = q.getX();
    ros_msg.odom.pose.pose.orientation.y = q.getY();
    ros_msg.odom.pose.pose.orientation.z = q.getZ();

    ros_msg.header.stamp = ros::Time::now();

    return true;
}



int main(int argc, char** argv){
     ros::init(argc, argv, "imu_tester");
     ros::NodeHandle n;

     //发布主题sensor
     ros::Publisher imu_data_pub = n.advertise<mav_comm_driver::MAVStatus>("/received_data", 10);
     
    
     string port = "";
     n.param<std::string>("/mav_driver/port", port, "/dev/ttyUSB0");
     int baudrate = 0;
     n.param<int>("/mav_driver/baudrate", baudrate, 115200);
     //TODO: ROS parameter

     try
     {
         ros_ser.setPort(port);
         ros_ser.setBaudrate(baudrate);
        //  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial::Timeout to(1, 10, 0 ,10, 0);
         ros_ser.setTimeout(to);
         ros_ser.open();
     }
     catch (serial::IOException& e)
     {
         ROS_ERROR_STREAM("Unable to open port.");
         return -1;
     }

     if(ros_ser.isOpen()){
         ROS_INFO_STREAM("Serial Port opened.");
     }
     else{
         return -1;
     }

     ros::Rate loop_rate(200);
     int i;
     while(ros::ok()){
         ros::spinOnce();
         if(ros_ser.available()){
             string serial_data;
             //获取串口数据
             serial_data = ros_ser.readline(34, "\r\n");
             for(i = 0 ;i < serial_data.size(); i++){
                 printf("%02x ",(uint8_t)serial_data[i]);
             }
             printf("\n");
             mav_comm_driver::MAVStatus msg;
             if(process_received_data(serial_data, msg)){
                imu_data_pub.publish(msg);
             }
         }
         loop_rate.sleep();
     }
 }
