#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <string.h>
#include <tf/transform_datatypes.h>

#include <mav_comm_driver/ModeConfig.h>
#include <mav_comm_driver/MAVStatus.h>

serial::Serial ros_ser;

#define DEG2RAD (M_PI/180.0)

using namespace std;

// send callback func
void send_config(const mav_comm_driver::ModeConfig::ConstPtr& msg){

    int i;
    switch(msg -> mode_id & 0x38){
        case mav_comm_driver::ModeConfig::START_MODE:
        case mav_comm_driver::ModeConfig::MANUAL_MODE:
        case mav_comm_driver::ModeConfig::FLIGHT_MODE:
        case mav_comm_driver::ModeConfig::TUNING_MODE:
        case mav_comm_driver::ModeConfig::FAULT_MODE:
            if(msg -> data.size() > 0 && msg -> data[0] == msg -> mode_id){
                ros_ser.write(msg->data);
                ROS_WARN("%02x", msg ->data[0]);
                for(i = 0 ;i < msg -> data.size(); i++){
                 printf("%02x ",msg -> data[i]);
             }
             printf("\n");
                ROS_INFO_STREAM("Configuration Sent. Size:" << msg ->data.size());
            }
            else
            {
                ROS_WARN("Mode ID is not attached to the message head! Ignored.");
            }
            
        break;
        
        default:
            ROS_WARN("Mode ID not exist.");

    }

}

bool process_received_data(string& serial_readings, mav_comm_driver::MAVStatus& ros_msg,
                            std_msgs::Float32& ext_meas_angle, std_msgs::Float32& ext_meas_rate){

    if(serial_readings[serial_readings.size() - 1] != 0x0a){
        ROS_WARN("Bad frame end. Ignored.");
        return false;
    }

    ros_msg.mode_id = serial_readings[0];
    if(ros_msg.mode_id != mav_comm_driver::MAVStatus::FAULT_MODE &&
       ros_msg.mode_id != mav_comm_driver::MAVStatus::START_MODE &&
       ros_msg.mode_id != mav_comm_driver::MAVStatus::MANUAL_MODE &&
       ros_msg.mode_id != mav_comm_driver::MAVStatus::FLIGHT_MODE &&
       ros_msg.mode_id != mav_comm_driver::MAVStatus::TUNING_MODE){
            ROS_WARN("Bad frame ID. Ignored.");
            return false;
       }
    
    if((ros_msg.mode_id == mav_comm_driver::MAVStatus::FAULT_MODE && serial_readings.size() != 27) ||
       (ros_msg.mode_id == mav_comm_driver::MAVStatus::START_MODE && serial_readings.size() != 29) ||
       (ros_msg.mode_id == mav_comm_driver::MAVStatus::MANUAL_MODE && serial_readings.size() != 29) ||
       (ros_msg.mode_id == mav_comm_driver::MAVStatus::FLIGHT_MODE && serial_readings.size() != 29) ||
       (ros_msg.mode_id == mav_comm_driver::MAVStatus::TUNING_MODE && serial_readings.size() != 37)){
            ROS_WARN("Frame length wrong. Ignored.");
            return false;
       }

    int16_t tmp;
    uint32_t tmp_32;

    if(ros_msg.mode_id == mav_comm_driver::MAVStatus::TUNING_MODE){

        ros_msg.pid_id = (uint8_t)serial_readings[26];
        switch(ros_msg.pid_id){
            case(0):    //yaw
                tmp = (uint8_t)serial_readings[1] | (((uint8_t)serial_readings[2]) << 8);
                ros_msg.yaw_angle = tmp / 100.0;
                tmp = (uint8_t)serial_readings[3] | (((uint8_t)serial_readings[4]) << 8);
                ros_msg.yaw_rate = tmp;
                break;
            case(1):    //pitch
                tmp = (uint8_t)serial_readings[1] | (((uint8_t)serial_readings[2]) << 8);
                ros_msg.pitch_angle = tmp / 100.0;
                tmp = (uint8_t)serial_readings[3] | (((uint8_t)serial_readings[4]) << 8);
                ros_msg.pitch_rate = tmp;
                break;
            case(2):    //roll
                tmp = (uint8_t)serial_readings[1] | (((uint8_t)serial_readings[2]) << 8);
                ros_msg.roll_angle = tmp / 100.0;
                tmp = (uint8_t)serial_readings[3] | (((uint8_t)serial_readings[4]) << 8);
                ros_msg.roll_rate = tmp;
                break;
        }
        //time
        tmp_32 = ((uint8_t)serial_readings[5]) | (((uint8_t)serial_readings[6]) << 8)
            | (((uint8_t)serial_readings[7]) << 16) | (((uint8_t)serial_readings[8]) << 24);
        ros_msg.board_time = tmp_32;

        ros_msg.header.stamp = ros::Time::now();

        //system status
        ros_msg.sys_status = (uint8_t)serial_readings[9];

        //pid values
        tmp = (uint8_t)serial_readings[10] | (((uint8_t)serial_readings[11]) << 8);
        ros_msg.pid_ext_err = tmp / 100.0;
        tmp = (uint8_t)serial_readings[12] | (((uint8_t)serial_readings[13]) << 8);
        ros_msg.pid_int_err = tmp / 10.0;

        tmp = (uint8_t)serial_readings[14] | (((uint8_t)serial_readings[15]) << 8);
        ros_msg.ext_p_output = tmp / 100.0;
        tmp = (uint8_t)serial_readings[16] | (((uint8_t)serial_readings[17]) << 8);
        ros_msg.ext_i_output = tmp / 100.0;
        tmp = (uint8_t)serial_readings[18] | (((uint8_t)serial_readings[19]) << 8);
        ros_msg.ext_d_output = tmp / 100.0;
        tmp = (uint8_t)serial_readings[20] | (((uint8_t)serial_readings[21]) << 8);
        ros_msg.int_p_output = tmp / 100.0;
        tmp = (uint8_t)serial_readings[22] | (((uint8_t)serial_readings[23]) << 8);
        ros_msg.int_i_output = tmp / 100.0;
        tmp = (uint8_t)serial_readings[24] | (((uint8_t)serial_readings[25]) << 8);
        ros_msg.int_d_output = tmp / 100.0;

        ros_msg.right_servo_pwm = (uint8_t) serial_readings[27];
        ros_msg.left_servo_pwm = (uint8_t) serial_readings[28];
        ros_msg.mid_servo_pwm = (uint8_t) serial_readings[29];
        ros_msg.throttle_pwm = (uint8_t) serial_readings[30];

        tmp = (uint8_t)serial_readings[31] | (((uint8_t)serial_readings[32]) << 8);
        ext_meas_angle.data = tmp / 100.0;
        tmp = (uint8_t)serial_readings[33] | (((uint8_t)serial_readings[34]) << 8);
        ext_meas_rate.data = tmp / 10.0;

    }
    else{

        //angular sensor values
        tmp = (uint8_t)serial_readings[1] | (((uint8_t)serial_readings[2]) << 8);
        ros_msg.yaw_angle = tmp / 100.0;
        tmp = (uint8_t)serial_readings[3] | (((uint8_t)serial_readings[4]) << 8);
        ros_msg.pitch_angle = tmp / 100.0;
        tmp = (uint8_t)serial_readings[5] | (((uint8_t)serial_readings[6]) << 8);
        ros_msg.roll_angle = tmp / 100.0;
        tmp = (uint8_t)serial_readings[7] | (((uint8_t)serial_readings[8]) << 8);
        ros_msg.yaw_rate = tmp;
        tmp = (uint8_t)serial_readings[9] | (((uint8_t)serial_readings[10]) << 8);
        ros_msg.pitch_rate = tmp;
        tmp = (uint8_t)serial_readings[11] | (((uint8_t)serial_readings[12]) << 8);
        ros_msg.roll_rate = tmp;

        //time
        tmp_32 = ((uint8_t)serial_readings[13]) | (((uint8_t)serial_readings[14]) << 8)
            | (((uint8_t)serial_readings[15]) << 16) | (((uint8_t)serial_readings[16]) << 24);
        ros_msg.board_time = tmp_32;

        ros_msg.header.stamp = ros::Time::now();

        //system status
        ros_msg.sys_status = (uint8_t)serial_readings[17];

        //pwm values
        ros_msg.right_servo_pwm = (uint8_t) serial_readings[18];
        ros_msg.left_servo_pwm = (uint8_t) serial_readings[19];
        ros_msg.mid_servo_pwm = (uint8_t) serial_readings[20];

        if(ros_msg.mode_id != mav_comm_driver::MAVStatus::FAULT_MODE){
            ros_msg.throttle_pwm = (uint8_t) serial_readings[21];
            ros_msg.climb_pwm = (uint8_t) serial_readings[22];

            tmp = (uint8_t)serial_readings[23] | (((uint8_t)serial_readings[24]) << 8);
            ext_meas_angle.data = tmp / 100.0;
            tmp = (uint8_t)serial_readings[25] | (((uint8_t)serial_readings[26]) << 8);
            ext_meas_rate.data = tmp / 10.0;
        }
        else{
            tmp = (uint8_t)serial_readings[21] | (((uint8_t)serial_readings[22]) << 8);
            ext_meas_angle.data = tmp / 100.0;
            tmp = (uint8_t)serial_readings[23] | (((uint8_t)serial_readings[24]) << 8);
            ext_meas_rate.data = tmp / 10.0;
        }
    
    }

    tf::Quaternion q;
    q.setRPY(ros_msg.roll_angle * DEG2RAD, ros_msg.pitch_angle * DEG2RAD, ros_msg.yaw_angle * DEG2RAD);
    ros_msg.odom.pose.pose.orientation.w = q.getW();
    ros_msg.odom.pose.pose.orientation.x = q.getX();
    ros_msg.odom.pose.pose.orientation.y = q.getY();
    ros_msg.odom.pose.pose.orientation.z = q.getZ();


    return true;
    
}



int main(int argc, char** argv){
     ros::init(argc, argv, "mav_comm_driver");
     ros::NodeHandle n;

     //接受并转发飞行器config
     ros::Subscriber config_sub = n.subscribe("/mode_config", 10, send_config);

     //发布主题sensor
     ros::Publisher mav_data_pub = n.advertise<mav_comm_driver::MAVStatus>("/received_data", 10);

     //发布外部测量数据
     ros::Publisher ext_measure_angle_pub = n.advertise<std_msgs::Float32>("/ext_angle", 10);
     ros::Publisher ext_measure_rate_pub = n.advertise<std_msgs::Float32>("/ext_rate", 10);
    
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
     string serial_data;
     
     while(ros::ok()){
         ros::spinOnce();
         if(ros_ser.available()){
            //  ROS_INFO_STREAM("Reading from serial port");
             
             //获取串口数据
             serial_data = ros_ser.readline(37, "\r\n");
            //  for(i = 0 ;i < serial_data.size(); i++){
            //      printf("%02x ",(uint8_t)serial_data[i]);
            //  }
            //  printf("\n");
             mav_comm_driver::MAVStatus msg;
             std_msgs::Float32 ext_angle, ext_rate;
             if(process_received_data(serial_data, msg, ext_angle, ext_rate)){
                mav_data_pub.publish(msg);
                ext_measure_angle_pub.publish(ext_angle);
                ext_measure_rate_pub.publish(ext_rate);
                // ROS_INFO("Success!");
             }
                
         }
         loop_rate.sleep();
     }
 }
