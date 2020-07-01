#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <string.h>
#include <tf/transform_datatypes.h>

#include <mav_comm_driver/MFPUnified.h>


serial::Serial ros_ser;

#define DEG2RAD (M_PI/180.0)
static const uint8_t send_frame_head[] = {0xaa, 0xaf};

using namespace std;

// send callback func
void send_config(const mav_comm_driver::MFPUnified::ConstPtr& msg){

    int i;
    uint8_t send_check_sum;
    
    if(msg -> data.size() != msg -> length + 2){
        ROS_WARN("Wrong Msg Length. Ignored");
        return;
    }

    send_check_sum = 0;
    for(i = 0 ;i < msg -> data.size(); i++){
        send_check_sum += msg -> data[i];
        // printf("%02x ",msg -> data[i]);
    }
    send_check_sum += 0x59;     //0xAA + 0xAF
    // printf("%02x ",send_check_sum);
    // printf("\n");

    ros_ser.write(send_frame_head, 2);
    ros_ser.write(msg -> data);
    ros_ser.write(&send_check_sum, 1);

    ROS_INFO_STREAM("Configuration Sent. Size:" << msg -> data.size() + 3);
    return;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "mav_comm_driver");
    ros::NodeHandle n;

    //接受并转发飞行器config
    ros::Subscriber config_sub = n.subscribe("/mav_download", 10, send_config);

    //发布主题sensor
    ros::Publisher mav_data_pub = n.advertise<mav_comm_driver::MFPUnified>("/received_data", 10);

    string port = "";
    n.param<std::string>("/mav_driver/port", port, "/dev/ttyACM0");
    int baudrate = 0;
    n.param<int>("/mav_driver/baudrate", baudrate, 921600);
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
        ROS_ERROR_STREAM(e.what());
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    if(ros_ser.isOpen()){
        ROS_INFO_STREAM("Serial Port opened.");
    }
    else{
        return -1;
    }

    int i;
    ros::Rate loop_rate(500);
    uint8_t check_sum;
    uint8_t head_judge_buffer[10];
    vector<uint8_t> serial_data;
    mav_comm_driver::MFPUnified rec_msg;
        
    while(ros::ok()){

        ros::spinOnce();
        
        ros_ser.waitReadable();
        if(ros_ser.read(head_judge_buffer, 1) && head_judge_buffer[0] == 0xaa){
            if(ros_ser.read(head_judge_buffer, 1) && head_judge_buffer[0] == 0xaa){

                serial_data.clear();
                ros_ser.read(serial_data, 2);
                ros_ser.read(serial_data, serial_data[1] + 1);

                check_sum = 0;
                for(i = 0 ;i < serial_data.size() - 1; i++){
                    check_sum += serial_data[i];
                    printf("%02x ",(uint8_t)serial_data[i]);
                }
                printf("\n");
                check_sum += 0x54;  //0xAA + 0xAA
                if(check_sum != serial_data[serial_data.size() - 1]){
                    ROS_WARN("Bad Check Sum. Dropped.");
                }
                else{
                    serial_data.pop_back();
                    rec_msg.header.stamp = ros::Time::now();
                    rec_msg.msg_id = serial_data[0];
                    rec_msg.length = serial_data[1];
                    rec_msg.data = serial_data;
                    mav_data_pub.publish(rec_msg);
                }
            }
        }
        loop_rate.sleep();
    }
}
