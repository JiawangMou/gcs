#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <string.h>
#include <tf/transform_datatypes.h>

#include <mav_comm_driver/MFPUnified.h>


serial::Serial ros_ser;

#define DEG2RAD (M_PI/180.0)
static const uint8_t send_frame_head[] = {0xaa, 0xaf};

// PID Debug
ros::Publisher int_pid_pub_[3]; //0:yaw 1:pitch 2:roll
ros::Publisher int_pid_pub_sum_;
ros::Publisher ext_pid_pub_[3]; //0:yaw 1:pitch 2:roll
ros::Publisher ext_pid_pub_sum_;

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

    // ROS_INFO("Msg with ID:AAAF0x%02x Sent.", msg -> msg_id);
    return;
}


void SendPIDDebug(vector<uint8_t> &data){

    geometry_msgs::Vector3 pid_debug_msg;
    switch(data[2]){
        case(0x00): // ext(rollp,rolli,rolld,pitchp,pitchi,pitchd)
            //roll
            pid_debug_msg.x = *((float*)&(data[3]));
            pid_debug_msg.y = *((float*)&(data[7]));
            pid_debug_msg.z = *((float*)&(data[11]));
            ext_pid_pub_[2].publish(pid_debug_msg);
            //pitch
            pid_debug_msg.x = *((float*)&(data[15]));
            pid_debug_msg.y = *((float*)&(data[19]));
            pid_debug_msg.z = *((float*)&(data[23]));
            ext_pid_pub_[1].publish(pid_debug_msg);

        break;
        
        case(0x01): // ext(yawp,yawi,yawd)  int(rollp,rolli,rolld)
            //yaw
            pid_debug_msg.x = *((float*)&(data[3]));
            pid_debug_msg.y = *((float*)&(data[7]));
            pid_debug_msg.z = *((float*)&(data[11]));
            ext_pid_pub_[0].publish(pid_debug_msg);
            //roll
            pid_debug_msg.x = *((float*)&(data[15]));
            pid_debug_msg.y = *((float*)&(data[19]));
            pid_debug_msg.z = *((float*)&(data[23]));
            int_pid_pub_[2].publish(pid_debug_msg);
        break;
        
        case(0x02): // int(pitchp,pitchi,pitchd,yawp,yawi,yawd)
            //pitch
            pid_debug_msg.x = *((float*)&(data[3]));
            pid_debug_msg.y = *((float*)&(data[7]));
            pid_debug_msg.z = *((float*)&(data[11]));
            int_pid_pub_[1].publish(pid_debug_msg);
            //yaw
            pid_debug_msg.x = *((float*)&(data[15]));
            pid_debug_msg.y = *((float*)&(data[19]));
            pid_debug_msg.z = *((float*)&(data[23]));
            int_pid_pub_[0].publish(pid_debug_msg);


            // // each sum (x: roll sum, y: pitch sum, z: yaw sum)
            // pid_debug_msg.x = (int16_t)(data[9] << 8 | data[10]);
            // pid_debug_msg.y = (int16_t)(data[17] << 8 | data[18]);
            // pid_debug_msg.z = (int16_t)(data[25] << 8 | data[26]);
            // ext_pid_pub_sum_.publish(pid_debug_msg);

            // // each sum (x: roll sum, y: pitch sum, z: yaw sum)
            // pid_debug_msg.x = (int16_t)(data[9] << 8 | data[10]);
            // pid_debug_msg.y = (int16_t)(data[17] << 8 | data[18]);
            // pid_debug_msg.z = (int16_t)(data[25] << 8 | data[26]);
            // int_pid_pub_sum_.publish(pid_debug_msg);
        break;
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "mav_comm_driver");
    ros::NodeHandle n;

    //接受并转发飞行器config
    ros::Subscriber config_sub = n.subscribe("/mav_download", 500, send_config);

    //发布主题sensor
    ros::Publisher mav_data_pub = n.advertise<mav_comm_driver::MFPUnified>("/received_data", 500);

    //PID Debug
    ext_pid_pub_[0] = n.advertise<geometry_msgs::Vector3>("/pid_ext_yaw", 500);
    ext_pid_pub_[1] = n.advertise<geometry_msgs::Vector3>("/pid_ext_pitch", 500);
    ext_pid_pub_[2] = n.advertise<geometry_msgs::Vector3>("/pid_ext_roll", 500);
    // ext_pid_pub_sum_ = n.advertise<geometry_msgs::Vector3>("/pid_ext_sum", 500);
    int_pid_pub_[0] = n.advertise<geometry_msgs::Vector3>("/pid_int_yaw", 500);
    int_pid_pub_[1] = n.advertise<geometry_msgs::Vector3>("/pid_int_pitch", 500);
    int_pid_pub_[2] = n.advertise<geometry_msgs::Vector3>("/pid_int_roll", 500);
    // int_pid_pub_sum_ = n.advertise<geometry_msgs::Vector3>("/pid_int_sum", 500);

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

                rec_msg.header.stamp = ros::Time::now();

                serial_data.clear();
                ros_ser.read(serial_data, 2);
                ros_ser.read(serial_data, serial_data[1] + 1);

                check_sum = 0;
                for(i = 0 ;i < serial_data.size() - 1; i++){
                    check_sum += serial_data[i];
                    // printf("%02x ",(uint8_t)serial_data[i]);
                }
                // printf("\n");
                check_sum += 0x54;  //0xAA + 0xAA
                if(check_sum != serial_data[serial_data.size() - 1]){
                    ROS_WARN("Bad Check Sum. Dropped.");
                }
                else{
                    serial_data.pop_back();
                    rec_msg.msg_id = serial_data[0];
                    rec_msg.length = serial_data[1];
                    rec_msg.data = serial_data;
                    if(rec_msg.msg_id == mav_comm_driver::MFPUnified::UP_PID_DEBUG)
                        SendPIDDebug(serial_data);
                    mav_data_pub.publish(rec_msg);

                }
            }
        }
        loop_rate.sleep();
    }
}
