#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <std_msgs/Bool.h>

serial::Serial ros_ser;

#define DEG2RAD (M_PI/180.0)
#define RAD2DEG (180.0/M_PI)
#define G2MS2 (9.81)
#define q30 1073741824.0f

using namespace std;

ofstream fout("imu_data.txt");

void send_motor(){

    static uint8_t data[4] = {0, 0, 0x0d, 0x0a};
    uint i;
    
    data[1] = 100 - data[1];
    ros_ser.write(data, 4);
        
    for(i = 0 ;i < 2; i++){
        printf("%02x ", data[i]);
    }
    printf("\n");

    return;
}

void motor_callback(const std_msgs::Bool::ConstPtr &msg){
    send_motor();
}

bool process_received_data(string& serial_readings){
    
    //     if(serial_readings.size() != 24 ||
    //    serial_readings[serial_readings.size() - 1] != 0x0a){
    //         ROS_WARN("Bad frame end. Ignored.");
    //         return false;
    // }

    // ros_msg.header.stamp = ros::Time::now();


    // int16_t tmp_16;
    // //Gyro raw values
    // tmp_16 = ((uint8_t)serial_readings[0]) | (((uint8_t)serial_readings[1]) << 8);
    // ros_msg.angular_velocity.x = tmp_16 / 16.384 * DEG2RAD;
    // fout << tmp_16 << ",";
    // tmp_16 = ((uint8_t)serial_readings[2]) | (((uint8_t)serial_readings[3]) << 8);
    // ros_msg.angular_velocity.y = tmp_16 / 16.384 * DEG2RAD;
    // fout << tmp_16 << ",";
    // tmp_16 = ((uint8_t)serial_readings[4]) | (((uint8_t)serial_readings[5]) << 8);
    // ros_msg.angular_velocity.z = tmp_16 / 16.384 * DEG2RAD;
    // fout << tmp_16 << ",";

    // //Acc raw values
    // tmp_16 = ((uint8_t)serial_readings[6]) | (((uint8_t)serial_readings[7]) << 8);
    // ros_msg.linear_acceleration.x = tmp_16 / 16384.0 * G2MS2;
    // fout << tmp_16 << ",";
    // tmp_16 = ((uint8_t)serial_readings[8]) | (((uint8_t)serial_readings[9]) << 8);
    // ros_msg.linear_acceleration.y = tmp_16 / 16384.0 * G2MS2;
    // fout << tmp_16 << ",";
    // tmp_16 = ((uint8_t)serial_readings[10]) | (((uint8_t)serial_readings[11]) << 8);
    // ros_msg.linear_acceleration.z = tmp_16 / 16384.0 * G2MS2;
    // fout << tmp_16 << ",";

    // //Mag raw values
    // tmp_16 = ((uint8_t)serial_readings[12]) | (((uint8_t)serial_readings[13]) << 8);
    // fout << tmp_16 << ",";
    // tmp_16 = ((uint8_t)serial_readings[14]) | (((uint8_t)serial_readings[15]) << 8);
    // fout << tmp_16 << ",";
    // tmp_16 = ((uint8_t)serial_readings[16]) | (((uint8_t)serial_readings[17]) << 8);
    // fout << tmp_16 << ",";

    // //Time in ms
    // uint32_t tmp_32;
    // tmp_32 = ((uint8_t)serial_readings[18]) | (((uint8_t)serial_readings[19]) << 8)
    //         | (((uint8_t)serial_readings[20]) << 16) | (((uint8_t)serial_readings[21]) << 24);
    // fout << tmp_32 << endl;

    // ros_msg.header.stamp = ros::Time::now();

    // return true;

    int i;

    if(serial_readings.size() != 22 ||
       serial_readings[serial_readings.size() - 1] != 0x0a){
            ROS_WARN("Bad frame end. Ignored.");
            for(i = 0 ;i < serial_readings.size(); i++){
                 printf("%02x ",(uint8_t)serial_readings[i]);
             }
             printf("\n");
            return false;
    }


    int16_t tmp_16;
    //Gyro raw values
    tmp_16 = ((uint8_t)serial_readings[0]) | (((uint8_t)serial_readings[1]) << 8);
    fout << tmp_16 << ",";
    tmp_16 = ((uint8_t)serial_readings[2]) | (((uint8_t)serial_readings[3]) << 8);
    fout << tmp_16 << ",";
    tmp_16 = ((uint8_t)serial_readings[4]) | (((uint8_t)serial_readings[5]) << 8);
    fout << tmp_16 << ",";

    //Acc raw values
    tmp_16 = ((uint8_t)serial_readings[6]) | (((uint8_t)serial_readings[7]) << 8);
    fout << tmp_16 << ",";
    tmp_16 = ((uint8_t)serial_readings[8]) | (((uint8_t)serial_readings[9]) << 8);
    fout << tmp_16 << ",";
    tmp_16 = ((uint8_t)serial_readings[10]) | (((uint8_t)serial_readings[11]) << 8);
    fout << tmp_16 << ",";

    //Time in ms
    uint64_t tmp_64;
    tmp_64 = (serial_readings[12] & 0x00000000000000ff) | ((serial_readings[13] & 0x00000000000000ff) << 8)
            | ((serial_readings[14] & 0x00000000000000ff) << 16) | ((serial_readings[15] & 0x00000000000000ff) << 24)
            | (((uint64_t)(serial_readings[16] & 0x00000000000000ff)) << 32) | (((uint64_t)(serial_readings[17] & 0x00000000000000ff)) << 40)
            | (((uint64_t)(serial_readings[18] & 0x00000000000000ff)) << 48) | (((uint64_t)(serial_readings[19] & 0x00000000000000ff)) << 56);
    fout << tmp_64 << endl;

    return true;
}



int main(int argc, char** argv){
     ros::init(argc, argv, "imu_recorder");
     ros::NodeHandle n;

     //发布主题sensor
     ros::Publisher imu_data_pub = n.advertise<sensor_msgs::Imu>("/imu_data", 10);
     ros::Subscriber motor_sub = n.subscribe("/motor_enable", 10, motor_callback);
     
    
     string port = "";
     n.param<std::string>("/mav_driver/port", port, "/dev/ttyUSB0");
     int baudrate = 0;
     n.param<int>("/mav_driver/baudrate", baudrate, 115200);

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
     string serial_data;
     ros::Rate loop_rate(1500);
     int i;
     while(ros::ok()){
         ros::spinOnce();
         if(ros_ser.available()){
             
             //获取串口数据
             serial_data = ros_ser.readline(22, "\r\n");
             if(serial_data.size() != 22){
                 serial_data += ros_ser.readline(22, "\r\n");
             }
            //  for(i = 0 ;i < serial_data.size(); i++){
            //      printf("%02x ",(uint8_t)serial_data[i]);
            //  }
            //  printf("\n");
            //  sensor_msgs::Imu msg;
             if(process_received_data(serial_data/*, msg*/)){
                // imu_data_pub.publish(msg);

             }
         }
         loop_rate.sleep();
     }
     ROS_INFO("Closing File...");
     fout.close();
     return 0;
 }
