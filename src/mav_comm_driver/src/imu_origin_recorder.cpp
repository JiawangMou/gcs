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

bool process_received_data(vector<uint8_t> &serial_readings){

    if(serial_readings[0] != 0x02) return false;

    int i;

    int16_t tmp_16;
    //Acc raw values
    tmp_16 = (serial_readings[3]) | (serial_readings[2] << 8);
    fout << tmp_16 << ",";
    tmp_16 = (serial_readings[5]) | (serial_readings[4] << 8);
    fout << tmp_16 << ",";
    tmp_16 = (serial_readings[7]) | (serial_readings[6] << 8);
    fout << tmp_16 << ",";

    //Gyro raw values
    tmp_16 = (serial_readings[9]) | (serial_readings[8] << 8);
    fout << tmp_16 << ",";
    tmp_16 = (serial_readings[11]) | (serial_readings[10] << 8);
    fout << tmp_16 << ",";
    tmp_16 = (serial_readings[13]) | (serial_readings[12] << 8);
    fout << tmp_16 << ",";

    //Mag raw values
    tmp_16 = (serial_readings[15]) | ((serial_readings[14]) << 8);
    fout << tmp_16 << ",";
    tmp_16 = (serial_readings[17]) | ((serial_readings[16]) << 8);
    fout << tmp_16 << ",";
    tmp_16 = (serial_readings[19]) | ((serial_readings[18]) << 8);
    fout << tmp_16 << endl;

    //temperature
    // tmp_16 = ((uint8_t)serial_readings[12]) | (((uint8_t)serial_readings[13]) << 8);
    // fout << tmp_16 << ",";

    //Time in ms
    // uint64_t tmp_64;
    // tmp_64 = (serial_readings[14] & 0x00000000000000ff) | ((serial_readings[15] & 0x00000000000000ff) << 8)
    //         | ((serial_readings[16] & 0x00000000000000ff) << 16) | ((serial_readings[17] & 0x00000000000000ff) << 24)
    //         | (((uint64_t)(serial_readings[18] & 0x00000000000000ff)) << 32) | (((uint64_t)(serial_readings[19] & 0x00000000000000ff)) << 40)
    //         | (((uint64_t)(serial_readings[20] & 0x00000000000000ff)) << 48) | (((uint64_t)(serial_readings[21] & 0x00000000000000ff)) << 56);
    // fout << tmp_64 << endl;

    return true;
}



int main(int argc, char** argv){

    ros::init(argc, argv, "imu_recorder");
    ros::NodeHandle n;

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
                    // printf("%02x ",(uint8_t)serial_data[i]);
                }
                // printf("\n");
                check_sum += 0x54;  //0xAA + 0xAA
                if(check_sum != serial_data[serial_data.size() - 1]){
                    ROS_WARN("Bad Check Sum. Dropped.");
                }
                else{
                    serial_data.pop_back();
                    process_received_data(serial_data);
                }
            }
        }
        loop_rate.sleep();
    }

     ROS_INFO("Closing File...");
     fout.close();
     return 0;
 }
