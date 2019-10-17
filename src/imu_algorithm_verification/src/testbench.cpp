#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <string.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>

#include <imu_algorithm_verification/Madgwick.h>


#define DEG2RAD (M_PI/180.0)
#define RAD2DEG (180.0/M_PI)
#define G2MS2 (9.81)
#define q30 1073741824.0f

using namespace std;


int main(int argc, char** argv){

    ros::init(argc, argv, "imu_algo_testbench");
    ros::NodeHandle n;

    //发布测试角度
    ros::Publisher imu_raw_data_pub = n.advertise<sensor_msgs::Imu>("/testbench_raw_output", 10);
    ros::Publisher imu_data_pub = n.advertise<sensor_msgs::Imu>("/testbench_output", 10);
    ros::Publisher acc_data_pub = n.advertise<visualization_msgs::Marker>("/acc_vis", 10);
    tf::TransformBroadcaster tf_pub;

    string data_path;
    n.param<string>("/data_path", data_path, "src/imu_algorithm_verification/data/imu_data_8g_test7(~300Hz, Good).csv");
    
    FILE *fp;
    fp = fopen(data_path.c_str(),"r");

    if(fp == NULL){
        ROS_FATAL("Unable to open file.");
        ROS_FATAL("File path: %s", data_path.c_str());
        return -1;
    }

    // initiate ros msgs
    sensor_msgs::Imu msg;
    sensor_msgs::Imu msg_raw;
    geometry_msgs::Point p1, p2;
    visualization_msgs::Marker marker;
    marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.lifetime = ros::Duration(0.5);
    marker.color.a = 0.5;
    marker.color.r = 1.0; marker.color.g = marker.color.b = 0;
    marker.frame_locked = false;
    marker.scale.x = 0.05;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.points.reserve(2);
    p1.x = p1.y = p1.z = 0;
    p2.x = p2.y = p2.z = 0;
    marker.points.push_back(p1);
    marker.points.push_back(p2);


    ros::Rate loop_rate(800);
    int g_r[3], a_r[3];
    uint time, time_0;
    float g[3], a[3];
    madgwick::IMU_init(&madgwick::estimator);
    bool is_first_data = true;

    while(ros::ok() && !feof(fp)){

        fscanf(fp, "%d,%d,%d,%d,%d,%d,%u", &g_r[0], &g_r[1], &g_r[2], &a_r[0], &a_r[1], &a_r[2], &time);
        g[0] = g_r[0] / 32.768 * DEG2RAD;
        g[1] = g_r[1] / 32.768 * DEG2RAD;
        g[2] = g_r[2] / 32.768 * DEG2RAD;
        a[0] = a_r[0]; a[1] = a_r[1]; a[2] = a_r[2];


        msg_raw.header.stamp = ros::Time::now();
        msg_raw.linear_acceleration.x = a[0] / 4096.0 * 9.8;
        msg_raw.linear_acceleration.y = a[1] / 4096.0 * 9.8;
        msg_raw.linear_acceleration.z = a[2] / 4096.0 * 9.8;
        msg_raw.angular_velocity.x = g[0];
        msg_raw.angular_velocity.y = g[1];
        msg_raw.angular_velocity.z = g[2];
        imu_raw_data_pub.publish(msg_raw);

        //filter
        a[0] = madgwick::biquadFilterApply(&madgwick::accFilter[0], a[0]);
        a[1] = madgwick::biquadFilterApply(&madgwick::accFilter[1], a[1]);
        a[2] = madgwick::biquadFilterApply(&madgwick::accFilter[2], a[2]);
        g[0] = madgwick::biquadFilterApply(&madgwick::gyroFilter[0], g[0]);
        g[1] = madgwick::biquadFilterApply(&madgwick::gyroFilter[1], g[1]);
        g[2] = madgwick::biquadFilterApply(&madgwick::gyroFilter[2], g[2]);

        if(is_first_data){
            is_first_data = false;
            time_0 = time - 1000;
            time = 1000;
        }
        else{
            time -= time_0;
        }
        // printf("%u\n", time);

        
        // demonstrate acc direction
        marker.points[1].x = a[0] / 4096.0;
        marker.points[1].y = a[1] / 4096.0;
        marker.points[1].z = a[2] / 4096.0;
        acc_data_pub.publish(marker);

        // ROS_INFO_STREAM("Time stamp:" << time << ", data: " << g_r[0] << "," << g_r[1] << "," << g_r[2] << ","
        //  << a[0] << "," << a[1] << "," << a[2]);

        madgwick::IMU_update(&madgwick::estimator, g[0], g[1], g[2], a[0], a[1], a[2], 0, 0, 0, time);
        msg.header.stamp = ros::Time::now();
        msg.orientation.w = atan2(2 * madgwick::estimator.q2 * madgwick::estimator.q3 + 2 * madgwick::estimator.q0 * madgwick::estimator.q1,
                         -2 * madgwick::estimator.q1 * madgwick::estimator.q1 - 2 * madgwick::estimator.q2 * madgwick::estimator.q2 + 1) * RAD2DEG;
        msg.linear_acceleration.x = a[0] / 4096.0 * 9.8;
        msg.linear_acceleration.y = a[1] / 4096.0 * 9.8;
        msg.linear_acceleration.z = a[2] / 4096.0 * 9.8;
        msg.angular_velocity.x = g[0];
        msg.angular_velocity.y = g[1];
        msg.angular_velocity.z = g[2];

        imu_data_pub.publish(msg);


        //send tf transform
        tf::Transform transform;
        tf::Quaternion q;
        q.setX(madgwick::estimator.q1);
        q.setY(madgwick::estimator.q2);
        q.setZ(madgwick::estimator.q3);
        q.setW(madgwick::estimator.q0);
        transform.setRotation(q);
        transform.setOrigin(tf::Vector3(0, 0, 0));
        tf_pub.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

        loop_rate.sleep();
    }
    ROS_INFO("Closing File...");
    fclose(fp);

    return 0;
 }
