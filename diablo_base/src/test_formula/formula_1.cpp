#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <string>

#include "diablo_sdk/OSDK_ACCL.h"
#include "diablo_sdk/OSDK_GYRO.h"
#include "diablo_sdk/OSDK_LEGMOTORS.h"
#include "diablo_sdk/OSDK_POWER.h"
#include "diablo_sdk/OSDK_QUATERNION.h"
#include "diablo_sdk/OSDK_RC.h"
#include "diablo_sdk/OSDK_STATUS.h"

#include "Onboard_SDK_Uart_Protocol.h"
#include "OSDK_Vehicle.hpp"

#include "diablo_base/angles.h"
#include "diablo_base/disps.h"
#include "diablo_base/coors.h"

#define _USE_MATH_DEFINES
#include <cmath>

float vals[] = {
    0,0,0,
    0,0,0,0,
    0,0,0,
    0,0,0};

float right_pos, left_pos, right_enc, left_enc, right_init, right_offset, left_init, left_offset;

float xcor = 0;
float ycor = 0;
float cent_prev = 0;

int init_values = 0;
int init_quat_values = 0;
float init_yaw = 0;

void gyro_callback(const diablo_sdk::OSDK_GYRO::ConstPtr& information){
    vals[10] = information->x;
    vals[11] = information->y;
    vals[12] = information->z;
}

void quat_callback(const diablo_sdk::OSDK_QUATERNION::ConstPtr& information){
    vals[3] = information->x;
    vals[4] = information->y;
    vals[5] = information->z;
    vals[6] = information->w;
    if(init_quat_values<2){
        double siny_cosp = 2 * (vals[6] * vals[5] + vals[3] * vals[4]);
        double cosy_cosp = 1 - 2 * (vals[4] * vals[4] + vals[5] * vals[5]);
        init_yaw = std::atan2(siny_cosp, cosy_cosp);
        init_quat_values++;
    }
}

void motor_callback(const diablo_sdk::OSDK_LEGMOTORS::ConstPtr& information){
    right_pos = information->right_wheel_pos;
    left_pos = information->left_wheel_pos;
    right_enc = information->right_wheel_enc_rev;
    left_enc = information->left_wheel_enc_rev;
    if(init_values < 2){
        right_init = information->right_wheel_enc_rev;
        left_init = information->left_wheel_enc_rev;
        right_offset = information->right_wheel_pos;
        left_offset = information->left_wheel_pos;
        init_values++;
    }

}

int quat_off = 0;
float x_old = 0;
float y_old = 0;

int main(int argc, char** argv){
    ros::init(argc, argv, "status_node");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("/all_data", 10);
    ros::Publisher pub2 = n.advertise<diablo_base::angles>("/angles", 10);
    ros::Publisher pub3 = n.advertise<diablo_base::disps>("/displacement", 10);
    ros::Publisher pub4 = n.advertise<diablo_base::coors>("/coordinates", 10);

    ros::Subscriber sub_gyr = n.subscribe("/robot_status_example/diablo_ros_GYRO_b", 10, gyro_callback);
    ros::Subscriber sub_quat = n.subscribe("/robot_status_example/diablo_ros_QUATERNION_b", 10, quat_callback);
    ros::Subscriber sub_motor = n.subscribe("/robot_status_example/diablo_ros_LEGMOTORS_b", 10, motor_callback);

    ros::Rate loop_rate(100);
    while(ros::ok()){

        diablo_base::angles ang;
        double sinr_cosp = 2 * (vals[6]* vals[3] + vals[4] * vals[5]);
        double cosr_cosp = 1 - 2 * (vals[3] * vals[3] + vals[4] * vals[4]);
        ang.roll = std::atan2(sinr_cosp, cosr_cosp);
        double sinp = std::sqrt(1 + 2 * (vals[6] * vals[4] - vals[3] * vals[5]));
        double cosp = std::sqrt(1 - 2 * (vals[6] * vals[4] - vals[3] * vals[5]));
        ang.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;
        double siny_cosp = 2 * (vals[6] * vals[5] + vals[3] * vals[4]);
        double cosy_cosp = 1 - 2 * (vals[4] * vals[4] + vals[5] * vals[5]);
        float yaw = std::atan2(siny_cosp, cosy_cosp);
        yaw = yaw - 0.0000286714*quat_off;
        quat_off++;
        ang.yaw = yaw;

        diablo_base::disps dps;
        float radius = 9;
        float right_displacement = radius*(2 * M_PI * (right_enc - right_init) + (right_pos - right_offset));
        float left_displacement = radius*(2 * M_PI * (left_enc - left_init) + (left_pos - left_offset));
        float cent_displacement = (right_displacement + left_displacement)/2;
        dps.right = right_displacement;
        dps.left = left_displacement;
        dps.centre = cent_displacement;

        // right_init = right_enc;
        // right_offset = right_pos;
        // left_init = left_enc;
        // left_offset = left_pos;


        // diablo_base::coors crs;
        // x_old = xcor;
        // y_old = ycor;
        // xcor = xcor + (cent_displacement)*std::cos(ang.yaw - init_yaw);
        // ycor = ycor + (cent_displacement)*std::sin(ang.yaw - init_yaw);
        // crs.x = xcor;
        // crs.y = ycor;
        

        // nav_msgs::Odometry message;
        // message.pose.pose.position.x = xcor;
        // message.pose.pose.position.y = ycor;
        // message.pose.pose.position.z = 0;
        // message.pose.pose.orientation.x = vals[3];
        // message.pose.pose.orientation.y = vals[4];
        // message.pose.pose.orientation.z = vals[5];
        // message.pose.pose.orientation.w = vals[6];
        // message.twist.twist.linear.x= (xcor-x_old)/0.01;
        // message.twist.twist.linear.y= (ycor-y_old)/0.01;
        // message.twist.twist.linear.z= 0;
        // message.twist.twist.angular.x=vals[10];
        // message.twist.twist.angular.y=vals[11];
        // message.twist.twist.angular.z=vals[12];


        // pub.publish(message);
        pub2.publish(ang);
        pub3.publish(dps);
        // pub4.publish(crs);


        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}