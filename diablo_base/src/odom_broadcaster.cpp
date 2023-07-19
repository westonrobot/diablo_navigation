#define _USE_MATH_DEFINES
#include <iostream>
#include <string>
#include <cmath>
#include <chrono>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// #include "diablo_sdk/OSDK_LEGMOTORS.h"
// #include "Onboard_SDK_Uart_Protocol.h"
// #include "OSDK_Vehicle.hpp"
#include "diablo_sdk/OSDK_LEGMOTORS.h"
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"
#include "diablo_utils/diablo_tools/onboard_sdk_uart_protocol.h"
#include "diablo_base/coors.h"


float right_speed_1, left_speed_1 = 0.0f;
float x_cor, y_cor, theta = 0.0f;
float radius = 0.09f;
float right_dist, left_dist, center_dist = 0.0f;
float x_old, y_old, theta_old = 0.0f;
float duration;

void motorCallback(const diablo_sdk::OSDK_LEGMOTORS::ConstPtr& information)
{
    right_speed_1 = information->right_wheel_vel;
    left_speed_1 = information->left_wheel_vel;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_status");
    ros::NodeHandle node_handle;
    ros::Publisher odom_pub = node_handle.advertise<nav_msgs::Odometry>("/all_data", 10);
    ros::Publisher coor_pub = node_handle.advertise<diablo_base::coors>("/coordinates", 10);
    ros::Subscriber sub_motor = node_handle.subscribe("/robot_status_example/diablo_ros_LEGMOTORS_b", 10, motorCallback);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        right_dist = right_speed_1 * radius * duration;
        left_dist = left_speed_1 * radius * duration;
        center_dist = (right_dist + left_dist) / 2.0f;
        theta = theta_old + std::asin((right_dist - left_dist) / 0.53f);

        if (theta > 2.0f * M_PI)
        {
            theta -= 2.0f * M_PI;
        }
        if (theta < 0.0f)
        {
            theta += 2.0f * M_PI;
        }
        x_cor = x_old + (center_dist) * std::cos(theta);
        y_cor = y_old + (center_dist) * std::sin(theta);

        diablo_base::coors crs;
        crs.x = x_cor;
        crs.y = y_cor;
        crs.theta = theta;
        coor_pub.publish(crs);

        tf2::Quaternion quaternion;
        tf2::Matrix3x3 rotation;
        rotation.setRPY(0.0, 0.0, theta);
        rotation.getRotation(quaternion);

        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()), tf::Vector3(x_cor, y_cor, 0.0)), ros::Time::now(), "odom", "base_link"));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.3, 0.0, 1.5)), ros::Time::now(), "base_link", "laser"));

        nav_msgs::Odometry odom_data;
        odom_data.header.frame_id = "odom";
        odom_data.child_frame_id = "base_link";
        odom_data.pose.pose.position.x = x_cor;
        odom_data.pose.pose.position.y = y_cor;
        odom_data.pose.pose.position.z = 0.0;
        odom_data.pose.pose.orientation.x = quaternion.x();
        odom_data.pose.pose.orientation.y = quaternion.y();
        odom_data.pose.pose.orientation.z = quaternion.z();
        odom_data.pose.pose.orientation.w = quaternion.w();
        odom_data.twist.twist.linear.x = ((right_speed_1 + left_speed_1) / 2.0f) * std::cos(theta);
        odom_data.twist.twist.linear.y = ((right_speed_1 + left_speed_1) / 2.0f) * std::cos(theta);
        odom_data.twist.twist.linear.z = 0.0;
        odom_data.twist.twist.angular.x = 0.0;
        odom_data.twist.twist.angular.y = 0.0;
        odom_data.twist.twist.angular.z = (theta - theta_old) / duration;
        odom_pub.publish(odom_data);

        x_old = x_cor;
        y_old = y_cor;
        theta_old = theta;

        ros::spinOnce();
        loop_rate.sleep();
        auto current_time = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count() / 1000000.0f;
    }

    return 0;
}
