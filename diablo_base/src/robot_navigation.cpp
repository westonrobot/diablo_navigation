#define _USE_MATH_DEFINES
#include <cmath>
#include <chrono>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// #include "diablo_sdk/OSDK_LEGMOTORS.h"
// #include "Onboard_SDK_Uart_Protocol.h"
// #include "OSDK_Vehicle.hpp"
#include "diablo_sdk/OSDK_LEGMOTORS.h"
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"
#include "diablo_utils/diablo_tools/onboard_sdk_uart_protocol.h"
#include "diablo_base/coors.h"

float right_speed_1, left_speed_1 = 0;
float xcor, ycor, theta = 0;
float radius = 0.09;
float right_dist, left_dist, centre_dist = 0;
float xold, yold, theta_old = 0;
float duration;

DIABLO::OSDK::Movement_Ctrl* pMovementCtrl;

void moveDiablo(const geometry_msgs::Twist::ConstPtr& msg)
{
    if (!pMovementCtrl->in_control())
    {
        std::cout << "To get movement ctrl." << std::endl;
        uint8_t result = pMovementCtrl->obtain_control();
        return;
    }

    if (pMovementCtrl->ctrl_mode_data.height_ctrl_mode == 1)
        pMovementCtrl->ctrl_data.up = 0.0f;

    pMovementCtrl->ctrl_data.forward = 0.0f;
    pMovementCtrl->ctrl_data.left = 0.0f;

    pMovementCtrl->ctrl_data.forward = msg->linear.x;
    pMovementCtrl->ctrl_data.left = msg->angular.z;
    std::cout << "Sending data" << std::endl;

    if (pMovementCtrl->ctrl_mode_cmd)
    {
        uint8_t result = pMovementCtrl->SendMovementModeCtrlCmd();
    }
    else
    {
        uint8_t result = pMovementCtrl->SendMovementCtrlCmd();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_base");
    ros::NodeHandle n("~");

    DIABLO::OSDK::HAL_Pi Hal;
    if (Hal.init())
        return -1;

    DIABLO::OSDK::Vehicle vehicle(&Hal);
    if (vehicle.init())
        return -1;

    vehicle.telemetry->activate();

    pMovementCtrl = vehicle.movement_ctrl;

    ros::Subscriber sub = n.subscribe("/cmd_vel", 1, moveDiablo);

    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR, OSDK_PUSH_DATA_10Hz);
    vehicle.telemetry->configUpdate();

    ros::Publisher ODOMPub = n.advertise<nav_msgs::Odometry>("/odom", 10);
    ros::Publisher COORPub = n.advertise<diablo_base::coors>("/coordinates", 10);
    ros::Rate loop_rate(100);
    tf::TransformBroadcaster broadcaster;

    while (ros::ok())
    {
        if (vehicle.telemetry->newcome & 0x01)
        {
            left_speed_1 = vehicle.telemetry->motors.left_wheel.vel;
            right_speed_1 = vehicle.telemetry->motors.right_wheel.vel;
            vehicle.telemetry->eraseNewcomeFlag(0xFE);
        }

        auto startTime = std::chrono::high_resolution_clock::now();

        right_dist = right_speed_1 * radius * duration;
        left_dist = left_speed_1 * radius * duration;

        centre_dist = (right_dist + left_dist) / 2;

        theta = theta_old + std::asin((right_dist - left_dist) / 0.53);

        if (theta > 2 * M_PI)
        {
            theta = theta - 2 * M_PI;
        }
        if (theta < 0)
        {
            theta = theta + 2 * M_PI;
        }

        xcor = xold + (centre_dist) * std::cos(theta);
        ycor = yold + (centre_dist) * std::sin(theta);

        diablo_base::coors crs;
        crs.x = xcor;
        crs.y = ycor;
        crs.theta = theta;
        COORPub.publish(crs);

        tf2::Quaternion quaternion;
        tf2::Matrix3x3 rotation;
        rotation.setRPY(0, 0, theta);
        rotation.getRotation(quaternion);

        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()), tf::Vector3(xcor, ycor, 0)),
                ros::Time::now(), "odom", "base_link"));

        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.3, 0, 1.5)),
                ros::Time::now(), "base_link", "laser"));

        nav_msgs::Odometry odom_data;
        odom_data.header.frame_id = "odom";
        odom_data.child_frame_id = "base_link";
        odom_data.pose.pose.position.x = xcor;
        odom_data.pose.pose.position.y = ycor;
        odom_data.pose.pose.position.z = 0;
        odom_data.pose.pose.orientation.x = quaternion.x();
        odom_data.pose.pose.orientation.y = quaternion.y();
        odom_data.pose.pose.orientation.z = quaternion.z();
        odom_data.pose.pose.orientation.w = quaternion.w();
        odom_data.twist.twist.linear.x = ((right_speed_1 + left_speed_1) / 2) * std::cos(theta);
        odom_data.twist.twist.linear.y = ((right_speed_1 + left_speed_1) / 2) * std::cos(theta);
        odom_data.twist.twist.linear.z = 0;
        odom_data.twist.twist.angular.x = 0;
        odom_data.twist.twist.angular.y = 0;
        odom_data.twist.twist.angular.z = (theta - theta_old) / duration;
        ODOMPub.publish(odom_data);

        xold = xcor;
        yold = ycor;
        theta_old = theta;

        ros::spinOnce();
        loop_rate.sleep();
        auto currentTime = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - startTime).count() / 1000000.00;
    }

    return 0;
}