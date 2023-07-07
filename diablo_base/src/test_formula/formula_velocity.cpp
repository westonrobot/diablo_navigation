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

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define _USE_MATH_DEFINES
#include <cmath>


float right_speed_1, left_speed_1 = 0;
float xcor, ycor, theta = 0;
int radius = 9;
float right_dist, left_dist, centre_dist = 0;
float xold, yold, theta_old = 0;

void motor_callback(const diablo_sdk::OSDK_LEGMOTORS::ConstPtr& information){
    right_speed_1= information->right_wheel_vel;
    left_speed_1 = information->left_wheel_vel;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "status_node");
    ros::NodeHandle n;
    ros::Publisher ODOMPub = n.advertise<nav_msgs::Odometry>("/all_data", 10);
    ros::Publisher COORPub = n.advertise<diablo_base::coors>("/coordinates", 10);

    ros::Subscriber sub_motor = n.subscribe("/robot_status_example/diablo_ros_LEGMOTORS_b", 10, motor_callback);

    tf::TransformBroadcaster broadcaster;
    
    ros::Rate loop_rate(100);
    while(ros::ok()){
        
        right_dist = (right_speed_1)*0.001;
        left_dist = (left_speed_1)*0.001;
        centre_dist = (right_dist + left_dist)/2;
        theta = theta_old + (((1.0/0.525)*(right_dist-left_dist)));
        // if(theta > 2*M_PI){
        //     theta = theta - 2*M_PI;
        // }
        // if(theta < 0){
        //     theta = theta + 2*M_PI;
        // }
        xcor = xold + (centre_dist)*std::cos(theta);
        ycor = yold + (centre_dist)*std::sin(theta);

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
            ros::Time::now(),"odom", "base_link")
        );

        broadcaster.sendTransform(
            tf::StampedTransform(
            tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.3, 0, 1.5)),
            ros::Time::now(),"base_link", "laser")
        );

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
        odom_data.twist.twist.linear.x= ((right_speed_1+left_speed_1)/2)*std::cos(theta); 
        odom_data.twist.twist.linear.y= ((right_speed_1+left_speed_1)/2)*std::cos(theta); 
        odom_data.twist.twist.linear.z= 0;
        odom_data.twist.twist.angular.x=0;
        odom_data.twist.twist.angular.y=0;
        odom_data.twist.twist.angular.z=(theta-theta_old)/0.01;
        ODOMPub.publish(odom_data);

        xold = xcor;
        yold = ycor;
        theta_old = theta;
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}