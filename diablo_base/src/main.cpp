/**
 * @file diablo_base.cpp
 * @date 19-07-2023
 * @brief
 *
 * @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
 */

#include <cmath>
#include <chrono>
#include <iostream>

#include "diablo_base/diablo_robot.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

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

using namespace westonrobot;

int main(int argc, char** argv) {
  ros::init(argc, argv, "diablo_base");
  ros::NodeHandle n("~");

  DiabloRobot robot(&n);
  if (robot.Initialize()) {
    ROS_ERROR("Failed to initialize robot");
    return -1;
  }

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    robot.Update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}