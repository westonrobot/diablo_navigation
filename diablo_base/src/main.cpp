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

#include <ros/ros.h>

#include "diablo_base/diablo_robot.hpp"

using namespace westonrobot;

int main(int argc, char** argv) {
  ros::init(argc, argv, "diablo_base");
  ros::NodeHandle n("~");

  DiabloRobot robot(&n);
  if (!robot.Initialize()) {
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