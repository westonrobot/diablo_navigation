/**
 * @file diablo_robot.hpp
 * @date 19-07-2023
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef DIABLO_ROBOT_HPP
#define DIABLO_ROBOT_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

namespace westonrobot {
class DiabloRobot {
 public:
  DiabloRobot(ros::NodeHandle* nh);
  ~DiabloRobot() = default;

  bool Initialize();
  void Update();

 private:
  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

  ros::NodeHandle* nh_;
  ros::Publisher odom_pub_;
  ros::Publisher pose_pub_;
  ros::Subscriber cmd_sub_;
  tf::TransformBroadcaster tf_broadcaster_;

  DIABLO::OSDK::HAL_Pi hal_;
  std::unique_ptr<DIABLO::OSDK::Vehicle> vehicle_;
  DIABLO::OSDK::Movement_Ctrl* movement_controller_;

  bool is_first_run_ = true;
  ros::Time last_time_;

  float right_speed_1_ = 0;
  float left_speed_1_ = 0;

  float xcor_ = 0;
  float ycor_ = 0;
  float theta_ = 0;
  float xold_, yold_, theta_old_ = 0;
  
  float radius_ = 0.09;
  float right_dist_ = 0;
  float left_dist_ = 0;
  float centre_dist_ = 0;
};
}  // namespace westonrobot

#endif /* DIABLO_ROBOT_HPP */