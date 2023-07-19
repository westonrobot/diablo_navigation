/**
 * @file diablo_robot.cpp
 * @date 19-07-2023
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "diablo_base/diablo_robot.hpp"

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "diablo_base/coors.h"

namespace westonrobot {
DiabloRobot::DiabloRobot(ros::NodeHandle* nh) : nh_(nh) {}

bool DiabloRobot::Initialize() {
  if (hal_.init()) return false;

  vehicle_ = std::make_unique<DIABLO::OSDK::Vehicle>(&hal_);
  if (vehicle_->init()) return false;
  vehicle_->telemetry->activate();
  movement_controller_ = vehicle_->movement_ctrl;

  vehicle_->telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR,
                                   OSDK_PUSH_DATA_10Hz);
  vehicle_->telemetry->configUpdate();

  // setup ros
  odom_pub_ = nh_->advertise<nav_msgs::Odometry>("/odom", 10);
  pose_pub_ = nh_->advertise<diablo_base::coors>("/coordinates", 10);
  cmd_sub_ =
      nh_->subscribe("/cmd_vel", 1, &DiabloRobot::TwistCmdCallback, this);

  return true;
}

void DiabloRobot::TwistCmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  if (!movement_controller_->in_control()) {
    std::cout << "To get movement ctrl." << std::endl;
    uint8_t result = movement_controller_->obtain_control();
    return;
  }

  if (movement_controller_->ctrl_mode_data.height_ctrl_mode == 1)
    movement_controller_->ctrl_data.up = 0.0f;

  movement_controller_->ctrl_data.forward = 0.0f;
  movement_controller_->ctrl_data.left = 0.0f;

  movement_controller_->ctrl_data.forward = msg->linear.x;
  movement_controller_->ctrl_data.left = msg->angular.z;
  std::cout << "Sending data" << std::endl;

  if (movement_controller_->ctrl_mode_cmd) {
    uint8_t result = movement_controller_->SendMovementModeCtrlCmd();
  } else {
    uint8_t result = movement_controller_->SendMovementCtrlCmd();
  }
}

void DiabloRobot::Update() {
  if (vehicle_->telemetry->newcome & 0x01) {
    left_speed_1_ = vehicle_->telemetry->motors.left_wheel.vel;
    right_speed_1_ = vehicle_->telemetry->motors.right_wheel.vel;
    vehicle_->telemetry->eraseNewcomeFlag(0xFE);

    if (is_first_run_) {
      is_first_run_ = false;
      last_time_ = ros::Time::now();
      return;
    }

    auto current_time = ros::Time::now();
    double dt = (current_time - last_time_).toSec();
    last_time_ = current_time;

    // calculate odometry
    right_dist_ = right_speed_1_ * radius_ * dt;
    left_dist_ = left_speed_1_ * radius_ * dt;

    centre_dist_ = (right_dist_ + left_dist_) / 2;

    theta_ = theta_old_ + std::asin((right_dist_ - left_dist_) / 0.53);

    if (theta_ > 2 * M_PI) {
      theta_ = theta_ - 2 * M_PI;
    }
    if (theta_ < 0) {
      theta_ = theta_ + 2 * M_PI;
    }

    xcor_ = xold_ + (centre_dist_)*std::cos(theta_);
    ycor_ = yold_ + (centre_dist_)*std::sin(theta_);

    // publish messages to ROS
    diablo_base::coors crs;
    crs.x = xcor_;
    crs.y = ycor_;
    crs.theta = theta_;
    pose_pub_.publish(crs);

    tf2::Quaternion quaternion;
    tf2::Matrix3x3 rotation;
    rotation.setRPY(0, 0, theta_);
    rotation.getRotation(quaternion);

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = xcor_;
    tf_msg.transform.translation.y = ycor_;
    tf_msg.transform.translation.z = 0;
    tf_msg.transform.rotation.x = quaternion.x();
    tf_msg.transform.rotation.y = quaternion.y();
    tf_msg.transform.rotation.z = quaternion.z();
    tf_msg.transform.rotation.w = quaternion.w();
    tf_broadcaster_.sendTransform(tf_msg);

    nav_msgs::Odometry odom_data;
    odom_data.header.stamp = current_time;
    odom_data.header.frame_id = "odom";
    odom_data.child_frame_id = "base_link";
    odom_data.pose.pose.position.x = xcor_;
    odom_data.pose.pose.position.y = ycor_;
    odom_data.pose.pose.position.z = 0;
    odom_data.pose.pose.orientation.x = quaternion.x();
    odom_data.pose.pose.orientation.y = quaternion.y();
    odom_data.pose.pose.orientation.z = quaternion.z();
    odom_data.pose.pose.orientation.w = quaternion.w();
    odom_data.twist.twist.linear.x =
        ((right_speed_1_ + left_speed_1_) / 2) * std::cos(theta_);
    odom_data.twist.twist.linear.y =
        ((right_speed_1_ + left_speed_1_) / 2) * std::cos(theta_);
    odom_data.twist.twist.linear.z = 0;
    odom_data.twist.twist.angular.x = 0;
    odom_data.twist.twist.angular.y = 0;
    odom_data.twist.twist.angular.z = (theta_ - theta_old_) / dt;
    odom_pub_.publish(odom_data);

    xold_ = xcor_;
    yold_ = ycor_;
    theta_old_ = theta_;
  }
}
}  // namespace westonrobot