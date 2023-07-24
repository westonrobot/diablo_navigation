/**
 * @file diablo_robot.cpp
 * @date 19-07-2023
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "diablo_base/diablo_robot.hpp"

#include <tf/transform_datatypes.h>

namespace westonrobot {
DiabloRobot::DiabloRobot(ros::NodeHandle* nh) : nh_(nh) {}

bool DiabloRobot::Initialize() {
  if (hal_.init()) {
    ROS_ERROR("Failed to initialize HAL");
    return false;
  }

  vehicle_ = std::make_unique<DIABLO::OSDK::Vehicle>(&hal_);
  if (vehicle_->init()) {
    ROS_ERROR("Failed to initialize Vehicle");
    return false;
  }
  vehicle_->telemetry->activate();
  movement_controller_ = vehicle_->movement_ctrl;

  vehicle_->telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR,
                                   OSDK_PUSH_DATA_10Hz);
  vehicle_->telemetry->configUpdate();

  // setup ros
  odom_pub_ = nh_->advertise<nav_msgs::Odometry>("/odom", 10);
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
    float left_speed_ =
        vehicle_->telemetry->motors.left_wheel.vel * kWheelRadius;
    float right_speed_ =
        vehicle_->telemetry->motors.right_wheel.vel * kWheelRadius;
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
    float linear_speed = (right_speed_ + left_speed_) / 2;
    float angular_speed = (right_speed_ - left_speed_) / kWheelTrack;

    position_x_ += linear_speed * std::cos(theta_) * dt;
    position_y_ += linear_speed * std::sin(theta_) * dt;
    theta_ += angular_speed * dt;

    // limit theta between -pi and pi
    if (theta_ > M_PI) {
      theta_ = theta_ - 2 * M_PI;
    }
    if (theta_ < -M_PI) {
      theta_ = theta_ + 2 * M_PI;
    }

    ROS_INFO("Wheel speed: %f, %f, Robot speed: %f, %f, Robot pose: %f, %f, %f",
             left_speed_, right_speed_, linear_speed, angular_speed,
             position_x_, position_y_, theta_ / M_PI * 180.0);

    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(theta_);

    // publish messages to ROS
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;
    tf_broadcaster_.sendTransform(tf_msg);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;
    odom_msg.twist.twist.linear.x = linear_speed;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = angular_speed;
    odom_pub_.publish(odom_msg);
  }
}
}  // namespace westonrobot