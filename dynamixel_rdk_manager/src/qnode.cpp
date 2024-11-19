/*
 * Copyright 2024 Myeong Jin Lee
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "../include/dynamixel_rdk_manager/qnode.hpp"

QNode::QNode()
{
  int argc = 0;
  char ** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("dynamixel_rdk_manager");

  RCLCPP_INFO(node->get_logger(), "DynamixelRDKManager is created");

  bulk_read_sub_ = node->create_subscription<dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs>(
    "/dynamixel_status", 10, std::bind(&QNode::status_callback, this, std::placeholders::_1));
  control_pub_ =
    node->create_publisher<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>("/dynamixel_control", 1);

  this->start();
}

void QNode::control_publish()
{
  RCLCPP_INFO(node->get_logger(), "DynamixelRDKManager is publishing control");
  dynamixel_rdk_msgs::msg::DynamixelControlMsgs msg;
  for (size_t i = 0; i < dynamixel_ids_.size(); ++i) {
    dynamixel_rdk_msgs::msg::DynamixelMsgs motor_control;
    motor_control.header.frame_id = dynamixel_types_[i];
    motor_control.goal_position = goal_positions_[i];
    motor_control.profile_acceleration = goal_accelerations_[i];
    motor_control.profile_velocity = goal_velocities_[i];
    msg.motor_control.push_back(motor_control);
  }
  control_pub_->publish(msg);
}

void QNode::status_callback(const dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs & msg)
{
  dynamixel_ids_.clear();
  dynamixel_types_.clear();
  torque_enabled_.clear();
  error_status_.clear();
  present_positions_.clear();
  present_velocities_.clear();
  present_accelerations_.clear();
  present_currents_.clear();
  present_voltages_.clear();
  present_temperatures_.clear();
  min_max_positions_.clear();

  for (auto & status : msg.status_msgs) {
    dynamixel_ids_.push_back(status.id);
    dynamixel_types_.push_back(status.header.frame_id);
    torque_enabled_.push_back(status.torque_enabled);
    error_status_.push_back(status.error_status);
    present_positions_.push_back(status.present_position);
    present_velocities_.push_back(status.present_velocity);
    present_accelerations_.push_back(status.present_acceleration);
    present_currents_.push_back(status.present_current);
    present_voltages_.push_back(status.present_voltage);
    present_temperatures_.push_back(status.present_temperature);
    min_max_positions_.push_back(
      std::make_pair(status.min_max_position[0], status.min_max_position[1]));
  }

  dynamixel_size_ = dynamixel_ids_.size();
  if (!is_size_initialized_ || dynamixel_size_ != dynamixel_size_) {
    goal_positions_.resize(dynamixel_size_);
    goal_velocities_.resize(dynamixel_size_);
    goal_accelerations_.resize(dynamixel_size_);

    for (size_t i = 0; i < dynamixel_size_; ++i) {
      goal_positions_[i] = present_positions_[i];
      goal_velocities_[i] = present_velocities_[i];
      goal_accelerations_[i] = present_accelerations_[i];
    }
    is_size_initialized_ = true;
    Q_EMIT update_id_list();
  }

  Q_EMIT update_data();
}

QNode::~QNode()
{
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(100);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}
