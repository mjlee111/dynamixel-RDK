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

#ifndef DYNAMIXEL_RDK_ROS_DYNAMIXEL_RDK_NODE_HPP_
#define DYNAMIXEL_RDK_ROS_DYNAMIXEL_RDK_NODE_HPP_

#include "dynamixel_rdk_ros/dynamixel.hpp"
#include "dynamixel_rdk_ros/dynamixel_ctrl.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "dynamixel_rdk_msgs/msg/dynamixel_bulk_read_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_control_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_status_msgs.hpp"
#include <lifecycle_msgs/msg/state.hpp>
#include <std_msgs/msg/header.hpp>

#include <string>

namespace dynamixel_rdk_ros
{

class DynamixelRDKNode : public rclcpp_lifecycle::LifecycleNode
{
public:  // Public Functions
  DynamixelRDKNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DynamixelRDKNode();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State &);
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters);

private:  // Private Functions
  void init_parameters();
  void get_parameters();

  void dynamixel_control_callback(const dynamixel_rdk_msgs::msg::DynamixelControlMsgs & msg);

  void dynamixel_status_publish();

private:  // Private Variables
  rclcpp_lifecycle::LifecyclePublisher<dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs>::SharedPtr
    dynamixel_status_pub_;
  rclcpp::Subscription<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>::SharedPtr
    dynamixel_control_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::string device_port_;
  int baud_rate_;
  std::shared_ptr<DynamixelCtrl> dynamixel_ctrl_;

  std::vector<uint8_t> dynamixel_ids_;
  std::vector<DynamixelType> dynamixel_types_;
  std::vector<double> max_position_limits_;
  std::vector<double> min_position_limits_;
};

}  // namespace dynamixel_rdk_ros

#endif  // DYNAMIXEL_RDK_ROS_DYNAMIXEL_RDK_NODE_HPP_