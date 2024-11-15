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

#include "../include/dynamixel_rdk_ros/dynamixel_rdk_node.hpp"

#include <rclcpp/rclcpp.hpp>
namespace dynamixel_rdk_ros
{

DynamixelRDKNode::DynamixelRDKNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("dynamixel_rdk_node", options)
{
  RCLCPP_INFO(get_logger(), "Creating node");
  init_parameters();
  RCLCPP_INFO(get_logger(), "Parameters initialized");
}

DynamixelRDKNode::~DynamixelRDKNode()
{
  RCLCPP_INFO(get_logger(), "Destroying node");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DynamixelRDKNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring node");
  get_parameters();

  dynamixel_status_pub_ =
    create_publisher<dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs>("dynamixel_status", 1);

  dynamixel_control_sub_ = create_subscription<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>(
    "dynamixel_control", 10,
    std::bind(&DynamixelRDKNode::dynamixel_control_callback, this, std::placeholders::_1));

  try {
    dynamixel_ctrl_ = std::make_shared<DynamixelCtrl>(
      device_port_, baud_rate_, dynamixel_ids_, dynamixel_types_, max_position_limits_,
      min_position_limits_);
    RCLCPP_INFO(get_logger(), "Dynamixel controller initialized");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize Dynamixel controller: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DynamixelRDKNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating node");

  dynamixel_status_pub_->on_activate();

  try {
    dynamixel_ctrl_->set_torque(true);
    RCLCPP_INFO(get_logger(), "Torque enabled");
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&DynamixelRDKNode::dynamixel_status_publish, this));

    RCLCPP_INFO(get_logger(), "Status publish timer started");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to enable torque: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DynamixelRDKNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating node");

  status_timer_->reset();
  dynamixel_status_pub_->on_deactivate();

  try {
    dynamixel_ctrl_->set_torque(false);
    RCLCPP_INFO(get_logger(), "Torque disabled");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to enable torque: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DynamixelRDKNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up node");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DynamixelRDKNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down node");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult DynamixelRDKNode::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    RCLCPP_INFO(get_logger(), "Parameter '%s' changed", param.get_name().c_str());
  }

  return result;
}

void DynamixelRDKNode::init_parameters()
{
  // ROS2 Parameters
  declare_parameter("device_port", rclcpp::ParameterValue("/dev/ttyUSB0"));
  declare_parameter("baud_rate", rclcpp::ParameterValue(1000000));

  declare_parameter("dynamixels.ids", std::vector<int64_t>{1});
  declare_parameter("dynamixels.types", std::vector<std::string>{"MX"});
  declare_parameter("dynamixels.max_position_limits", std::vector<double>{M_PI});
  declare_parameter("dynamixels.min_position_limits", std::vector<double>{-M_PI});
}

void DynamixelRDKNode::get_parameters()
{
  device_port_ = get_parameter("device_port").as_string();
  baud_rate_ = get_parameter("baud_rate").as_int();

  std::vector<int64_t> dynamixel_ids = get_parameter("dynamixels.ids").as_integer_array();
  for (auto & id : dynamixel_ids) {
    dynamixel_ids_.push_back(static_cast<uint8_t>(id));
  }
  std::vector<std::string> dynamixel_types = get_parameter("dynamixels.types").as_string_array();
  for (auto & type : dynamixel_types) {
    if (type == "MX") {
      dynamixel_types_.push_back(DynamixelType::MX);
    } else if (type == "PH54_200") {
      dynamixel_types_.push_back(DynamixelType::PH54_200);
    } else if (type == "PH54_100") {
      dynamixel_types_.push_back(DynamixelType::PH54_100);
    } else if (type == "PH42_020") {
      dynamixel_types_.push_back(DynamixelType::PH42_020);
    } else if (type == "PM54_060") {
      dynamixel_types_.push_back(DynamixelType::PM54_060);
    } else if (type == "PM54_040") {
      dynamixel_types_.push_back(DynamixelType::PM54_040);
    } else if (type == "PM42_010") {
      dynamixel_types_.push_back(DynamixelType::PM42_010);
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid dynamixel type: %s", type.c_str());
    }
  }
  max_position_limits_ = get_parameter("dynamixels.max_position_limits").as_double_array();
  min_position_limits_ = get_parameter("dynamixels.min_position_limits").as_double_array();

  if (
    dynamixel_ids_.size() != dynamixel_types_.size() ||
    dynamixel_ids_.size() != max_position_limits_.size() ||
    dynamixel_ids_.size() != min_position_limits_.size()) {
    RCLCPP_ERROR(
      get_logger(), "Dynamixel ID, Type, Max Position Limit, and Min Position Limit size mismatch");
  }
}

void DynamixelRDKNode::dynamixel_control_callback(
  const dynamixel_rdk_msgs::msg::DynamixelControlMsgs & msg)
{
  std::vector<double> goal_positions;
  std::vector<double> goal_velocities;
  std::vector<double> goal_accelerations;
  for (auto & motor_control : msg.motor_control) {
    goal_positions.push_back(motor_control.goal_position);
    goal_velocities.push_back(motor_control.profile_velocity);
    goal_accelerations.push_back(motor_control.profile_acceleration);
  }
  dynamixel_ctrl_->sync_write(goal_positions, goal_velocities, goal_accelerations);
}

void DynamixelRDKNode::dynamixel_status_publish()
{
  try {
    dynamixel_ctrl_->read_dynamixel_status();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to read dynamixel status: %s", e.what());
  }

  dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs status_msg;

  for (auto & dynamixel : dynamixel_ctrl_->dynamixels) {
    dynamixel_rdk_msgs::msg::DynamixelStatusMsgs status;
    status.header.stamp = get_clock()->now();
    status.id = dynamixel->id;
    status.torque_enabled = dynamixel->torque_enabled;
    status.present_position = dynamixel->present_position;
    status.present_velocity = dynamixel->present_velocity;
    status.present_acceleration = dynamixel->profile_acceleration;
    status.present_current = dynamixel->present_current;
    status.present_voltage = dynamixel->present_voltage;
    status.present_temperature = dynamixel->present_temperature;
    status_msg.status_msgs.push_back(status);
  }
  dynamixel_status_pub_->publish(status_msg);
}

}  // namespace dynamixel_rdk_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dynamixel_rdk_ros::DynamixelRDKNode>();

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
