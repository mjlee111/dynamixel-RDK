/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date August 2024
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/dynamixel_rdk_ctrl/qnode.hpp"

QNode::QNode()
{
  int argc = 0;
  char ** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("dynamixel_rdk_ctrl");

  pub =
    node->create_publisher<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>("dynamixel_control", 10);
  this->start();
}

QNode::~QNode()
{
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

void QNode::set_position(int pos1, int pos2, int pos3)
{
  dynamixel_rdk_msgs::msg::DynamixelControlMsgs ctrl_msg;

  dynamixel_rdk_msgs::msg::DynamixelMsgs msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "dynamixel_rdk";
  msg.id = 1;
  msg.goal_position = pos1;
  msg.profile_acceleration = 1;
  msg.profile_velocity = 2;
  ctrl_msg.motor_control.push_back(msg);

  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "dynamixel_rdk";
  msg.id = 2;
  msg.goal_position = pos2;
  msg.profile_acceleration = 1;
  msg.profile_velocity = 2;
  ctrl_msg.motor_control.push_back(msg);

  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "dynamixel_rdk";
  msg.id = 3;
  msg.goal_position = pos3;
  msg.profile_acceleration = 1;
  msg.profile_velocity = 2;
  ctrl_msg.motor_control.push_back(msg);

  pub->publish(ctrl_msg);
}

double QNode::pos_to_degree(int pos)
{
  return pos * 300.0 / 4096.0;
}

double QNode::pos_to_radian(int pos)
{
  return pos * 2.0 * M_PI / 4096.0;
}

int QNode::degree_to_pos(double degree)
{
  return static_cast<int>(degree * 4096.0 / 300.0);
}

int QNode::radian_to_pos(double radian)
{
  return static_cast<int>(radian * 4096.0 / (2.0 * M_PI));
}
