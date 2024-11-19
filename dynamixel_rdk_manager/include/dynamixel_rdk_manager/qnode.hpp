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

#ifndef dynamixel_rdk_manager_QNODE_HPP_
#define dynamixel_rdk_manager_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif
#include <dynamixel_rdk_msgs/msg/dynamixel_bulk_read_msgs.hpp>
#include <dynamixel_rdk_msgs/msg/dynamixel_control_msgs.hpp>

#include <QThread>
#include <vector>

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
  Q_OBJECT
public:  // Public Functions
  QNode();
  ~QNode();

  void control_publish();

protected:  // Protected Functions
  void run();

private:  // Private Functions
  void status_callback(const dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs & msg);

private:  // Private Variables
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs>::SharedPtr bulk_read_sub_;
  rclcpp::Publisher<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>::SharedPtr control_pub_;

public:  // Public Variables
  // Status
  std::vector<uint8_t> dynamixel_ids_;
  std::vector<bool> torque_enabled_;
  std::vector<std::string> dynamixel_types_;
  std::vector<uint8_t> error_status_;
  std::vector<double> present_positions_;
  std::vector<double> present_velocities_;
  std::vector<double> present_accelerations_;
  std::vector<double> present_currents_;
  std::vector<double> present_voltages_;
  std::vector<double> present_temperatures_;
  std::vector<std::pair<double, double>> min_max_positions_;

  // Control
  std::vector<double> goal_positions_;
  std::vector<double> goal_velocities_;
  std::vector<double> goal_accelerations_;

  int dynamixel_size_ = 0;
  bool is_size_initialized_ = false;

Q_SIGNALS:  // Q_SIGNALS
  void rosShutDown();
  void update_data();
  void update_id_list();
};

#endif /* dynamixel_rdk_manager_QNODE_HPP_ */
