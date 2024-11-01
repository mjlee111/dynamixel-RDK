/**
 * @file /include/dynamixel_rdk_ctrl/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef dynamixel_rdk_ctrl_QNODE_HPP_
#define dynamixel_rdk_ctrl_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif
#include "dynamixel_rdk_msgs/msg/dynamixel_control_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_msgs.hpp"

#include <QThread>

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();

  void set_position(int pos1, int pos2, int pos3);

  double pos_to_degree(int pos);
  double pos_to_radian(int pos);
  int degree_to_pos(double degree);
  int radian_to_pos(double radian);

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>::SharedPtr pub;
Q_SIGNALS:
  void rosShutDown();
};

#endif /* dynamixel_rdk_ctrl_QNODE_HPP_ */
