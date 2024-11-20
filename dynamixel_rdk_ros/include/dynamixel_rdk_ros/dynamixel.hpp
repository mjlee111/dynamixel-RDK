#ifndef DYNAMIXEL_RDK_ROS_DYNAMIXEL_HPP_
#define DYNAMIXEL_RDK_ROS_DYNAMIXEL_HPP_

#include "dynamixel_rdk_ros/control_table.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

// dynamixel unit
// position : 0~4095 (0~2pi) -> MinPosition(52) ~ MaxPosition(48)
// velocity : rev/min
// acceleration : rev/min^2
// current : mA
// voltage : V
// temperature : C

// ROS unit
// position : rad
// velocity : rad/s
// acceleration : rad/s^2
// current : A
// voltage : V
// temperature : C

#define M_PI 3.14159265358979323846

// Dynamixel MX Series Profile
#define MX_RPS_PROFILE 0.229     // [rev/min]
#define MX_ACC_PROFILE 214.577   // [rev/min^2]
#define MX_CURRENT_PROFILE 3.36  // [mA]

// Dynamixel PRO Series Profile
#define PRO_RPS_PROFILE 0.01   // [rev/min]
#define PRO_ACC_PROFILE 1      // [rev/min^2]
#define PRO_CURRENT_PROFILE 1  // [mA]

#define MX_RESOLUTION 4095
#define PRO_PH54_RESOLUTION 501923
#define PRO_PM54_RESOLUTION 251417
#define PRO_PH42_RESOLUTION 303750
#define PRO_PM42_RESOLUTION 263187

#define MX_INDIRECT_ADDRESS_START 578
#define PRO_INDIRECT_ADDRESS_START 168
#define INDIRECT_DATA_START 634

#define DXL_CURRENT(current_profile) (1000 / current_profile)  // A -> dxl

#define READ_LENGTH 28

namespace dynamixel_rdk_ros
{

enum DynamixelType { MX, PH54_200, PH54_100, PH42_020, PM54_060, PM54_040, PM42_010 };
extern std::vector<std::string> dynamixel_type_str;

enum AddressNumber {
  TORQUE_ENABLE = 0,           // 1 byte
  HARDWARE_ERROR_STATUS = 1,   // 1 byte
  PROFILE_ACCELERATION = 2,    // 4 byte
  PROFILE_VELOCITY = 6,        // 4 byte
  GOAL_POSITION = 10,          // 4 byte
  MOVING = 14,                 // 1 byte
  PRESENT_CURRENT = 15,        // 2 byte
  PRESENT_VELOCITY = 17,       // 4 byte
  PRESENT_POSITION = 21,       // 4 byte
  PRESENT_INPUT_VOLTAGE = 25,  // 2 byte
  PRESENT_TEMPERATURE = 27     // 1 byte
};

enum DynamixelRebootSequence { STABLE, REBOOT_START, REBOOT_PING, INITIALIZE, SET_TORQUE };

// Dynamixel
class Dynamixel
{
  // Dynamixel Information
public:
  DynamixelType type;
  uint8_t id;
  double min_rad, max_rad;  // [rad]
  uint16_t dynamixel_indirect_address;
  double dxl_rps_ratio, dxl_acc_ratio, dxl_current_ratio;

  double max_velocity_limit;  // [rev/min]

  std::vector<uint8_t> dynamixel_addresses;

  // Dynamixel Status Data
  bool torque_enabled;
  bool is_moving;
  uint8_t error_status;
  double present_position;     // rad
  double present_velocity;     // rad/s
  double present_current;      // mA
  double present_voltage;      // V
  double present_temperature;  // C

  DynamixelRebootSequence reboot_seq_ = DynamixelRebootSequence::STABLE;

  // Dynamixel Control Data
  double goal_position;
  double profile_acceleration;
  double profile_velocity;

  // Dynamixel Information Functions
  DynamixelType get_type() { return type; }
  int get_id() { return id; }
  std::pair<double, double> get_rad_limit() { return std::make_pair(min_rad, max_rad); }
  bool get_torque_status() { return torque_enabled; }
  bool get_moving_status() { return is_moving; }
  uint8_t get_error_status() { return error_status; }
  double get_present_position() { return present_position; }
  double get_present_velocity() { return present_velocity; }
  double get_present_current() { return present_current; }
  double get_present_voltage() { return present_voltage; }
  double get_present_temperature() { return present_temperature; }

  DynamixelRebootSequence get_reboot_sequence() { return reboot_seq_; }

  // Dynamixel Control Functions
public:
  Dynamixel(DynamixelType type, uint8_t id, double min_rad = -M_PI, double max_rad = M_PI);
  ~Dynamixel();

  bool get_dynamixel_status(dynamixel::GroupSyncRead & SyncRead);
  bool get_max_velocity_limit(dynamixel::GroupSyncRead & SyncRead);

  bool set_indirect_address(dynamixel::GroupBulkWrite & BulkWrite);
  bool set_single_indirect_address(
    dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler * port_handler);

  bool set_torque(dynamixel::GroupSyncWrite & SyncWrite, bool torque);
  bool set_single_torque(
    dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler * port_handler, bool torque);

  bool set_position(double position);
  bool set_velocity(double velocity);
  bool set_acceleration(double acceleration);
  bool set_control_data(double position, double velocity, double acceleration);

  bool set_control_data_param(dynamixel::GroupSyncWrite & SyncWrite);

  static uint16_t address(const uint16_t address);

  void reboot_sequence(
    dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler * port_handler);

  std::string get_reboot_sequence_str() const;
  std::string sequence_to_string(DynamixelRebootSequence seq) const;

private:
  void divide_byte(std::vector<uint8_t> & data, int address, int byte_size);
  void add_address(const std::pair<int, int> & control_table);

  double min_max_rad(int32_t min, int32_t max, int32_t position);
  double to_rad(int32_t position);
  int32_t min_max_position(int32_t min, int32_t max, double rad);
  int32_t to_position(double rad);
};

}  // namespace dynamixel_rdk_ros

#endif  // DYNAMIXEL_RDK_ROS_DYNAMIXEL_HPP_
