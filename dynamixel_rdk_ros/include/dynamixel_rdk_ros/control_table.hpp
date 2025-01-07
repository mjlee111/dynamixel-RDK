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

#ifndef DYNAMIXEL_RDK_ROS_CONTROL_TABLE_HPP_
#define DYNAMIXEL_RDK_ROS_CONTROL_TABLE_HPP_

#include <utility>

namespace dynamixel_rdk_ros
{
//////////////////////////////////////////////////////////////////////////////////////////////////

namespace EEPROM
{
constexpr std::pair<int, int> MODEL_NUMBER = {0, 2};       // Model Number, Read, Default: 321
constexpr std::pair<int, int> MODEL_INFORMATION = {2, 4};  // Model Information, Read, Default: -
constexpr std::pair<int, int> FIRMWARE_VERSION = {6, 1};   // Firmware Version, Read, Default: -
constexpr std::pair<int, int> ID = {7, 1};                 // ID, Read/Write, Default: 1
constexpr std::pair<int, int> BAUD_RATE = {8, 1};          // Baud Rate, Read/Write, Default: 1
constexpr std::pair<int, int> RETURN_DELAY_TIME = {
  9, 1};                                             // Return Delay Time, Read/Write, Default: 250
constexpr std::pair<int, int> DRIVE_MODE = {10, 1};  // Drive Mode, Read/Write, Default: 0
constexpr std::pair<int, int> OPERATING_MODE = {11, 1};  // Operating Mode, Read/Write, Default: 3
constexpr std::pair<int, int> SECONDARY_ID = {
  12, 1};  // Secondary (Shadow) ID, Read/Write, Default: 255
constexpr std::pair<int, int> PROTOCOL_TYPE = {13, 1};  // Protocol Type, Read/Write, Default: 2
constexpr std::pair<int, int> HOMING_OFFSET = {20, 4};  // Homing Offset, Read/Write, Default: 0
constexpr std::pair<int, int> MOVING_THRESHOLD = {
  24, 4};  // Moving Threshold, Read/Write, Default: 10
constexpr std::pair<int, int> TEMPERATURE_LIMIT = {
  31, 1};  // Temperature Limit, Read/Write, Default: 80
constexpr std::pair<int, int> MAX_VOLTAGE_LIMIT = {
  32, 2};  // Max Voltage Limit, Read/Write, Default: 160
constexpr std::pair<int, int> MIN_VOLTAGE_LIMIT = {
  34, 2};                                           // Min Voltage Limit, Read/Write, Default: 95
constexpr std::pair<int, int> PWM_LIMIT = {36, 2};  // PWM Limit, Read/Write, Default: 885
constexpr std::pair<int, int> CURRENT_LIMIT = {38, 2};  // Current Limit, Read/Write, Default: 2047
constexpr std::pair<int, int> ACCELERATION_LIMIT = {
  40, 4};  // Acceleration Limit, Read/Write, Default: 32767
constexpr std::pair<int, int> VELOCITY_LIMIT = {44, 4};  // Velocity Limit, Read/Write, Default: 210
constexpr std::pair<int, int> MAX_POSITION_LIMIT = {
  48, 4};  // Max Position Limit, Read/Write, Default: 4,095
constexpr std::pair<int, int> MIN_POSITION_LIMIT = {
  52, 4};                                          // Min Position Limit, Read/Write, Default: 0
constexpr std::pair<int, int> SHUTDOWN = {63, 1};  // Shutdown, Read/Write, Default: 52 std::
}  // namespace EEPROM

// Dynamixel MX Series RAM Area
namespace MXRAM
{
constexpr std::pair<int, int> TORQUE_ENABLE = {64, 1};  // Torque Enable, Read/Write, Default: 0
constexpr std::pair<int, int> LED = {65, 1};            // LED, Read/Write, Default: 0
constexpr std::pair<int, int> STATUS_RETURN_LEVEL = {
  68, 1};  // Status Return Level, Read/Write, Default: 2
constexpr std::pair<int, int> REGISTERED_INSTRUCTION = {
  69, 1};  // Registered Instruction, Read, Default: 0
constexpr std::pair<int, int> HARDWARE_ERROR_STATUS = {
  70, 1};  // Hardware Error Status, Read, Default: 0
constexpr std::pair<int, int> VELOCITY_I_GAIN = {
  76, 2};  // Velocity I Gain, Read/Write, Default: 1920
constexpr std::pair<int, int> VELOCITY_P_GAIN = {
  78, 2};  // Velocity P Gain, Read/Write, Default: 100
constexpr std::pair<int, int> POSITION_D_GAIN = {80, 2};  // Position D Gain, Read/Write, Default: 0
constexpr std::pair<int, int> POSITION_I_GAIN = {82, 2};  // Position I Gain, Read/Write, Default: 0
constexpr std::pair<int, int> POSITION_P_GAIN = {
  84, 2};  // Position P Gain, Read/Write, Default: 850
constexpr std::pair<int, int> FEEDFORWARD_2ND_GAIN = {
  88, 2};  // Feedforward 2nd Gain, Read/Write, Default: 0
constexpr std::pair<int, int> FEEDFORWARD_1ST_GAIN = {
  90, 2};  // Feedforward 1st Gain, Read/Write, Default: 0
constexpr std::pair<int, int> BUS_WATCHDOG = {98, 1};    // BUS Watchdog, Read/Write, Default: 0
constexpr std::pair<int, int> GOAL_PWM = {100, 2};       // Goal PWM, Read/Write, Default: -
constexpr std::pair<int, int> GOAL_CURRENT = {102, 2};   // Goal Current, Read/Write, Default: -
constexpr std::pair<int, int> GOAL_VELOCITY = {104, 4};  // Goal Velocity, Read/Write, Default: -
constexpr std::pair<int, int> PROFILE_ACCELERATION = {
  108, 4};  // Profile Acceleration, Read/Write, Default: 0
constexpr std::pair<int, int> PROFILE_VELOCITY = {
  112, 4};                                               // Profile Velocity, Read/Write, Default: 0
constexpr std::pair<int, int> GOAL_POSITION = {116, 4};  // Goal Position, Read/Write, Default: -
constexpr std::pair<int, int> REALTIME_TICK = {120, 2};  // Realtime Tick, Read, Default: -
constexpr std::pair<int, int> MOVING = {122, 1};         // Moving, Read, Default: 0
constexpr std::pair<int, int> MOVING_STATUS = {123, 1};  // Moving Status, Read, Default: 0
constexpr std::pair<int, int> PRESENT_PWM = {124, 2};    // Present PWM, Read, Default: -
constexpr std::pair<int, int> PRESENT_CURRENT = {126, 2};   // Present Current, Read, Default: -
constexpr std::pair<int, int> PRESENT_VELOCITY = {128, 4};  // Present Velocity, Read, Default: -
constexpr std::pair<int, int> PRESENT_POSITION = {132, 4};  // Present Position, Read, Default: -
constexpr std::pair<int, int> VELOCITY_TRAJECTORY = {
  136, 4};  // Velocity Trajectory, Read, Default: -
constexpr std::pair<int, int> POSITION_TRAJECTORY = {
  140, 4};  // Position Trajectory, Read, Default: -
constexpr std::pair<int, int> PRESENT_INPUT_VOLTAGE = {
  144, 2};  // Present Input Voltage, Read, Default: -
constexpr std::pair<int, int> PRESENT_TEMPERATURE = {
  146, 1};  // Present Temperature, Read, Default: -
};          // namespace MXRAM

// Dynamixel PRO Series RAM Area
namespace PRORAM
{
constexpr std::pair<int, int> TORQUE_ENABLE = {512, 1};  // Torque Enable, Read/Write, Default: 0
constexpr std::pair<int, int> LED_R = {513, 1};          // LED Red, Read/Write, Default: 0
constexpr std::pair<int, int> LED_G = {514, 1};          // LED Green, Read/Write, Default: 0
constexpr std::pair<int, int> LED_B = {515, 1};          // LED Blue, Read/Write, Default: 0
constexpr std::pair<int, int> STATUS_RETURN_LEVEL = {
  516, 1};  // Status Return Level, Read/Write, Default: 2
constexpr std::pair<int, int> REGISTERED_INSTRUCTION = {
  517, 1};  // Registered Instruction, Read, Default: 0
constexpr std::pair<int, int> HARDWARE_ERROR_STATUS = {
  518, 1};  // Hardware Error Status, Read, Default: 0
constexpr std::pair<int, int> VELOCITY_I_GAIN = {
  524, 2};  // Velocity I Gain, Read/Write, Default: 1920
constexpr std::pair<int, int> VELOCITY_P_GAIN = {
  526, 2};  // Velocity P Gain, Read/Write, Default: 100
constexpr std::pair<int, int> POSITION_D_GAIN = {
  528, 2};  // Position D Gain, Read/Write, Default: 0
constexpr std::pair<int, int> POSITION_I_GAIN = {
  530, 2};  // Position I Gain, Read/Write, Default: 0
constexpr std::pair<int, int> POSITION_P_GAIN = {
  532, 2};  // Position P Gain, Read/Write, Default: 850
constexpr std::pair<int, int> FEEDFORWARD_2ND_GAIN = {
  536, 2};  // Feedforward 2nd Gain, Read/Write, Default: 0
constexpr std::pair<int, int> FEEDFORWARD_1ST_GAIN = {
  538, 2};  // Feedforward 1st Gain, Read/Write, Default: 0
constexpr std::pair<int, int> BUS_WATCHDOG = {546, 1};   // BUS Watchdog, Read/Write, Default: 0
constexpr std::pair<int, int> GOAL_PWM = {548, 2};       // Goal PWM, Read/Write, Default: -
constexpr std::pair<int, int> GOAL_CURRENT = {550, 2};   // Goal Current, Read/Write, Default: -
constexpr std::pair<int, int> GOAL_VELOCITY = {552, 4};  // Goal Velocity, Read/Write, Default: -
constexpr std::pair<int, int> PROFILE_ACCELERATION = {
  556, 4};  // Profile Acceleration, Read/Write, Default: 0
constexpr std::pair<int, int> PROFILE_VELOCITY = {
  560, 4};                                               // Profile Velocity, Read/Write, Default: 0
constexpr std::pair<int, int> GOAL_POSITION = {564, 4};  // Goal Position, Read/Write, Default: -
constexpr std::pair<int, int> REALTIME_TICK = {568, 2};  // Realtime Tick, Read, Default: -
constexpr std::pair<int, int> MOVING = {570, 1};         // Moving, Read, Default: 0
constexpr std::pair<int, int> MOVING_STATUS = {571, 1};  // Moving Status, Read, Default: 0
constexpr std::pair<int, int> PRESENT_PWM = {572, 2};    // Present PWM, Read, Default: -
constexpr std::pair<int, int> PRESENT_CURRENT = {574, 2};   // Present Current, Read, Default: -
constexpr std::pair<int, int> PRESENT_VELOCITY = {576, 4};  // Present Velocity, Read, Default: -
constexpr std::pair<int, int> PRESENT_POSITION = {580, 4};  // Present Position, Read, Default: -
constexpr std::pair<int, int> VELOCITY_TRAJECTORY = {
  584, 4};  // Velocity Trajectory, Read, Default: -
constexpr std::pair<int, int> POSITION_TRAJECTORY = {
  588, 4};  // Position Trajectory, Read, Default: -
constexpr std::pair<int, int> PRESENT_INPUT_VOLTAGE = {
  592, 2};  // Present Input Voltage, Read, Default: -
constexpr std::pair<int, int> PRESENT_TEMPERATURE = {
  594, 1};  // Present Temperature, Read, Default: -
constexpr std::pair<int, int> BACKUP_READY = {
  878, 1};  // Control data Backup Ready, Read, Default: 0
};          // namespace PRORAM
}  // namespace dynamixel_rdk_ros

#endif  // DYNAMIXEL_RDK_ROS_CONTROL_TABLE_HPP_