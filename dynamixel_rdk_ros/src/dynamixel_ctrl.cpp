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

#include "../include/dynamixel_rdk_ros/dynamixel_ctrl.hpp"

namespace dynamixel_rdk_ros
{

DynamixelCtrl::DynamixelCtrl(
  std::string device_port, int baud_rate, std::vector<uint8_t> dynamixel_ids,
  std::vector<DynamixelType> dynamixel_types, std::vector<double> max_position_limit,
  std::vector<double> min_position_limit)
: device_port(device_port),
  baud_rate(baud_rate),
  dynamixel_ids(dynamixel_ids),
  dynamixel_types(dynamixel_types),
  max_position_limit(max_position_limit),
  min_position_limit(min_position_limit)
{
  if (!init_dynamixel_sdk()) {
    std::cerr << "Failed to initialize Dynamixel SDK. Exiting..." << std::endl;
    throw std::runtime_error("[DynamixelCtrl] Dynamixel SDK initialization failed");
  }
  if (!init_max_velocity_limit()) {
    std::cerr << "Failed to initialize max velocity limit. Exiting..." << std::endl;
    throw std::runtime_error("[DynamixelCtrl] Max velocity limit initialization failed");
  }
  if (!init_indirect_address()) {
    std::cerr << "Failed to initialize indirect address. Exiting..." << std::endl;
    throw std::runtime_error("[DynamixelCtrl] Indirect address initialization failed");
  }
  std::cout << "========================================================" << std::endl;
  controller_status = true;
}

DynamixelCtrl::~DynamixelCtrl()
{
  portHandler_->closePort();
  delete portHandler_;
  delete packetHandler_;
}

bool DynamixelCtrl::init_dynamixel_sdk()
{
  std::cout << "========================================================" << std::endl;
  std::cout << "Initializing Dynamixel SDK" << std::endl;
  std::cout << "======================Configuration=====================" << std::endl;
  std::cout << "Device Port: " << device_port << std::endl;
  std::cout << "Baud Rate: " << baud_rate << std::endl;
  if (dynamixel_ids.size() != dynamixel_types.size()) {
    std::cerr << "Dynamixel ID and Type size mismatch" << std::endl;
    return false;
  }
  std::cout << "Dynamixel ID Count: " << dynamixel_ids.size() << std::endl;
  for (size_t i = 0; i < dynamixel_ids.size(); i++) {
    dynamixels.push_back(std::make_shared<Dynamixel>(
      dynamixel_types[i], dynamixel_ids[i], min_position_limit[i], max_position_limit[i]));
  }
  std::cout << "======================Port Handler=====================" << std::endl;

  portHandler_ = dynamixel::PortHandler::getPortHandler(device_port.c_str());
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

  std::cout << "Opening port: " << device_port << std::endl;
  if (!portHandler_->openPort()) {
    std::cerr << "Failed to open port" << std::endl;
    return false;
  }
  std::cout << "Port opened. Device Name: " << device_port << std::endl;

  std::cout << "Setting baudrate" << std::endl;
  if (!portHandler_->setBaudRate(baud_rate)) {
    std::cerr << "Failed to set baudrate" << std::endl;
    return false;
  }
  std::cout << "Baudrate set to: " << baud_rate << std::endl;

  return true;
}

bool DynamixelCtrl::init_max_velocity_limit()
{
  dynamixel::GroupSyncRead SyncRead(
    portHandler_, packetHandler_, EEPROM::VELOCITY_LIMIT.first, EEPROM::VELOCITY_LIMIT.second);

  std::cout << "======================Initializing=====================" << std::endl;

  for (auto & dynamixel : dynamixels) {
    bool dxl_addparam_result = SyncRead.addParam(dynamixel->get_id());
    if (!dxl_addparam_result) {
      std::cerr << "Failed to add parameter to sync read on Dynamixel ID: " << dynamixel->get_id()
                << std::endl;
      return false;
    }
  }

  int dxl_comm_result = SyncRead.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to get max velocity limit. Error Code: " << dxl_comm_result << std::endl;
    return false;
  } else {
    for (auto & dynamixel : dynamixels) {
      bool dxl_max_velocity_limit_result = dynamixel->get_max_velocity_limit(SyncRead);
      if (!dxl_max_velocity_limit_result) {
        std::cerr << "Failed to get max velocity limit on Dynamixel ID: " << dynamixel->get_id()
                  << std::endl;
        return false;
      } else {
        std::cout << "Successfully got max velocity limit on Dynamixel ID: " << dynamixel->get_id()
                  << " Max Velocity Limit: " << dynamixel->max_velocity_limit << std::endl;
      }
    }
  }
  return true;
}

bool DynamixelCtrl::init_single_max_velocity_limit(uint8_t id)
{
  dynamixel::GroupSyncRead SyncRead(
    portHandler_, packetHandler_, EEPROM::VELOCITY_LIMIT.first, EEPROM::VELOCITY_LIMIT.second);

  std::cout << "======================Initializing=====================" << std::endl;

  bool dxl_addparam_result = SyncRead.addParam(id);
  if (!dxl_addparam_result) {
    std::cerr << "Failed to add parameter to sync read on Dynamixel ID: " << id << std::endl;
    return false;
  }

  int dxl_comm_result = SyncRead.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to get max velocity limit. Error Code: " << dxl_comm_result << std::endl;
    return false;
  } else {
    bool dxl_max_velocity_limit_result = dynamixels[id]->get_max_velocity_limit(SyncRead);
    if (!dxl_max_velocity_limit_result) {
      std::cerr << "Failed to get max velocity limit on Dynamixel ID: " << id << std::endl;
      return false;
    } else {
      std::cout << "Successfully got max velocity limit on Dynamixel ID: " << id
                << " Max Velocity Limit: " << dynamixels[id]->max_velocity_limit << std::endl;
    }
  }
  return true;
}

bool DynamixelCtrl::init_indirect_address()
{
  dynamixel::GroupBulkWrite BulkWrite(portHandler_, packetHandler_);
  for (auto & dynamixel : dynamixels) {
    bool dxl_set_indirect_address_result = dynamixel->set_indirect_address(BulkWrite);
    if (!dxl_set_indirect_address_result) {
      std::cerr << "Failed to set indirect address on Dynamixel ID: " << dynamixel->get_id()
                << std::endl;
      return false;
    } else {
      std::cout << "Successfully set indirect address on Dynamixel ID: " << dynamixel->get_id()
                << std::endl;
    }
  }

  int dxl_comm_result = BulkWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to bulk write" << std::endl;
    return false;
  }
  return true;
}

bool DynamixelCtrl::init_single_indirect_address(uint8_t id)
{
  dynamixel::GroupBulkWrite BulkWrite(portHandler_, packetHandler_);
  bool dxl_set_indirect_address_result = dynamixels[id]->set_indirect_address(BulkWrite);
  if (!dxl_set_indirect_address_result) {
    std::cerr << "Failed to set indirect address on Dynamixel ID: " << id << std::endl;
    return false;
  } else {
    std::cout << "Successfully set indirect address on Dynamixel ID: " << id << std::endl;
  }

  int dxl_comm_result = BulkWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to bulk write" << std::endl;
    return false;
  }
  return true;
}

bool DynamixelCtrl::set_torque(bool torque)
{
  dynamixel::GroupSyncWrite SyncWrite(
    portHandler_, packetHandler_, Dynamixel::address(AddressNumber::TORQUE_ENABLE), 1);

  for (auto & dynamixel : dynamixels) {
    bool dxl_addparam_result = dynamixel->set_torque(SyncWrite, torque);
    if (!dxl_addparam_result) {
      throw std::runtime_error(
        "[DynamixelCtrl] Failed to add parameter to sync write on Dynamixel ID: " +
        std::to_string(dynamixel->get_id()));
    }
  }

  int dxl_comm_result = SyncWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
  } else {
    throw std::runtime_error("[DynamixelCtrl] Failed to set torque");
  }
  return true;
}

bool DynamixelCtrl::set_single_torque(uint8_t id, bool torque)
{
  dynamixel::GroupSyncWrite SyncWrite(
    portHandler_, packetHandler_, Dynamixel::address(AddressNumber::TORQUE_ENABLE), 1);

  bool dxl_addparam_result = dynamixels[id]->set_torque(SyncWrite, torque);
  if (!dxl_addparam_result) {
    throw std::runtime_error(
      "[DynamixelCtrl] Failed to add parameter to sync write on Dynamixel ID: " +
      std::to_string(id));
  }

  int dxl_comm_result = SyncWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
  } else {
    throw std::runtime_error("[DynamixelCtrl] Failed to set torque");
  }
  return true;
}

bool DynamixelCtrl::read_dynamixel_status()
{
  dynamixel::GroupSyncRead SyncRead(
    portHandler_, packetHandler_, Dynamixel::address(AddressNumber::TORQUE_ENABLE), READ_LENGTH);

  for (auto & dynamixel : dynamixels) {
    bool dxl_addparam_result = SyncRead.addParam(dynamixel->get_id());
    if (!dxl_addparam_result) {
      throw std::runtime_error(
        "[DynamixelCtrl] Failed to add parameter to sync read on Dynamixel ID: " +
        std::to_string(dynamixel->get_id()));
    }
  }

  int dxl_comm_result = SyncRead.txRxPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    for (auto & dynamixel : dynamixels) {
      bool dxl_get_dynamixel_status_result = dynamixel->get_dynamixel_status(SyncRead);
      if (!dxl_get_dynamixel_status_result) {
        throw std::runtime_error(
          "[DynamixelCtrl] Failed to get dynamixel status on Dynamixel ID: " +
          std::to_string(dynamixel->get_id()));
      }
    }
  } else {
    throw std::runtime_error("[DynamixelCtrl] Failed to get dynamixel status");
  }
  return true;
}

void DynamixelCtrl::sync_write(
  std::vector<double> position, std::vector<double> velocity, std::vector<double> acceleration)
{
  dynamixel::GroupSyncWrite SyncWrite(
    portHandler_, packetHandler_, Dynamixel::address(AddressNumber::PROFILE_ACCELERATION), 12);

  for (size_t i = 0; i < dynamixels.size(); i++) {
    dynamixels[i]->set_control_data(position[i], velocity[i], acceleration[i]);
    bool dxl_addparam_result = dynamixels[i]->set_control_data_param(SyncWrite);
    if (!dxl_addparam_result) {
      throw std::runtime_error(
        "[DynamixelCtrl] Failed to add parameter to sync write on Dynamixel ID: " +
        std::to_string(dynamixels[i]->get_id()));
    }
  }

  int dxl_comm_result = SyncWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
  } else {
    throw std::runtime_error("[DynamixelCtrl] Failed to set goal position");
  }
}

bool DynamixelCtrl::status()
{
  return controller_status;
}

}  // namespace dynamixel_rdk_ros
