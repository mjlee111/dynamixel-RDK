#include "../include/dynamixel_rdk_ros/dynamixel.hpp"

namespace dynamixel_rdk_ros
{

std::vector<std::string> dynamixel_type_str = {"MX",       "PH54_200", "PH54_100", "PH42_020",
                                               "PM54_060", "PM54_040", "PM42_010"};

Dynamixel::Dynamixel(DynamixelType type, uint8_t id, double min_rad, double max_rad)
: type(type), id(id), min_rad(min_rad), max_rad(max_rad)
{
  dynamixel_addresses.clear();
  if (type == DynamixelType::MX) {
    dynamixel_indirect_address = MX_INDIRECT_ADDRESS_START;
    add_address(MXRAM::TORQUE_ENABLE);
    add_address(MXRAM::HARDWARE_ERROR_STATUS);
    add_address(MXRAM::PROFILE_ACCELERATION);
    add_address(MXRAM::PROFILE_VELOCITY);
    add_address(MXRAM::GOAL_POSITION);
    add_address(MXRAM::MOVING);
    add_address(MXRAM::PRESENT_CURRENT);
    add_address(MXRAM::PRESENT_VELOCITY);
    add_address(MXRAM::PRESENT_POSITION);
    add_address(MXRAM::PRESENT_INPUT_VOLTAGE);
    add_address(MXRAM::PRESENT_TEMPERATURE);
    dxl_current_ratio = MX_CURRENT_PROFILE;
    dxl_rps_ratio = MX_RPS_PROFILE * M_PI / 30;
    dxl_acc_ratio = MX_ACC_PROFILE * M_PI / 1800;
  } else if (
    type == DynamixelType::PH54_200 || type == DynamixelType::PH54_100 ||
    type == DynamixelType::PH42_020 || type == DynamixelType::PM54_060 ||
    type == DynamixelType::PM54_040 || type == DynamixelType::PM42_010) {
    dynamixel_indirect_address = PRO_INDIRECT_ADDRESS_START;
    add_address(PRORAM::TORQUE_ENABLE);
    add_address(PRORAM::HARDWARE_ERROR_STATUS);
    add_address(PRORAM::PROFILE_ACCELERATION);
    add_address(PRORAM::PROFILE_VELOCITY);
    add_address(PRORAM::GOAL_POSITION);
    add_address(PRORAM::MOVING);
    add_address(PRORAM::PRESENT_CURRENT);
    add_address(PRORAM::PRESENT_VELOCITY);
    add_address(PRORAM::PRESENT_POSITION);
    add_address(PRORAM::PRESENT_INPUT_VOLTAGE);
    add_address(PRORAM::PRESENT_TEMPERATURE);
    dxl_current_ratio = PRO_CURRENT_PROFILE;
    dxl_rps_ratio = PRO_RPS_PROFILE * M_PI / 30;
    dxl_acc_ratio = PRO_ACC_PROFILE * M_PI / 1800;
  } else {
    std::cerr << "Invalid Dynamixel Type" << std::endl;
    throw std::runtime_error("[Dynamixel] Invalid Dynamixel Type");
  }
}

Dynamixel::~Dynamixel()
{
  std::cout << "[Dynamixel] Finishing Dynamixel id : " << static_cast<int>(id) << std::endl;
}

bool Dynamixel::get_max_velocity_limit(dynamixel::GroupSyncRead & SyncRead)
{
  if (!SyncRead.isAvailable(id, EEPROM::VELOCITY_LIMIT.first, EEPROM::VELOCITY_LIMIT.second)) {
    return false;
  }
  max_velocity_limit =
    SyncRead.getData(id, EEPROM::VELOCITY_LIMIT.first, EEPROM::VELOCITY_LIMIT.second) *
    dxl_rps_ratio;
  return true;
}

bool Dynamixel::get_dynamixel_status(dynamixel::GroupSyncRead & SyncRead)
{
  if (!SyncRead.isAvailable(id, address(TORQUE_ENABLE), 1)) {
    reboot_seq_ = DynamixelRebootSequence::REBOOT_START;
    return false;
  }
  if (!SyncRead.isAvailable(id, address(MOVING), 1)) {
    reboot_seq_ = DynamixelRebootSequence::REBOOT_START;
    return false;
  }
  if (!SyncRead.isAvailable(id, address(HARDWARE_ERROR_STATUS), 1)) {
    reboot_seq_ = DynamixelRebootSequence::REBOOT_START;
    return false;
  }
  if (!SyncRead.isAvailable(id, address(PRESENT_POSITION), 4)) {
    reboot_seq_ = DynamixelRebootSequence::REBOOT_START;
    return false;
  }
  if (!SyncRead.isAvailable(id, address(PRESENT_VELOCITY), 4)) {
    reboot_seq_ = DynamixelRebootSequence::REBOOT_START;
    return false;
  }
  if (!SyncRead.isAvailable(id, address(PRESENT_CURRENT), 2)) {
    reboot_seq_ = DynamixelRebootSequence::REBOOT_START;
    return false;
  }
  if (!SyncRead.isAvailable(id, address(PRESENT_INPUT_VOLTAGE), 2)) {
    reboot_seq_ = DynamixelRebootSequence::REBOOT_START;
    return false;
  }
  if (!SyncRead.isAvailable(id, address(PRESENT_TEMPERATURE), 1)) {
    reboot_seq_ = DynamixelRebootSequence::REBOOT_START;
    return false;
  }

  torque_enabled = SyncRead.getData(id, address(TORQUE_ENABLE), 1) & 0x01;
  is_moving = SyncRead.getData(id, address(MOVING), 1) & 0x01;
  error_status = SyncRead.getData(id, address(HARDWARE_ERROR_STATUS), 1);
  present_position = to_rad((int32_t)SyncRead.getData(id, address(PRESENT_POSITION), 4));
  present_velocity = (int32_t)SyncRead.getData(id, address(PRESENT_VELOCITY), 4) * dxl_rps_ratio;
  present_current = -(int16_t)SyncRead.getData(id, address(PRESENT_CURRENT), 2) / dxl_current_ratio;
  present_voltage = SyncRead.getData(id, address(PRESENT_INPUT_VOLTAGE), 2) * 0.1;
  present_temperature = (int16_t)SyncRead.getData(id, address(PRESENT_TEMPERATURE), 1);

  int error_status_int = static_cast<int>(error_status);
  if (error_status_int != 0 && reboot_seq_ == DynamixelRebootSequence::STABLE) {
    std::cerr << "[Dynamixel] Got error on ID: " << static_cast<int>(id)
              << " Error status: " << error_status_int << std::endl;
    reboot_seq_ = DynamixelRebootSequence::REBOOT_START;
    return false;
  }
  return true;
}

bool Dynamixel::set_indirect_address(dynamixel::GroupBulkWrite & BulkWrite)
{
  return BulkWrite.addParam(
    id, dynamixel_indirect_address, dynamixel_addresses.size(), dynamixel_addresses.data());
}

bool Dynamixel::set_single_indirect_address(
  dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler * port_handler)
{
  uint8_t dxl_initialize_err;
  int dxl_initialize_result = packet_handler->writeTxRx(
    port_handler, id, dynamixel_indirect_address, dynamixel_addresses.size(),
    dynamixel_addresses.data(), &dxl_initialize_err);

  if (dxl_initialize_result != COMM_SUCCESS) {
    std::cerr << "[Dynamixel] Failed to set indirect address. Error id: " << static_cast<int>(id)
              << std::endl;
    return false;
  }

  uint8_t dxl_max_velocity_err;
  uint32_t dxl_max_velocity_data;
  int dxl_max_velocity_result = packet_handler->read4ByteTxRx(
    port_handler, id, EEPROM::VELOCITY_LIMIT.first, &dxl_max_velocity_data, &dxl_max_velocity_err);
  if (dxl_max_velocity_result != COMM_SUCCESS) {
    std::cerr << "[Dynamixel] Failed to get max velocity limit. Error id: " << static_cast<int>(id)
              << std::endl;
    return false;
  }
  max_velocity_limit = dxl_max_velocity_data * dxl_rps_ratio;

  return true;
}

bool Dynamixel::set_torque(dynamixel::GroupSyncWrite & SyncWrite, bool torque)
{
  uint8_t torque_data[1] = {static_cast<uint8_t>(torque ? 1U : 0U)};
  return SyncWrite.addParam(id, torque_data);
}

bool Dynamixel::set_single_torque(
  dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler * port_handler, bool torque)
{
  uint8_t dxl_torque_err;
  uint8_t torque_data[1] = {static_cast<uint8_t>(torque ? 1U : 0U)};
  int dxl_torque_result = packet_handler->writeTxRx(
    port_handler, id, address(TORQUE_ENABLE), 1, torque_data, &dxl_torque_err);
  return dxl_torque_result == COMM_SUCCESS;
}

bool Dynamixel::set_position(double position)
{
  if (position < min_rad || position > max_rad) {
    return false;
  }
  goal_position = position;
  return true;
}

bool Dynamixel::set_velocity(double velocity)
{
  profile_velocity = velocity;
  return true;
}

bool Dynamixel::set_acceleration(double acceleration)
{
  profile_acceleration = acceleration;
  return true;
}

bool Dynamixel::set_control_data(double position, double velocity, double acceleration)
{
  return set_position(position) && set_velocity(velocity) && set_acceleration(acceleration);
}

bool Dynamixel::set_control_data_param(dynamixel::GroupSyncWrite & SyncWrite)
{
  std::vector<uint8_t> control_data_vector;
  int position_data, velocity_data, acceleration_data;

  if (profile_velocity >= max_velocity_limit || profile_velocity <= 0.0) {
    velocity_data = 0;
  } else {
    velocity_data = static_cast<int>(profile_velocity / dxl_rps_ratio);
    if (velocity_data < 0) {
      velocity_data = 1;
    }
  }

  if (profile_acceleration >= max_velocity_limit) {
    acceleration_data = 0;
  } else {
    acceleration_data = static_cast<int>(profile_acceleration / dxl_acc_ratio);
    if (acceleration_data < 0) {
      acceleration_data = 1;
    }
  }

  position_data = to_position(goal_position);

  divide_byte(control_data_vector, acceleration_data, 4);
  divide_byte(control_data_vector, velocity_data, 4);
  divide_byte(control_data_vector, position_data, 4);

  return SyncWrite.addParam(id, control_data_vector.data());
}

void Dynamixel::reboot_sequence(
  dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler * port_handler)
{
  uint8_t dxl_reboot_err;
  int dxl_reboot_result;

  std::cout << "Starting reboot sequence for ID: " << static_cast<int>(id)
            << " Reboot seq: " << static_cast<int>(reboot_seq_) << std::endl;

  try {
    switch (reboot_seq_) {
      case DynamixelRebootSequence::STABLE:
        break;
      case DynamixelRebootSequence::REBOOT_START:
        dxl_reboot_result = packet_handler->reboot(port_handler, id, &dxl_reboot_err);
        if (dxl_reboot_result != COMM_SUCCESS) {
          std::cout << "[Dynamixel] Reboot start failed for ID " << static_cast<int>(id)
                    << ", retrying..." << std::endl;
          reboot_seq_ = DynamixelRebootSequence::REBOOT_PING;
        } else {
          // std::cout << "[Dynamixel] Reboot start success for ID " << static_cast<int>(id)
          //           << std::endl;
          reboot_seq_ = DynamixelRebootSequence::REBOOT_PING;
        }
        break;
      case DynamixelRebootSequence::REBOOT_PING:
        dxl_reboot_result = packet_handler->ping(port_handler, id, &dxl_reboot_err);
        if (dxl_reboot_result != COMM_SUCCESS) {
          std::cout << "[Dynamixel] Ping failed for ID " << static_cast<int>(id) << ", retrying..."
                    << std::endl;
          reboot_seq_ = DynamixelRebootSequence::INITIALIZE;
        } else {
          // std::cout << "[Dynamixel] Ping success for ID " << static_cast<int>(id) << std::endl;
          reboot_seq_ = DynamixelRebootSequence::INITIALIZE;
        }
        break;
      case DynamixelRebootSequence::INITIALIZE:
        if (!set_single_indirect_address(packet_handler, port_handler)) {
          std::cout << "[Dynamixel] Initialize failed for ID " << static_cast<int>(id)
                    << ", retrying..." << std::endl;
          reboot_seq_ = DynamixelRebootSequence::SET_TORQUE;
        } else {
          // std::cout << "[Dynamixel] Initialize success for ID " << static_cast<int>(id)
          //           << std::endl;
          reboot_seq_ = DynamixelRebootSequence::SET_TORQUE;
        }
        break;
      case DynamixelRebootSequence::SET_TORQUE:
        if (!set_single_torque(packet_handler, port_handler, true)) {
          std::cout << "[Dynamixel] Set torque failed for ID " << static_cast<int>(id)
                    << ", retrying..." << std::endl;
        } else {
          reboot_seq_ = DynamixelRebootSequence::STABLE;
          std::cout << "[Dynamixel] Reboot sequence completed for ID " << static_cast<int>(id)
                    << std::endl;
        }
        break;
      default:
        std::cerr << "[Dynamixel] Invalid reboot sequence for ID " << static_cast<int>(id)
                  << std::endl;
        break;
    }
  } catch (const std::exception & e) {
    std::cerr << "[Dynamixel] Reboot sequence error for ID " << static_cast<int>(id) << ": "
              << e.what() << std::endl;
  }
}

void Dynamixel::divide_byte(std::vector<uint8_t> & data, int address, int byte_size)
{
  switch (byte_size) {
    case 1:
      data.push_back(address && 0xFF);
      break;
    case 2:
      data.push_back(DXL_LOBYTE(address));
      data.push_back(DXL_HIBYTE(address));
      break;
    case 4:
      data.push_back(DXL_LOBYTE(DXL_LOWORD(address)));
      data.push_back(DXL_HIBYTE(DXL_LOWORD(address)));
      data.push_back(DXL_LOBYTE(DXL_HIWORD(address)));
      data.push_back(DXL_HIBYTE(DXL_HIWORD(address)));
      break;
  }
}

void Dynamixel::add_address(const std::pair<int, int> & control_table)
{
  for (int i = 0; i < control_table.second; i++) {
    divide_byte(dynamixel_addresses, control_table.first + i, 2);
  }
}

uint16_t Dynamixel::address(const uint16_t address)
{
  return INDIRECT_DATA_START + address;
}

double Dynamixel::min_max_rad(int32_t min, int32_t max, int32_t position)
{
  return ((static_cast<double>(position) - min) / (max - min)) * (2 * M_PI) - M_PI;
}

double Dynamixel::to_rad(int32_t position)
{
  switch (type) {
    case DynamixelType::MX:
      return min_max_rad(0, MX_RESOLUTION, position);
    case DynamixelType::PH54_200:
      return min_max_rad(-PRO_PH54_RESOLUTION, PRO_PH54_RESOLUTION, position);
    case DynamixelType::PH54_100:
      return min_max_rad(-PRO_PH54_RESOLUTION, PRO_PH54_RESOLUTION, position);
    case DynamixelType::PH42_020:
      return min_max_rad(-PRO_PH42_RESOLUTION, PRO_PH42_RESOLUTION, position);
    case DynamixelType::PM54_060:
      return min_max_rad(-PRO_PM54_RESOLUTION, PRO_PM54_RESOLUTION, position);
    case DynamixelType::PM54_040:
      return min_max_rad(-PRO_PM54_RESOLUTION, PRO_PM54_RESOLUTION, position);
    case DynamixelType::PM42_010:
      return min_max_rad(-PRO_PM42_RESOLUTION, PRO_PM42_RESOLUTION, position);
    default:
      return 0.0;
  }
}

int32_t Dynamixel::min_max_position(int32_t min, int32_t max, double rad)
{
  return static_cast<int32_t>(min + ((rad + M_PI) / (2 * M_PI)) * (max - min));
}

int32_t Dynamixel::to_position(double rad)
{
  switch (type) {
    case DynamixelType::MX:
      return min_max_position(0, MX_RESOLUTION, rad);
    case DynamixelType::PH54_200:
      return min_max_position(-PRO_PH54_RESOLUTION, PRO_PH54_RESOLUTION, rad);
    case DynamixelType::PH54_100:
      return min_max_position(-PRO_PH54_RESOLUTION, PRO_PH54_RESOLUTION, rad);
    case DynamixelType::PH42_020:
      return min_max_position(-PRO_PH42_RESOLUTION, PRO_PH42_RESOLUTION, rad);
    case DynamixelType::PM54_060:
      return min_max_position(-PRO_PM54_RESOLUTION, PRO_PM54_RESOLUTION, rad);
    case DynamixelType::PM54_040:
      return min_max_position(-PRO_PM54_RESOLUTION, PRO_PM54_RESOLUTION, rad);
    case DynamixelType::PM42_010:
      return min_max_position(-PRO_PM42_RESOLUTION, PRO_PM42_RESOLUTION, rad);
    default:
      return 0;
  }
}

std::string Dynamixel::get_reboot_sequence_str() const
{
  return sequence_to_string(reboot_seq_);
}

std::string Dynamixel::sequence_to_string(DynamixelRebootSequence seq) const
{
  switch (seq) {
    case DynamixelRebootSequence::STABLE:
      return "STABLE";
    case DynamixelRebootSequence::REBOOT_START:
      return "REBOOT_START";
    case DynamixelRebootSequence::REBOOT_PING:
      return "REBOOT_PING";
    case DynamixelRebootSequence::INITIALIZE:
      return "INITIALIZE";
    case DynamixelRebootSequence::SET_TORQUE:
      return "SET_TORQUE";
    default:
      return "UNKNOWN";
  }
}

}  // namespace dynamixel_rdk_ros