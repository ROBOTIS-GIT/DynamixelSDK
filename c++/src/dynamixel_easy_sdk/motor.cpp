// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Hyungyu Kim

#include "dynamixel_easy_sdk/motor.hpp"
#include "dynamixel_easy_sdk/connector.hpp"

namespace dynamixel
{
Motor::Motor(uint8_t id, uint16_t model_number, Connector * connector)
: id_(id),
  model_number_(model_number),
  model_name_(ControlTable::getModelName(model_number)),
  connector_(connector),
  control_table_(ControlTable::getControlTable(model_number))
{
  Result<uint8_t, DxlError> torque_result = this->isTorqueOn();
  if (!torque_result.isSuccess()) {
    throw DxlRuntimeError("Failed to get torque status: " + getErrorMessage(torque_result.error()));
  }
  torque_status_ = torque_result.value();

  Result<OperatingMode, DxlError> mode_result = this->getOperatingMode();
  if (!mode_result.isSuccess()) {
    throw DxlRuntimeError("Failed to get operating mode: " + getErrorMessage(mode_result.error()));
  }
  operating_mode_status_ = mode_result.value();
}

Motor::~Motor() {}

Result<void, DxlError> Motor::enableTorque()
{
  Result<void, DxlError> result = writeData(id_, "Torque Enable", 1);
  if (result.isSuccess()) {
    torque_status_ = 1;
  }
  return result;
}

Result<void, DxlError> Motor::disableTorque()
{
  Result<void, DxlError> result = writeData(id_, "Torque Enable", 0);
  if (result.isSuccess()) {
    torque_status_ = 0;
  }
  return result;
}

Result<void, DxlError> Motor::setGoalPosition(int32_t position)
{
  if (torque_status_ == 0) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }
  if (operating_mode_status_ != OperatingMode::POSITION &&
    operating_mode_status_ != OperatingMode::EXTENDED_POSITION &&
    operating_mode_status_ != OperatingMode::CURRENT_BASED_POSITION)
  {
    return DxlError::EASY_SDK_OPERATING_MODE_MISMATCH;
  }
  Result<void, DxlError> result = writeData(id_, "Goal Position", static_cast<uint32_t>(position));
  return result;
}

Result<void, DxlError> Motor::setGoalVelocity(int32_t velocity)
{
  if (torque_status_ == 0) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }
  if (operating_mode_status_ != OperatingMode::VELOCITY) {
    return DxlError::EASY_SDK_OPERATING_MODE_MISMATCH;
  }
  Result<void, DxlError> result = writeData(id_, "Goal Velocity", static_cast<uint32_t>(velocity));
  return result;
}

Result<void, DxlError> Motor::setGoalCurrent(int16_t current)
{
  if (torque_status_ == 0) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }
  if (operating_mode_status_ != OperatingMode::CURRENT &&
    operating_mode_status_ != OperatingMode::CURRENT_BASED_POSITION) {
    return DxlError::EASY_SDK_OPERATING_MODE_MISMATCH;
  }
  Result<void, DxlError> result = writeData(id_, "Goal Current", static_cast<uint16_t>(current));
  return result;
}

Result<void, DxlError> Motor::setGoalPWM(int16_t pwm)
{
  if (torque_status_ == 0) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }
  Result<void, DxlError> result = writeData(id_, "Goal PWM", static_cast<uint16_t>(pwm));
  return result;
}

Result<void, DxlError> Motor::LEDOn()
{
  Result<void, DxlError> result = writeData(id_, "LED", 1);
  return result;
}

Result<void, DxlError> Motor::LEDOff()
{
  Result<void, DxlError> result = writeData(id_, "LED", 0);
  return result;
}

Result<uint16_t, DxlError> Motor::ping()
{
  Result<uint16_t, DxlError> result = connector_->ping(id_);
  return result;
}

Result<uint8_t, DxlError> Motor::isTorqueOn()
{
  Result<uint32_t, DxlError> result = readData(id_, "Torque Enable");
  if (!result.isSuccess()) {
    return result.error();
  }
  torque_status_ = static_cast<uint8_t>(result.value());
  return torque_status_;
}

Result<uint8_t, DxlError> Motor::isLEDOn()
{
  Result<uint32_t, DxlError> result = readData(id_, "LED");
  if (!result.isSuccess()) {
    return result.error();
  }
  return static_cast<uint8_t>(result.value());
}

Result<int32_t, DxlError> Motor::getPresentPosition()
{
  Result<uint32_t, DxlError> result = readData(id_, "Present Position");
  if (!result.isSuccess()) {
    return result.error();
  }
  return static_cast<int32_t>(result.value());
}

Result<int32_t, DxlError> Motor::getPresentVelocity()
{
  Result<uint32_t, DxlError> result = readData(id_, "Present Velocity");
  if (!result.isSuccess()) {
    return result.error();
  }
  return static_cast<int32_t>(result.value());
}

Result<int16_t, DxlError> Motor::getPresentCurrent()
{
  Result<uint32_t, DxlError> result = readData(id_, "Present Current");
  if (!result.isSuccess()) {
    return result.error();
  }
  return static_cast<int16_t>(result.value());
}

Result<int16_t, DxlError> Motor::getPresentPWM()
{
  Result<uint32_t, DxlError> result = readData(id_, "Present PWM");
  if (!result.isSuccess()) {
    return result.error();
  }
  return static_cast<int16_t>(result.value());
}

Result<uint32_t, DxlError> Motor::getMaxPositionLimit()
{
  Result<uint32_t, DxlError> result = readData(id_, "Max Position Limit");
  return result;
}

Result<uint32_t, DxlError> Motor::getMinPositionLimit()
{
  Result<uint32_t, DxlError> result = readData(id_, "Min Position Limit");
  return result;
}

Result<uint32_t, DxlError> Motor::getVelocityLimit()
{
  Result<uint32_t, DxlError> result = readData(id_, "Velocity Limit");
  return result;
}

Result<uint16_t, DxlError> Motor::getCurrentLimit()
{
  Result<uint32_t, DxlError> result = readData(id_, "Current Limit");
  if (!result.isSuccess()) {
    return result.error();
  }
  return static_cast<uint16_t>(result.value());
}

Result<uint16_t, DxlError> Motor::getPWMLimit()
{
  Result<uint32_t, DxlError> result = readData(id_, "PWM Limit");
  if (!result.isSuccess()) {
    return result.error();
  }
  return static_cast<uint16_t>(result.value());
}

Result<OperatingMode, DxlError> Motor::getOperatingMode()
{
  Result<uint32_t, DxlError> result = readData(id_, "Operating Mode");
  if (!result.isSuccess()) {
    return result.error();
  }
  return static_cast<OperatingMode>(result.value());
}

Result<void, DxlError> Motor::changeID(uint8_t new_id)
{
  Result<void, DxlError> result = writeData(id_, "ID", new_id);
  if (result.isSuccess()) {
    id_ = new_id;
  }
  return result;
}

Result<void, DxlError> Motor::setOperatingMode(OperatingMode mode)
{
  if (torque_status_ == 1) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }
  uint8_t mode_value = static_cast<uint8_t>(mode);
  Result<void, DxlError> result = writeData(id_, "Operating Mode", mode_value);
  if (result.isSuccess()) {
    operating_mode_status_ = mode;
  }
  return result;
}

Result<void, DxlError> Motor::setProfileConfiguration(ProfileConfiguration config)
{
  if (torque_status_ == 1) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }

  Result<uint32_t, DxlError> read_result = readData(id_, "Drive Mode");
  if (!read_result.isSuccess()) {
    return read_result.error();
  }

  uint8_t drive_mode = static_cast<uint8_t>(read_result.value());
  const uint8_t PROFILE_BIT_MASK = 0b00000100;

  if (config == ProfileConfiguration::TIME_BASED) {
    drive_mode |= PROFILE_BIT_MASK;
  } else if (config == ProfileConfiguration::VELOCITY_BASED) {
    drive_mode &= ~PROFILE_BIT_MASK;
  }
  Result<void, DxlError> result = writeData(id_, "Drive Mode", drive_mode);
  return result;
}

Result<void, DxlError> Motor::setDirection(Direction direction)
{
  if (torque_status_ == 1) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }

  Result<uint32_t, DxlError> read_result = readData(id_, "Drive Mode");
  if (!read_result.isSuccess()) {
    return read_result.error();
  }
  uint8_t drive_mode = static_cast<uint8_t>(read_result.value());
  const uint8_t DIRECTION_BIT_MASK = 0b00000001;
  if (direction == Direction::NORMAL) {
    drive_mode &= ~DIRECTION_BIT_MASK;
  } else if (direction == Direction::REVERSE) {
    drive_mode |= DIRECTION_BIT_MASK;
  }
  Result<void, DxlError> result = writeData(id_, "Drive Mode", drive_mode);
  return result;
}

Result<void, DxlError> Motor::setPositionPGain(uint16_t p_gain)
{
  if (torque_status_ == 1) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }

  Result<void, DxlError> result = writeData(id_, "Position P Gain", p_gain);
  return result;
}

Result<void, DxlError> Motor::setPositionIGain(uint16_t i_gain)
{
  if (torque_status_ == 1) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }

  Result<void, DxlError> result = writeData(id_, "Position I Gain", i_gain);
  return result;
}

Result<void, DxlError> Motor::setPositionDGain(uint16_t d_gain)
{
  if (torque_status_ == 1) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }

  Result<void, DxlError> result = writeData(id_, "Position D Gain", d_gain);
  return result;
}

Result<void, DxlError> Motor::setVelocityPGain(uint16_t p_gain)
{
  if (torque_status_ == 1) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }

  Result<void, DxlError> result = writeData(id_, "Velocity P Gain", p_gain);
  return result;
}

Result<void, DxlError> Motor::setVelocityIGain(uint16_t i_gain)
{
  if (torque_status_ == 1) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }

  Result<void, DxlError> result = writeData(id_, "Velocity I Gain", i_gain);
  return result;
}

Result<void, DxlError> Motor::setHomingOffset(int32_t offset)
{
  if (torque_status_ == 1) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }
  Result<void, DxlError> result = writeData(
    id_,
    "Homing Offset",
    static_cast<uint32_t>(offset));
  return result;
}

Result<void, DxlError> Motor::setMaxPositionLimit(uint32_t limit)
{
  if (torque_status_ == 1) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }

  Result<void, DxlError> result = writeData(id_, "Max Position Limit", limit);
  return result;
}

Result<void, DxlError> Motor::setMinPositionLimit(uint32_t limit)
{
  if (torque_status_ == 1) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }

  Result<void, DxlError> result = writeData(id_, "Min Position Limit", limit);
  return result;
}

Result<void, DxlError> Motor::setVelocityLimit(uint32_t limit)
{
  if (torque_status_ == 1) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }

  Result<void, DxlError> result = writeData(id_, "Velocity Limit", limit);
  return result;
}

Result<void, DxlError> Motor::setCurrentLimit(uint16_t limit)
{
  if (torque_status_ == 1) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }

  Result<void, DxlError> result = writeData(id_, "Current Limit", limit);
  return result;
}

Result<void, DxlError> Motor::setPWMLimit(uint16_t limit)
{
  if (torque_status_ == 1) {
    return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
  }

  Result<void, DxlError> result = writeData(id_, "PWM Limit", limit);
  return result;
}

Result<void, DxlError> Motor::reboot()
{
  Result<void, DxlError> result = connector_->reboot(id_);
  return result;
}

Result<void, DxlError> Motor::factoryResetAll()
{
  Result<void, DxlError> result = connector_->factoryReset(id_, 0xFF);
  return result;
}

Result<void, DxlError> Motor::factoryResetExceptID()
{
  Result<void, DxlError> result = connector_->factoryReset(id_, 0x01);
  return result;
}

Result<void, DxlError> Motor::factoryResetExceptIDAndBaudRate()
{
  Result<void, DxlError> result = connector_->factoryReset(id_, 0x02);
  return result;
}

Result<StagedCommand, DxlError> Motor::stageEnableTorque()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Torque Enable");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  StagedCommand cmd(
    CommandType::WRITE,
    id_,
    item_result.value().address,
    item_result.value().size,
    {1},
    {StatusRequest::UPDATE_TORQUE_STATUS},
    this
  );
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageDisableTorque()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Torque Enable");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  StagedCommand cmd(
    CommandType::WRITE,
    id_,
    item_result.value().address,
    item_result.value().size,
    {0},
    {StatusRequest::UPDATE_TORQUE_STATUS},
    this
  );
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageSetGoalPosition(int32_t position)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Goal Position");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  std::vector<uint8_t> data;
  for (size_t i = 0; i < item_result.value().size; ++i) {
    data.push_back((position >> (8 * i)) & 0xFF);
  }
  StagedCommand cmd(
    CommandType::WRITE,
    id_,
    item_result.value().address,
    item_result.value().size,
    data,
    {StatusRequest::UPDATE_TORQUE_STATUS, StatusRequest::CHECK_OPERATING_MODE},
    this,
    {OperatingMode::POSITION, OperatingMode::EXTENDED_POSITION, OperatingMode::CURRENT_BASED_POSITION}
  );
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageSetGoalVelocity(int32_t velocity)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Goal Velocity");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  std::vector<uint8_t> data;
  for (size_t i = 0; i < item_result.value().size; ++i) {
    data.push_back((velocity >> (8 * i)) & 0xFF);
  }
  StagedCommand cmd(
    CommandType::WRITE,
    id_,
    item_result.value().address,
    item_result.value().size,
    data,
    {StatusRequest::UPDATE_TORQUE_STATUS, StatusRequest::CHECK_OPERATING_MODE},
    this,
    {OperatingMode::VELOCITY}
  );
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageSetGoalCurrent(int16_t current)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Goal Current");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  std::vector<uint8_t> data;
  for (size_t i = 0; i < item_result.value().size; ++i) {
    data.push_back((current >> (8 * i)) & 0xFF);
  }
  StagedCommand cmd(
    CommandType::WRITE,
    id_,
    item_result.value().address,
    item_result.value().size,
    data,
    {StatusRequest::UPDATE_TORQUE_STATUS, StatusRequest::CHECK_OPERATING_MODE},
    this,
    {OperatingMode::CURRENT, OperatingMode::CURRENT_BASED_POSITION}
  );
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageSetGoalPWM(int16_t pwm)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Goal PWM");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  std::vector<uint8_t> data;
  for (size_t i = 0; i < item_result.value().size; ++i) {
    data.push_back((pwm >> (8 * i)) & 0xFF);
  }
  StagedCommand cmd(
    CommandType::WRITE,
    id_,
    item_result.value().address,
    item_result.value().size,
    data,
    {StatusRequest::UPDATE_TORQUE_STATUS},
    this
  );
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageLEDOn()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("LED");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  StagedCommand cmd(
    CommandType::WRITE,
    id_,
    item_result.value().address,
    item_result.value().size,
    {1}
  );
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageLEDOff()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("LED");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  StagedCommand cmd(
    CommandType::WRITE,
    id_,
    item_result.value().address,
    item_result.value().size,
    {0}
  );
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageIsTorqueOn()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Torque Enable");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  StagedCommand cmd(
    CommandType::READ,
    id_,
    item_result.value().address,
    item_result.value().size,
    {},
    {StatusRequest::UPDATE_TORQUE_STATUS},
    this
  );
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageIsLEDOn()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("LED");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  StagedCommand cmd(
    CommandType::READ,
    id_,
    item_result.value().address,
    item_result.value().size,
    {}
  );
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageGetPresentPosition()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Present Position");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  StagedCommand cmd(
    CommandType::READ,
    id_,
    item_result.value().address,
    item_result.value().size,
    {}
  );
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageGetPresentVelocity()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Present Velocity");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  StagedCommand cmd(
    CommandType::READ,
    id_,
    item_result.value().address,
    item_result.value().size,
    {}
  );
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageGetPresentCurrent()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Present Current");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  StagedCommand cmd(
    CommandType::READ,
    id_,
    item_result.value().address,
    item_result.value().size,
    {}
  );
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageGetPresentPWM()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Present PWM");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  StagedCommand cmd(
    CommandType::READ,
    id_,
    item_result.value().address,
    item_result.value().size,
    {}
  );
  return cmd;
}

Result<ControlTableItem, DxlError> Motor::getControlTableItem(const std::string & item_name)
{
  auto it = control_table_.find(item_name);
  if (it == control_table_.end()) {
    return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
  }
  return it->second;
}

Result<void, DxlError> Motor::writeData(uint8_t id, const std::string & item_name, uint32_t value)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem(item_name);
  if (!item_result.isSuccess()) {
    return item_result.error();
  }

  const ControlTableItem & item = item_result.value();
  uint16_t address = item.address;
  uint8_t size = item.size;

  switch (size) {
    case 1:
      return connector_->write1ByteData(id, address, static_cast<uint8_t>(value));
    case 2:
      return connector_->write2ByteData(id, address, static_cast<uint16_t>(value));
    case 4:
      return connector_->write4ByteData(id, address, value);
  }
  return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
}

Result<uint32_t, DxlError> Motor::readData(uint8_t id, const std::string & item_name)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem(item_name);
  if (!item_result.isSuccess()) {
    return item_result.error();
  }

  const ControlTableItem & item = item_result.value();
  uint16_t address = item.address;
  uint8_t size = item.size;

  switch (size) {
    case 1: {
        Result<uint8_t, DxlError> result = connector_->read1ByteData(id, address);
        if (!result.isSuccess()) {
          return result.error();
        }
        return static_cast<uint32_t>(result.value());
      }
    case 2: {
        Result<uint16_t, DxlError> result = connector_->read2ByteData(id, address);
        if (!result.isSuccess()) {
          return result.error();
        }
        return static_cast<uint32_t>(result.value());
      }
    case 4: {
        Result<uint32_t, DxlError> result = connector_->read4ByteData(id, address);
        if (!result.isSuccess()) {
          return result.error();
        }
        return result.value();
      }
  }
  return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
}

}  // namespace dynamixel
