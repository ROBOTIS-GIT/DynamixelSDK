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
  torque_ = torque_result.value();

  Result<uint8_t, DxlError> mode_result = this->getOperatingMode();
  if (!mode_result.isSuccess()) {
    throw DxlRuntimeError("Failed to get operating mode: " + getErrorMessage(mode_result.error()));
  }
  operating_mode_ = mode_result.value();
}

Motor::~Motor() {}

Result<void, DxlError> Motor::enableTorque()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Torque Enable");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, 1);
  return result;
}

Result<void, DxlError> Motor::disableTorque()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Torque Enable");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, 0);
  return result;
}

Result<void, DxlError> Motor::setGoalPosition(uint32_t position)
{
  if(torque_ == 0) {
    return DxlError::EASY_SDK_MOTOR_TORQUE_OFF;
  }
  if(operating_mode_ != static_cast<uint8_t>(OperatingMode::POSITION)) {
    return DxlError::EASY_SDK_OPERATING_MODE_MISMATCH;
  }
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Goal Position");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  if (item.size != 4) {
    return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
  }
  Result<void, DxlError> result = connector_->write4ByteData(id_, item.address, position);
  return result;
}

Result<void, DxlError> Motor::setGoalVelocity(uint32_t velocity)
{
  if(torque_ == 0) {
    return DxlError::EASY_SDK_MOTOR_TORQUE_OFF;
  }
  if(operating_mode_ != static_cast<uint8_t>(OperatingMode::VELOCITY)) {
    return DxlError::EASY_SDK_OPERATING_MODE_MISMATCH;
  }
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Goal Velocity");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  if (item.size != 4) {
    return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
  }
  Result<void, DxlError> result = connector_->write4ByteData(id_, item.address, velocity);
  return result;
}

Result<void, DxlError> Motor::setGoalCurrent(int16_t current)
{
  if(torque_ == 0) {
    return DxlError::EASY_SDK_MOTOR_TORQUE_OFF;
  }
  if(operating_mode_ != static_cast<uint8_t>(OperatingMode::CURRENT)) {
    return DxlError::EASY_SDK_OPERATING_MODE_MISMATCH;
  }
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Goal Current");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  if (item.size != 2) {
    return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
  }
  Result<void, DxlError> result = connector_->write2ByteData(id_, item.address, current);
  return result;
}

Result<void, DxlError> Motor::setGoalPWM(int16_t pwm)
{
  if(torque_ == 0) {
    return DxlError::EASY_SDK_MOTOR_TORQUE_OFF;
  }
  if(operating_mode_ != static_cast<uint8_t>(OperatingMode::PWM)) {
    return DxlError::EASY_SDK_OPERATING_MODE_MISMATCH;
  }
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Goal PWM");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  if (item.size != 2) {
    return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
  }
  Result<void, DxlError> result = connector_->write2ByteData(id_, item.address, pwm);
  return result;
}

Result<void, DxlError> Motor::LEDOn()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("LED");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, 1);
  return result;
}

Result<void, DxlError> Motor::LEDOff()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("LED");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, 0);
  return result;
}

Result<uint16_t, DxlError> Motor::ping()
{
  Result<uint16_t, DxlError> result = connector_->ping(id_);
  return result;
}

Result<uint8_t, DxlError> Motor::isTorqueOn()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Torque Enable");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<uint8_t, DxlError> result = connector_->read1ByteData(id_, item.address);
  return result;
}

Result<uint8_t, DxlError> Motor::isLEDOn()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("LED");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<uint8_t, DxlError> result = connector_->read1ByteData(id_, item.address);
  return result;
}

Result<int32_t, DxlError> Motor::getPresentPosition()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Present Position");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<uint32_t, DxlError> result = connector_->read4ByteData(id_, item.address);
  if (!result.isSuccess()) {
    return result.error();
  }
  return static_cast<int32_t>(result.value());
}

Result<int32_t, DxlError> Motor::getPresentVelocity()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Present Velocity");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<uint32_t, DxlError> result = connector_->read4ByteData(id_, item.address);
  if (!result.isSuccess()) {
    return result.error();
  }
  return static_cast<int32_t>(result.value());
}

Result<uint32_t, DxlError> Motor::getMaxPositionLimit()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Max Position Limit");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  if (item.size != 4) {
    return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
  }
  Result<uint32_t, DxlError> result = connector_->read4ByteData(id_, item.address);
  return result;
}

Result<uint32_t, DxlError> Motor::getMinPositionLimit()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Min Position Limit");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  if (item.size != 4) {
    return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
  }
  Result<uint32_t, DxlError> result = connector_->read4ByteData(id_, item.address);
  return result;
}

Result<uint32_t, DxlError> Motor::getVelocityLimit()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Velocity Limit");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  if (item.size != 4) {
    return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
  }
  Result<uint32_t, DxlError> result = connector_->read4ByteData(id_, item.address);
  return result;
}

Result<uint16_t, DxlError> Motor::getCurrentLimit()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Current Limit");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<uint16_t, DxlError> result = connector_->read2ByteData(id_, item.address);
  return result;
}

Result<uint16_t, DxlError> Motor::getPWMLimit()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("PWM Limit");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<uint16_t, DxlError> result = connector_->read2ByteData(id_, item.address);
  return result;
}

Result<uint8_t, DxlError> Motor::getOperatingMode()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Operating Mode");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<uint8_t, DxlError> result = connector_->read1ByteData(id_, item.address);
  return result;
}

Result<void, DxlError> Motor::changeID(uint8_t new_id)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("ID");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, new_id);
  if (result.isSuccess()) {
    id_ = new_id;
  }
  return result;
}

Result<void, DxlError> Motor::setOperatingMode(OperatingMode mode)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Operating Mode");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  uint8_t mode_value = static_cast<uint8_t>(mode);
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, mode_value);
  if (!result.isSuccess()) {
    return result.error();
  }
  return result;
}

Result<void, DxlError> Motor::setProfileConfiguration(ProfileConfiguration config)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Drive Mode");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  auto read_result = connector_->read1ByteData(id_, item.address);
  if (!read_result.isSuccess()) {
    return read_result.error();
  }

  uint8_t drive_mode = read_result.value();
  const uint8_t PROFILE_BIT_MASK = 0b00000100;

  if (config == ProfileConfiguration::TIME_BASED) {
    drive_mode |= PROFILE_BIT_MASK;
  } else if (config == ProfileConfiguration::VELOCITY_BASED) {
    drive_mode &= ~PROFILE_BIT_MASK;
  }
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, drive_mode);
  return result;
}

Result<void, DxlError> Motor::setDirection(Direction direction)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Drive Mode");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  auto read_result = connector_->read1ByteData(id_, item.address);
  if (!read_result.isSuccess()) {
    return read_result.error();
  }
  uint8_t drive_mode = read_result.value();
  const uint8_t DIRECTION_BIT_MASK = 0b00000001;
  if (direction == Direction::NORMAL) {
    drive_mode &= ~DIRECTION_BIT_MASK;
  } else if (direction == Direction::REVERSE) {
    drive_mode |= DIRECTION_BIT_MASK;
  }
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, drive_mode);
  return result;
}

Result<void, DxlError> Motor::setPositionPGain(uint16_t p_gain)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Position P Gain");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  if (item.size != 2) {
    return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
  }
  Result<void, DxlError> result = connector_->write2ByteData(id_, item.address, p_gain);
  return result;
}

Result<void, DxlError> Motor::setPositionIGain(uint16_t i_gain)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Position I Gain");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  if (item.size != 2) {
    return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
  }
  Result<void, DxlError> result = connector_->write2ByteData(id_, item.address, i_gain);
  return result;
}

Result<void, DxlError> Motor::setPositionDGain(uint16_t d_gain)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Position D Gain");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  if (item.size != 2) {
    return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
  }
  Result<void, DxlError> result = connector_->write2ByteData(id_, item.address, d_gain);
  return result;
}

Result<void, DxlError> Motor::setVelocityPGain(uint16_t p_gain)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Velocity P Gain");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  if (item.size != 2) {
    return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
  }
  Result<void, DxlError> result = connector_->write2ByteData(id_, item.address, p_gain);
  return result;
}

Result<void, DxlError> Motor::setVelocityIGain(uint16_t i_gain)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Velocity I Gain");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  if (item.size != 2) {
    return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
  }
  Result<void, DxlError> result = connector_->write2ByteData(id_, item.address, i_gain);
  return result;
}

Result<void, DxlError> Motor::setHomingOffset(int32_t offset)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Homing Offset");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<void, DxlError> result = connector_->write4ByteData(
    id_,
    item.address,
    static_cast<uint32_t>(offset));
  return result;
}

Result<void, DxlError> Motor::setMaxPositionLimit(uint32_t limit)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Max Position Limit");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<void, DxlError> result = connector_->write4ByteData(id_, item.address, limit);
  return result;
}

Result<void, DxlError> Motor::setMinPositionLimit(uint32_t limit)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Min Position Limit");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<void, DxlError> result = connector_->write4ByteData(id_, item.address, limit);
  return result;
}

Result<void, DxlError> Motor::setVelocityLimit(uint32_t limit)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Velocity Limit");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<void, DxlError> result = connector_->write4ByteData(id_, item.address, limit);
  return result;
}

Result<void, DxlError> Motor::setCurrentLimit(uint16_t limit)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Current Limit");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<void, DxlError> result = connector_->write2ByteData(id_, item.address, limit);
  return result;
}

Result<void, DxlError> Motor::setPWMLimit(uint16_t limit)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("PWM Limit");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<void, DxlError> result = connector_->write2ByteData(id_, item.address, limit);
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
    {1});
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
    {0});
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageSetGoalPosition(uint32_t position)
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
    data
  );
  return cmd;
}

Result<StagedCommand, DxlError> Motor::stageSetGoalVelocity(uint32_t velocity)
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
    data
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
    {}
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

Result<ControlTableItem, DxlError> Motor::getControlTableItem(const std::string & name)
{
  auto it = control_table_.find(name);
  if (it == control_table_.end()) {
    return DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED;
  }
  return it->second;
}
}  // namespace dynamixel
