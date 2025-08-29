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

#include "dynamixel_api/motor.hpp"
#include "dynamixel_api/connector.hpp"

namespace dynamixel
{
Motor::Motor(uint8_t id, uint16_t model_number, Connector * connector)
: id_(id),
  model_number_(model_number),
  model_name_(ControlTable::getModelName(model_number)),
  connector_(connector),
  control_table_(ControlTable::getControlTable(model_number))
{}

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
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Goal Position");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  if (item.size != 4) {
    return DxlError::API_FUNCTION_NOT_SUPPORTED;
  }
  Result<void, DxlError> result = connector_->write4ByteData(id_, item.address, position);
  return result;
}

Result<void, DxlError> Motor::setGoalVelocity(uint32_t velocity)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Goal Velocity");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  if (item.size != 4) {
    return DxlError::API_FUNCTION_NOT_SUPPORTED;
  }
  Result<void, DxlError> result = connector_->write4ByteData(id_, item.address, velocity);
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
  Result<uint16_t, DxlError> result = connector_->read2ByteData(id_, 0);
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
    return DxlError::API_FUNCTION_NOT_SUPPORTED;
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
    return DxlError::API_FUNCTION_NOT_SUPPORTED;
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
    return DxlError::API_FUNCTION_NOT_SUPPORTED;
  }
  Result<uint32_t, DxlError> result = connector_->read4ByteData(id_, item.address);
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

Result<void, DxlError> Motor::changeBaudRate(int new_baud_rate)
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Baud Rate");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  uint8_t baud_value = static_cast<uint8_t>(2000000 / new_baud_rate - 1);
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, baud_value);
  return result;
}

Result<void, DxlError> Motor::setPositionControlMode()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Operating Mode");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, 3);
  return result;
}

Result<void, DxlError> Motor::setVelocityControlMode()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Operating Mode");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, 1);
  return result;
}

Result<void, DxlError> Motor::setCurrentControlMode()
{
  Result<ControlTableItem, DxlError> item_result = getControlTableItem("Operating Mode");
  if (!item_result.isSuccess()) {
    return item_result.error();
  }
  const ControlTableItem & item = item_result.value();
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, 0);
  return result;
}

Result<void, DxlError> Motor::setTimeBasedProfile()
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
  drive_mode |= 0b00000100;
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, drive_mode);
  return result;
}

Result<void, DxlError> Motor::setVelocityBasedProfile()
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
  drive_mode &= 0b11111011;
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, drive_mode);
  return result;
}

Result<void, DxlError> Motor::setNormalDirection()
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
  drive_mode &= 0b11111110;
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, drive_mode);
  return result;
}

Result<void, DxlError> Motor::setReverseDirection()
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
  drive_mode |= 0b00000001;
  Result<void, DxlError> result = connector_->write1ByteData(id_, item.address, drive_mode);
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

Result<ControlTableItem, DxlError> Motor::getControlTableItem(const std::string & name)
{
  auto it = control_table_.find(name);
  if (it == control_table_.end())
    return DxlError::API_FUNCTION_NOT_SUPPORTED;
  return it->second;
}
}  // namespace dynamixel
