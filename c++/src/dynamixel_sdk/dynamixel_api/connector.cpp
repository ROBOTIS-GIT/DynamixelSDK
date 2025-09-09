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

#include "dynamixel_sdk/dynamixel_api/connector.hpp"

namespace dynamixel
{
Connector::Connector(const std::string & port_name, float protocol_version, int baud_rate)
{
  if (protocol_version != 2.0f){
    throw DxlRuntimeError("Only Protocol 2.0 is supported in this Dynamixel API.");
  }
  port_handler_ = std::unique_ptr<PortHandler>(PortHandler::getPortHandler(port_name.c_str()));
  packet_handler_ = std::unique_ptr<PacketHandler>(
  PacketHandler::getPacketHandler(protocol_version));

  if (!port_handler_->openPort()) {
    throw DxlRuntimeError("Failed to open the port!");
  }

  if (!port_handler_->setBaudRate(baud_rate)) {
    throw DxlRuntimeError("Failed to set the baud rate!");
  }
}

Connector::~Connector()
{
  port_handler_->closePort();
}

std::unique_ptr<Motor> Connector::getMotor(uint8_t id)
{
  Result<uint16_t, DxlError> result = ping(id);
  if (!result.isSuccess()) {
    throw DxlRuntimeError(getErrorMessage(result.error()));
  }
  return std::make_unique<Motor>(id, result.value(), this);
}

std::vector<std::unique_ptr<Motor>> Connector::getAllMotors(int start_id, int end_id)
{
  if (start_id < 0 || start_id > 252 || end_id < 0 || end_id > 252 || start_id > end_id) {
    throw DxlRuntimeError("Invalid ID range. ID should be in the range of 0 to 252.");
  }
  std::vector<uint8_t> ids;
  std::vector<std::unique_ptr<Motor>> motors;
  int dxl_comm_result = packet_handler_->broadcastPing(port_handler_.get(), ids);
  if (dxl_comm_result != COMM_SUCCESS) {
    throw DxlRuntimeError(getErrorMessage(static_cast<DxlError>(dxl_comm_result)));
  }
  for (auto &id : ids) {
      if (id >= start_id && id <= end_id) {
          motors.push_back(getMotor(id));
      }
  }
  return motors;
}

Result<uint8_t, DxlError> Connector::read1ByteData(uint8_t id, uint16_t address)
{
  uint8_t dxl_error = 0;
  uint8_t data = 0;
  int dxl_comm_result = packet_handler_->read1ByteTxRx(
    port_handler_.get(),
    id, address,
    &data,
    &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    return static_cast<DxlError>(dxl_comm_result);
  }
  if (dxl_error != 0) {
    return static_cast<DxlError>(dxl_error);
  }
  return data;
}

Result<uint16_t, DxlError> Connector::read2ByteData(uint8_t id, uint16_t address)
{
  uint8_t dxl_error = 0;
  uint16_t data = 0;
  int dxl_comm_result = packet_handler_->read2ByteTxRx(
    port_handler_.get(),
    id, address,
    &data,
    &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    return static_cast<DxlError>(dxl_comm_result);
  }
  if (dxl_error != 0) {
    return static_cast<DxlError>(dxl_error);
  }
  return data;
}

Result<uint32_t, DxlError> Connector::read4ByteData(uint8_t id, uint16_t address)
{
  uint8_t dxl_error = 0;
  uint32_t data = 0;
  int dxl_comm_result = packet_handler_->read4ByteTxRx(
    port_handler_.get(),
    id, address,
    &data,
    &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    return static_cast<DxlError>(dxl_comm_result);
  }
  if (dxl_error != 0) {
    return static_cast<DxlError>(dxl_error);
  }
  return data;
}

Result<void, DxlError> Connector::write1ByteData(uint8_t id, uint16_t address, uint8_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = packet_handler_->write1ByteTxRx(
    port_handler_.get(),
    id, address, value,
    &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    return static_cast<DxlError>(dxl_comm_result);
  }
  if (dxl_error != 0) {
    return static_cast<DxlError>(dxl_error);
  }
  return {};
}

Result<void, DxlError> Connector::write2ByteData(uint8_t id, uint16_t address, uint16_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = packet_handler_->write2ByteTxRx(
    port_handler_.get(),
    id, address,
    value,
    &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    return static_cast<DxlError>(dxl_comm_result);
  }
  if (dxl_error != 0) {
    return static_cast<DxlError>(dxl_error);
  }
  return {};
}

Result<void, DxlError> Connector::write4ByteData(uint8_t id, uint16_t address, uint32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = packet_handler_->write4ByteTxRx(
    port_handler_.get(),
    id, address,
    value,
    &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    return static_cast<DxlError>(dxl_comm_result);
  }
  if (dxl_error != 0) {
    return static_cast<DxlError>(dxl_error);
  }
  return {};
}

Result<void, DxlError> Connector::reboot(uint8_t id)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = packet_handler_->reboot(port_handler_.get(), id, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    return static_cast<DxlError>(dxl_comm_result);
  }
  if (dxl_error != 0) {
    return static_cast<DxlError>(dxl_error);
  }
  return {};
}

Result<uint16_t, DxlError> Connector::ping(uint8_t id)
{
  Result<uint16_t, DxlError> result = read2ByteData(id, 0);
  return result;
}

Result<void, DxlError> Connector::factoryReset(uint8_t id, uint8_t option)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = packet_handler_->factoryReset(port_handler_.get(), id, option, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    return static_cast<DxlError>(dxl_comm_result);
  }
  if (dxl_error != 0) {
    return static_cast<DxlError>(dxl_error);
  }
  return {};
}
}  // namespace dynamixel
