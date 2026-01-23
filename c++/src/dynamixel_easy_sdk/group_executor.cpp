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

#include <algorithm>

#include "dynamixel_easy_sdk/group_executor.hpp"
#include "dynamixel_easy_sdk/connector.hpp"

// Legacy Definitions usually available, but we define if missing
#ifndef COMM_SUCCESS
#define COMM_SUCCESS 0
#endif

namespace dynamixel
{

GroupExecutor::GroupExecutor(Connector * connector)
: connector_(connector),
  port_handler_(connector->getPortHandler()),
  packet_handler_(connector->getPacketHandler()),
  group_bulk_write_(port_handler_, packet_handler_),
  group_bulk_read_(port_handler_, packet_handler_)
{}

void GroupExecutor::addCmd(StagedCommand command)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (command.command_type == CommandType::WRITE) {
    staged_write_command_list_.push_back(std::move(command));
  } else if (command.command_type == CommandType::READ) {
    staged_read_command_list_.push_back(std::move(command));
  }
}

void GroupExecutor::addCmd(Result<StagedCommand, DxlError> result)
{
  if (!result.is_ok()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (result.value().command_type == CommandType::WRITE) {
    staged_write_command_list_.push_back(std::move(result.value()));
  } else if (result.value().command_type == CommandType::READ) {
    staged_read_command_list_.push_back(std::move(result.value()));
  }
}

Result<void, DxlError> GroupExecutor::executeWrite()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (staged_write_command_list_.empty()) {
    return DxlError(ErrorCode::EASY_SDK_COMMAND_IS_EMPTY);
  }
  if (checkDuplicateId(staged_write_command_list_)) {
    return DxlError(ErrorCode::EASY_SDK_DUPLICATE_ID);
  }

  const auto & reference_command = staged_write_command_list_.front();
  bool is_sync = checkSync(staged_write_command_list_);

  Result<void, DxlError> result;
  if (is_sync) {
    result = executeSyncWrite(reference_command.address, reference_command.length);
  } else {
    result = executeBulkWrite();
  }
  return result;
}

Result<std::vector<Result<int32_t, DxlError>>, DxlError> GroupExecutor::executeRead()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (staged_read_command_list_.empty()) {
    return DxlError(ErrorCode::EASY_SDK_COMMAND_IS_EMPTY);
  }
  if (checkDuplicateId(staged_read_command_list_)) {
    return DxlError(ErrorCode::EASY_SDK_DUPLICATE_ID);
  }

  const auto & reference_command = staged_read_command_list_.front();
  bool is_sync = checkSync(staged_read_command_list_);

  Result<std::vector<Result<int32_t, DxlError>>, DxlError> result;
  if (is_sync) {
    result = executeSyncRead(reference_command.address, reference_command.length);
  } else {
    result = executeBulkRead();
  }
  return result;
}

Result<void, DxlError> GroupExecutor::executeSyncWrite(uint16_t address, uint16_t length)
{
  GroupSyncWrite group_sync_write(port_handler_, packet_handler_, address, length);
  for (auto & command : staged_write_command_list_) {
    Result<void, DxlError> result = processStatusRequests(command);
    if (!result.is_ok()) {
      return result.err();
    }
    if (!group_sync_write.addParam(command.id, command.data.data())) {
      return DxlError(ErrorCode::EASY_SDK_ADD_PARAM_FAIL);
    }
  }
  int dxl_comm_result = group_sync_write.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    return DxlError(static_cast<ErrorCode>(dxl_comm_result));
  }
  return {};
}

Result<void, DxlError> GroupExecutor::executeBulkWrite()
{
  group_bulk_write_.clearParam();

  for (auto & command : staged_write_command_list_) {
    Result<void, DxlError> result = processStatusRequests(command);
    if (!result.is_ok()) {
      return result.err();
    }
    if (!group_bulk_write_.addParam(
        command.id,
        command.address,
        command.length,
        command.data.data()))
    {
      return DxlError(ErrorCode::EASY_SDK_ADD_PARAM_FAIL);
    }
  }

  int dxl_comm_result = group_bulk_write_.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    return DxlError(static_cast<ErrorCode>(dxl_comm_result));
  }
  return {};
}

Result<std::vector<Result<int32_t, DxlError>>, DxlError> GroupExecutor::executeSyncRead(
  uint16_t address,
  uint16_t length)
{
  GroupSyncRead group_sync_read(port_handler_, packet_handler_, address, length);
  for (auto & command : staged_read_command_list_) {
    if (!group_sync_read.addParam(command.id)) {
      return DxlError(ErrorCode::EASY_SDK_ADD_PARAM_FAIL);
    }
  }

  int dxl_comm_result = group_sync_read.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    return DxlError(static_cast<ErrorCode>(dxl_comm_result));
  }
  std::vector<Result<int32_t, DxlError>> result_list;
  for (auto & command : staged_read_command_list_) {
    if (!group_sync_read.isAvailable(command.id, address, length)) {
      result_list.push_back(DxlError(ErrorCode::EASY_SDK_FAIL_TO_GET_DATA));
      continue;
    }

    uint32_t value = group_sync_read.getData(command.id, address, length);
    int32_t signed_value = -1;
    if (command.length == 1) {
      signed_value = static_cast<int32_t>(static_cast<int8_t>(value & 0xFF));
    } else if (command.length == 2) {
      signed_value = static_cast<int32_t>(static_cast<int16_t>(value & 0xFFFF));
    } else if (command.length == 4) {
      signed_value = static_cast<int32_t>(value);
    }

    Result<void, DxlError> result = processStatusRequests(command, signed_value);
    if (!result.is_ok()) {
      return result.err();
    }
    result_list.push_back(signed_value);
  }

  return result_list;
}

Result<std::vector<Result<int32_t, DxlError>>, DxlError> GroupExecutor::executeBulkRead()
{
  group_bulk_read_.clearParam();

  for (auto & command : staged_read_command_list_) {
    if (!group_bulk_read_.addParam(command.id, command.address, command.length)) {
      return DxlError(ErrorCode::EASY_SDK_ADD_PARAM_FAIL);
    }
  }

  int dxl_comm_result = group_bulk_read_.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    return DxlError(static_cast<ErrorCode>(dxl_comm_result));
  }

  std::vector<Result<int32_t, DxlError>> result_list;
  for (auto & command : staged_read_command_list_) {
    if (!group_bulk_read_.isAvailable(command.id, command.address, command.length)) {
      result_list.push_back(DxlError(ErrorCode::EASY_SDK_FAIL_TO_GET_DATA));
      continue;
    }

    uint32_t value = group_bulk_read_.getData(command.id, command.address, command.length);
    int32_t signed_value = -1;
    if (command.length == 1) {
      signed_value = static_cast<int32_t>(static_cast<int8_t>(value & 0xFF));
    } else if (command.length == 2) {
      signed_value = static_cast<int32_t>(static_cast<int16_t>(value & 0xFFFF));
    } else if (command.length == 4) {
      signed_value = static_cast<int32_t>(value);
    }

    Result<void, DxlError> result = processStatusRequests(command, signed_value);
    if (!result.is_ok()) {
      return result.err();
    }
    result_list.push_back(signed_value);
  }

  return result_list;
}

bool GroupExecutor::checkDuplicateId(const std::vector<StagedCommand> & list)
{
  std::vector<uint8_t> ids;
  for (const auto & cmd : list) {
    if (std::find(ids.begin(), ids.end(), cmd.id) != ids.end()) {
      return true;
    }
    ids.push_back(cmd.id);
  }
  return false;
}

bool GroupExecutor::checkSync(const std::vector<StagedCommand> & list)
{
  if (list.empty()) {return false;}
  uint16_t addr = list[0].address;
  uint16_t len = list[0].length;
  for (const auto & cmd : list) {
    if (cmd.address != addr || cmd.length != len) {
      return false;
    }
  }
  return true;
}

Result<void, DxlError> GroupExecutor::processStatusRequests(
  StagedCommand & command, int32_t read_val)
{
  if (!command.status_check_functions.empty()) {
    // Torque Enable Status Check (Virtual Logic)
    // In real implementation, we might need to read Torque Enable from cache or device.
    // Here we assume Torque is enabled if we are writing, or based on some state.
    // For now we skip actual torque check logic implementation as it requires more state management.
  }

  if (command.operating_mode_check.has_value()) {
     // Operating Mode Check (Virtual Logic)
  }

  return {};
}

} // namespace dynamixel
