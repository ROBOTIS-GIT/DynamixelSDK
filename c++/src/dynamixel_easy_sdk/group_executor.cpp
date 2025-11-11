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
  if (command.command_type == CommandType::WRITE) {
    staged_write_command_list_.push_back(std::move(command));
  } else if (command.command_type == CommandType::READ) {
    staged_read_command_list_.push_back(std::move(command));
  }
}

void GroupExecutor::addCmd(Result<StagedCommand, DxlError> result)
{
  if (!result.isSuccess()) {
    return;
  }

  if (result.value().command_type == CommandType::WRITE) {
    staged_write_command_list_.push_back(std::move(result.value()));
  } else if (result.value().command_type == CommandType::READ) {
    staged_read_command_list_.push_back(std::move(result.value()));
  }
}

Result<void, DxlError> GroupExecutor::executeWrite()
{
  if (staged_write_command_list_.empty()) {
    return DxlError::EASY_SDK_COMMAND_IS_EMPTY;
  }
  const auto & reference_command = staged_write_command_list_.front();
  bool is_sync = true;
  std::vector<StagedCommand> sorted_list = staged_write_command_list_;
  std::sort(
    sorted_list.begin(),
    sorted_list.end(),
    [](const StagedCommand & a, const StagedCommand & b) {
      return a.id < b.id;
    }
  );

  auto it = std::adjacent_find(
    sorted_list.begin(),
    sorted_list.end(),
    [](const StagedCommand & a, const StagedCommand & b) {
      return a.id == b.id;
    }
  );

  if (it != sorted_list.end()) {
    return DxlError::EASY_SDK_DUPLICATE_ID;
  }

  for (size_t i = 1; i < staged_write_command_list_.size(); ++i) {
    if (staged_write_command_list_[i].address != reference_command.address ||
      staged_write_command_list_[i].length != reference_command.length)
    {
      is_sync = false;
      break;
    }
  }

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
  if (staged_read_command_list_.empty()) {
    return DxlError::EASY_SDK_COMMAND_IS_EMPTY;
  }
  const auto & reference_command = staged_read_command_list_.front();
  bool is_sync = true;
  std::vector<StagedCommand> sorted_list = staged_read_command_list_;
  std::sort(
    sorted_list.begin(),
    sorted_list.end(),
    [](const StagedCommand & a, const StagedCommand & b) {
      return a.id < b.id;
    }
  );

  auto it = std::adjacent_find(
    sorted_list.begin(),
    sorted_list.end(),
    [](const StagedCommand & a, const StagedCommand & b) {
      return a.id == b.id;
    }
  );

  if (it != sorted_list.end()) {
    return DxlError::EASY_SDK_DUPLICATE_ID;
  }

  for (size_t i = 1; i < staged_read_command_list_.size(); ++i) {
    if (staged_read_command_list_[i].address != reference_command.address ||
      staged_read_command_list_[i].length != reference_command.length)
    {
      is_sync = false;
      break;
    }
  }

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
    if (!result.isSuccess()) {
      return result.error();
    }
    if (!group_sync_write.addParam(command.id, command.data.data())) {
      return DxlError::EASY_SDK_ADD_PARAM_FAIL;
    }
  }
  int dxl_comm_result = group_sync_write.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    return static_cast<DxlError>(dxl_comm_result);
  }
  return {};
}

Result<void, DxlError> GroupExecutor::executeBulkWrite()
{
  group_bulk_write_.clearParam();

  for (auto & command : staged_write_command_list_) {
    Result<void, DxlError> result = processStatusRequests(command);
    if (!result.isSuccess()) {
      return result.error();
    }
    if (!group_bulk_write_.addParam(
        command.id,
        command.address,
        command.length,
        command.data.data()))
    {
      return DxlError::EASY_SDK_ADD_PARAM_FAIL;
    }
  }

  int dxl_comm_result = group_bulk_write_.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    return static_cast<DxlError>(dxl_comm_result);
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
      return DxlError::EASY_SDK_ADD_PARAM_FAIL;
    }
  }

  int dxl_comm_result = group_sync_read.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    return static_cast<DxlError>(dxl_comm_result);
  }
  std::vector<Result<int32_t, DxlError>> result_list;
  for (auto & command : staged_read_command_list_) {
    if (!group_sync_read.isAvailable(command.id, address, length)) {
      result_list.push_back(DxlError::EASY_SDK_FAIL_TO_GET_DATA);
      continue;
    }

    uint32_t value = group_sync_read.getData(command.id, address, length);
    int32_t signed_value;
    if (command.length == 1) {
      signed_value = static_cast<int32_t>(static_cast<int8_t>(value & 0xFF));
    } else if (command.length == 2) {
      signed_value = static_cast<int32_t>(static_cast<int16_t>(value & 0xFFFF));
    } else {
      signed_value = static_cast<int32_t>(value);
    }

    Result<void, DxlError> result = processStatusRequests(command, signed_value);
    if (!result.isSuccess()) {
      return result.error();
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
      return DxlError::EASY_SDK_ADD_PARAM_FAIL;
    }
  }

  int dxl_comm_result = group_bulk_read_.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    return static_cast<DxlError>(dxl_comm_result);
  }

  std::vector<Result<int32_t, DxlError>> result_list;
  for (auto & command : staged_read_command_list_) {
    if (!group_bulk_read_.isAvailable(command.id, command.address, command.length)) {
      result_list.push_back(DxlError::EASY_SDK_FAIL_TO_GET_DATA);
      continue;
    }

    uint32_t value = group_bulk_read_.getData(command.id, command.address, command.length);
    int32_t signed_value;
    if (command.length == 1) {
      signed_value = static_cast<int32_t>(static_cast<int8_t>(value & 0xFF));
    } else if (command.length == 2) {
      signed_value = static_cast<int32_t>(static_cast<int16_t>(value & 0xFFFF));
    } else {
      signed_value = static_cast<int32_t>(value);
    }
    Result<void, DxlError> result = processStatusRequests(command, signed_value);
    if (!result.isSuccess()) {
      return result.error();
    }
    result_list.push_back(signed_value);
  }

  return result_list;
}

Result<void,DxlError> GroupExecutor::processStatusRequests(StagedCommand & cmd, int data)
{
  if (cmd.status_request == StatusRequest::NONE) {
    return {};
  }

  if (cmd.status_request == StatusRequest::CHECK_TORQUE_ON) {
    if (cmd.motor_ptr->getTorqueStatus() != 1) {
      return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
    }
  } else if (cmd.status_request == StatusRequest::CHECK_CURRENT_MODE) {
    if (cmd.motor_ptr->getOperatingModeStatus() != OperatingMode::CURRENT) {
      return DxlError::EASY_SDK_OPERATING_MODE_MISMATCH;
    }
    if (cmd.motor_ptr->getTorqueStatus() != 1) {
      return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
    }
  } else if (cmd.status_request == StatusRequest::CHECK_VELOCITY_MODE) {
    if (cmd.motor_ptr->getOperatingModeStatus() != OperatingMode::VELOCITY) {
      return DxlError::EASY_SDK_OPERATING_MODE_MISMATCH;
    }
    if (cmd.motor_ptr->getTorqueStatus() != 1) {
      return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
    }
  } else if (cmd.status_request == StatusRequest::CHECK_POSITION_MODE) {
    if (cmd.motor_ptr->getOperatingModeStatus() != OperatingMode::POSITION &&
        cmd.motor_ptr->getOperatingModeStatus() != OperatingMode::EXTENDED_POSITION) {
      return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
    }
  } else if (cmd.status_request == StatusRequest::CHECK_PWM_MODE) {
    if (cmd.motor_ptr->getOperatingModeStatus() != OperatingMode::PWM) {
      return DxlError::EASY_SDK_OPERATING_MODE_MISMATCH;
    }
    if (cmd.motor_ptr->getTorqueStatus() != 1) {
      return DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH;
    }
  } else if (cmd.status_request == StatusRequest::UPDATE_TORQUE_STATUS) {
    if (data != -1) {
      cmd.motor_ptr->setTorqueStatus(data);
      return {};
    }
    cmd.motor_ptr->setTorqueStatus(cmd.data[0]);
    return {};
  }
  return {};
}
}  // namespace dynamixel
