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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_GROUP_EXECUTOR_HPP_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_GROUP_EXECUTOR_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_easy_sdk/control_table.hpp"
#include "dynamixel_easy_sdk/dynamixel_error.hpp"
#include "dynamixel_easy_sdk/staged_command.hpp"

namespace dynamixel
{
class Connector;

class GroupExecutor
{
public:
  explicit GroupExecutor(Connector * connector);
  virtual ~GroupExecutor() = default;

  void addCmd(StagedCommand command);
  void addCmd(Result<StagedCommand, DxlError> result);
  Result<void, DxlError> executeWrite();
  Result<std::vector<Result<int32_t, DxlError>>, DxlError> executeRead();
  std::vector<StagedCommand> getStagedWriteCommands() const { return staged_write_command_list_; }
  std::vector<StagedCommand> getStagedReadCommands() const { return staged_read_command_list_; }
  void clearStagedWriteCommands() { staged_write_command_list_.clear(); }
  void clearStagedReadCommands() { staged_read_command_list_.clear(); }

private:
  Result<void, DxlError> executeSyncWrite(uint16_t address, uint16_t length);
  Result<void, DxlError> executeBulkWrite();
  Result<std::vector<Result<int32_t, DxlError>>, DxlError> executeSyncRead(
    uint16_t address,
    uint16_t length);
  Result<std::vector<Result<int32_t, DxlError>>, DxlError> executeBulkRead();

  Connector * connector_;
  PortHandler * port_handler_;
  PacketHandler * packet_handler_;

  GroupBulkWrite group_bulk_write_;
  GroupBulkRead group_bulk_read_;
  std::vector<StagedCommand> staged_write_command_list_;
  std::vector<StagedCommand> staged_read_command_list_;
};

}  // namespace dynamixel

#endif  // DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_GROUP_EXECUTOR_HPP_
