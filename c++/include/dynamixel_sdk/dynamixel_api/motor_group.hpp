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

#ifndef DYNAMIXEL_SDK_DYNAMIXEL_API_MOTOR_GROUP_HPP_
#define DYNAMIXEL_SDK_DYNAMIXEL_API_MOTOR_GROUP_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/dynamixel_api/motor.hpp"
#include "dynamixel_sdk/dynamixel_api/control_table.hpp"
#include "dynamixel_sdk/dynamixel_api/dynamixel_error.hpp"

namespace dynamixel
{
class Connector;

class MotorGroup
{
public:
  enum class CommandType
  {
    WRITE,
    READ
  };
  class MotorProxy
  {
  public:
    using OperatingMode = Motor::OperatingMode;
    using ProfileConfiguration = Motor::ProfileConfiguration;
    using Direction = Motor::Direction;

    MotorProxy(uint8_t id, uint16_t model_number, Connector * connector, MotorGroup * motor_group);
    ~MotorProxy() = default;

    Result<void, DxlError> stageEnableTorque();
    Result<void, DxlError> stageDisableTorque();
    Result<void, DxlError> stageSetGoalPosition(uint32_t position);
    Result<void, DxlError> stageSetGoalVelocity(uint32_t velocity);
    Result<void, DxlError> stageLEDOn();
    Result<void, DxlError> stageLEDOff();

    Result<void, DxlError> stageIsTorqueOn();
    Result<void, DxlError> stageIsLEDOn();
    Result<void, DxlError> stageGetPresentPosition();
    Result<void, DxlError> stageGetPresentVelocity();

    uint8_t getID() const {return id_;}
    uint16_t getModelNumber() const {return model_number_;}
    std::string getModelName() const {return model_name_;}

  private:
    Result<ControlTableItem, DxlError> getControlTableItem(const std::string & item_name);
    uint8_t id_;
    uint16_t model_number_;
    std::string model_name_;
    MotorGroup * motor_group_;

    Connector * connector_;
    const std::map<std::string, ControlTableItem> & control_table_;
  };

  explicit MotorGroup(Connector * connector);
  virtual ~MotorGroup() = default;

  MotorProxy * addMotor(uint8_t id, const std::string & motor_name);
  Result<void, DxlError> executeWrite();
  Result<std::vector<Result<int32_t, DxlError>>, DxlError> executeRead();

private:
  struct StagedCommand
  {
    StagedCommand(
      CommandType _command_type,
      uint8_t _id,
      uint16_t _address,
      uint16_t _length,
      const std::vector<uint8_t> & _data)
    : command_type(_command_type), id(_id), address(_address), length(_length), data(_data) {}
    CommandType command_type;
    uint8_t id;
    uint16_t address;
    uint16_t length;
    std::vector<uint8_t> data;
  };

  Result<void, DxlError> executeSyncWrite(uint16_t address, uint16_t length);
  Result<void, DxlError> executeBulkWrite();
  Result<std::vector<Result<int32_t, DxlError>>, DxlError> executeSyncRead(
    uint16_t address,
    uint16_t length);
  Result<std::vector<Result<int32_t, DxlError>>, DxlError> executeBulkRead();
  void stageCommand(StagedCommand && command, CommandType type);

  Connector * connector_;
  PortHandler * port_handler_;
  PacketHandler * packet_handler_;

  std::unordered_map<std::string, std::unique_ptr<MotorProxy>> motors_list_;
  GroupBulkWrite group_bulk_write_;
  GroupBulkRead group_bulk_read_;
  std::vector<StagedCommand> staged_write_command_list_;
  std::vector<StagedCommand> staged_read_command_list_;
  friend class MotorProxy;
};

}  // namespace dynamixel

#endif  // DYNAMIXEL_SDK_DYNAMIXEL_API_MOTOR_GROUP_HPP_
