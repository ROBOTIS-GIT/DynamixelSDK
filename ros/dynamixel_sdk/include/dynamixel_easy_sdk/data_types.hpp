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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_DATA_TYPES_HPP_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_DATA_TYPES_HPP_

#include <cstdint>
#include <vector>

namespace dynamixel
{
class Motor;

struct ControlTableItem
{
  uint16_t address;
  uint8_t size;
};

enum class OperatingMode
{
  CURRENT = 0,
  VELOCITY = 1,
  POSITION = 3,
  EXTENDED_POSITION = 4,
  CURRENT_BASED_POSITION = 5,
  PWM = 16
};

enum class Direction
{
  NORMAL = 0,
  REVERSE = 1
};

enum class ProfileConfiguration
{
  VELOCITY_BASED = 0,
  TIME_BASED = 1
};

enum class CommandType
{
  WRITE,
  READ
};

enum class StatusRequest
{
  NONE = 0,
  CHECK_TORQUE_ON = 1,
  CHECK_OPERATING_MODE = 2,
  UPDATE_TORQUE_STATUS = 3
};

struct StagedCommand
{
  StagedCommand(
    CommandType _command_type,
    uint8_t _id,
    uint16_t _address,
    uint16_t _length,
    const std::vector<uint8_t> & _data,
    const std::vector<StatusRequest> & _status_request = {StatusRequest::NONE},
    Motor * _motor_ptr = nullptr,
    const std::vector<OperatingMode> & _allowable_operating_modes = {})
  : command_type(_command_type),
    id(_id),
    address(_address),
    length(_length),
    data(_data),
    status_request(_status_request),
    motor_ptr(_motor_ptr),
    allowable_operating_modes(_allowable_operating_modes) {}

  CommandType command_type;
  uint8_t id;
  uint16_t address;
  uint16_t length;
  std::vector<uint8_t> data;
  std::vector<StatusRequest> status_request;
  Motor * motor_ptr;
  std::vector<OperatingMode> allowable_operating_modes;
};

}  // namespace dynamixel

#endif  // DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_DATA_TYPES_HPP_
