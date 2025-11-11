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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_STAGED_COMMAND_HPP_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_STAGED_COMMAND_HPP_

#include <cstdint>
#include <vector>

namespace dynamixel
{
class Motor;
enum class CommandType
{
  WRITE,
  READ
};

enum class StatusRequest
{
  NONE = 0,
  CHECK_TORQUE_ON = 1,
  CHECK_CURRENT_MODE = 2,
  CHECK_VELOCITY_MODE = 3,
  CHECK_POSITION_MODE = 4,
  CHECK_PWM_MODE = 5,
  UPDATE_TORQUE_STATUS = 6
};

struct StagedCommand
{
  StagedCommand(
    CommandType _command_type,
    uint8_t _id,
    uint16_t _address,
    uint16_t _length,
    const std::vector<uint8_t> & _data,
    StatusRequest _status_request = StatusRequest::NONE,
    Motor * _motor_ptr = nullptr)
  : command_type(_command_type),
    id(_id),
    address(_address),
    length(_length),
    data(_data),
    status_request(_status_request),
    motor_ptr(_motor_ptr) {}

  CommandType command_type;
  uint8_t id;
  uint16_t address;
  uint16_t length;
  std::vector<uint8_t> data;
  StatusRequest status_request;
  Motor * motor_ptr;
};

}  // namespace dynamixel

#endif  // DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_STAGED_COMMAND_HPP_
