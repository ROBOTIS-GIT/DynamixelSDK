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
enum class CommandType
{
  WRITE,
  READ
};

struct StagedCommand
{
  StagedCommand(
    CommandType _command_type,
    uint8_t _id,
    uint16_t _address,
    uint16_t _length,
    const std::vector<uint8_t> & _data)
  : command_type(_command_type),
    id(_id),
    address(_address),
    length(_length),
    data(_data) {}

  CommandType command_type;
  uint8_t id;
  uint16_t address;
  uint16_t length;
  std::vector<uint8_t> data;
};

}  // namespace dynamixel

#endif  // DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_STAGED_COMMAND_HPP_
