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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_CONTROL_TABLE_HPP_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_CONTROL_TABLE_HPP_

#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

#include "dynamixel_easy_sdk/dynamixel_error.hpp"

namespace dynamixel
{
struct ControlTableItem
{
  uint16_t address;
  uint8_t size;
};

class ControlTable
{
public:
  static const std::map<uint16_t, std::string> ParsingModelList();
  static const std::string getModelName(uint16_t model_number);
  static const std::map<std::string, ControlTableItem> & getControlTable(uint16_t model_number);
};
}  // namespace dynamixel
#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_CONTROL_TABLE_HPP_ */
