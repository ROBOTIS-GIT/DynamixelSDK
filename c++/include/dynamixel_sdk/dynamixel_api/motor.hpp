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

#ifndef DYNAMIXEL_SDK_DYNAMIXEL_API_MOTOR_HPP_
#define DYNAMIXEL_SDK_DYNAMIXEL_API_MOTOR_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_api/control_table.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/dynamixel_api/dynamixel_error.hpp"

namespace dynamixel
{
class Connector;

class Motor
{
public:
  enum class OperatingMode
  {
    POSITION = 3,
    VELOCITY = 1,
    CURRENT = 0
  };

  enum class ProfileConfiguration
  {
    VELOCITY_BASED = 0,
    TIME_BASED = 1
  };

  enum class Direction
  {
    NORMAL = 0,
    REVERSE = 1
  };

  Motor(uint8_t id, uint16_t model_number, Connector * connector);

  virtual ~Motor();

  Result<void, DxlError> enableTorque();
  Result<void, DxlError> disableTorque();
  Result<void, DxlError> setGoalPosition(uint32_t position);
  Result<void, DxlError> setGoalVelocity(uint32_t velocity);
  Result<void, DxlError> LEDOn();
  Result<void, DxlError> LEDOff();

  Result<uint16_t, DxlError> ping();
  Result<uint8_t, DxlError> isTorqueOn();
  Result<uint8_t, DxlError> isLEDOn();
  Result<int32_t, DxlError> getPresentPosition();
  Result<int32_t, DxlError> getPresentVelocity();
  Result<uint32_t, DxlError> getMaxPositionLimit();
  Result<uint32_t, DxlError> getMinPositionLimit();
  Result<uint32_t, DxlError> getVelocityLimit();

  Result<void, DxlError> changeID(uint8_t new_id);
  Result<void, DxlError> setOperatingMode(OperatingMode mode);
  Result<void, DxlError> setProfileConfiguration(ProfileConfiguration config);
  Result<void, DxlError> setDirection(Direction direction);

  Result<void, DxlError> reboot();
  Result<void, DxlError> factoryResetAll();
  Result<void, DxlError> factoryResetExceptID();
  Result<void, DxlError> factoryResetExceptIDAndBaudRate();

  uint8_t getID() const {return id_;}
  uint16_t getModelNumber() const {return model_number_;}
  std::string getModelName() const {return model_name_;}

private:
  Result<ControlTableItem, DxlError> getControlTableItem(const std::string & item_name);
  uint8_t id_;
  uint16_t model_number_;
  std::string model_name_;

  Connector * connector_;
  const std::map<std::string, ControlTableItem> & control_table_;
};
}  // namespace dynamixel
#endif /* DYNAMIXEL_SDK_DYNAMIXEL_API_MOTOR_HPP_ */
