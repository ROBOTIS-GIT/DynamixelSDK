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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_MOTOR_HPP_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_MOTOR_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_easy_sdk/control_table.hpp"
#include "dynamixel_easy_sdk/dynamixel_error.hpp"
#include "dynamixel_easy_sdk/staged_command.hpp"

namespace dynamixel
{
class Connector;

class Motor
{
public:
  enum class OperatingMode
  {
    CURRENT = 0,
    VELOCITY = 1,
    POSITION = 3,
    EXTENDED_POSITION = 4,
    PWM = 16
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
  Result<void, DxlError> setGoalCurrent(int16_t current);
  Result<void, DxlError> setGoalPWM(int16_t pwm);
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
  Result<uint16_t, DxlError> getCurrentLimit();
  Result<uint16_t, DxlError> getPWMLimit();
  Result<uint8_t, DxlError> getOperatingMode();

  Result<void, DxlError> changeID(uint8_t new_id);
  Result<void, DxlError> setOperatingMode(OperatingMode mode);
  Result<void, DxlError> setProfileConfiguration(ProfileConfiguration config);
  Result<void, DxlError> setDirection(Direction direction);
  Result<void, DxlError> setPositionPGain(uint16_t p_gain);
  Result<void, DxlError> setPositionIGain(uint16_t i_gain);
  Result<void, DxlError> setPositionDGain(uint16_t d_gain);
  Result<void, DxlError> setVelocityPGain(uint16_t p_gain);
  Result<void, DxlError> setVelocityIGain(uint16_t i_gain);
  Result<void, DxlError> setHomingOffset(int32_t offset);
  Result<void, DxlError> setMaxPositionLimit(uint32_t limit);
  Result<void, DxlError> setMinPositionLimit(uint32_t limit);
  Result<void, DxlError> setVelocityLimit(uint32_t limit);
  Result<void, DxlError> setCurrentLimit(uint16_t limit);
  Result<void, DxlError> setPWMLimit(uint16_t limit);

  Result<void, DxlError> reboot();
  Result<void, DxlError> factoryResetAll();
  Result<void, DxlError> factoryResetExceptID();
  Result<void, DxlError> factoryResetExceptIDAndBaudRate();

  Result<StagedCommand, DxlError> stageEnableTorque();
  Result<StagedCommand, DxlError> stageDisableTorque();
  Result<StagedCommand, DxlError> stageSetGoalPosition(uint32_t position);
  Result<StagedCommand, DxlError> stageSetGoalVelocity(uint32_t velocity);
  Result<StagedCommand, DxlError> stageLEDOn();
  Result<StagedCommand, DxlError> stageLEDOff();

  Result<StagedCommand, DxlError> stageIsTorqueOn();
  Result<StagedCommand, DxlError> stageIsLEDOn();
  Result<StagedCommand, DxlError> stageGetPresentPosition();
  Result<StagedCommand, DxlError> stageGetPresentVelocity();

  uint8_t getID() const {return id_;}
  uint16_t getModelNumber() const {return model_number_;}
  std::string getModelName() const {return model_name_;}

private:
  Result<ControlTableItem, DxlError> getControlTableItem(const std::string & item_name);
  uint8_t id_;
  uint16_t model_number_;
  std::string model_name_;
  uint8_t torque_;
  uint8_t operating_mode_;

  Connector * connector_;
  const std::map<std::string, ControlTableItem> & control_table_;
};
}  // namespace dynamixel
#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_MOTOR_HPP_ */
