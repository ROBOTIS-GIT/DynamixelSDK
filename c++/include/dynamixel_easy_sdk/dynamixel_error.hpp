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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_DYNAMIXEL_ERROR_HPP_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_DYNAMIXEL_ERROR_HPP_

#include <variant>
#include <stdexcept>
#include <string>

namespace dynamixel
{
enum class DxlError
{
  SDK_COMM_SUCCESS = 0,             // tx or rx packet communication success
  SDK_COMM_PORT_BUSY = -1000,       // Port is busy (in use)
  SDK_COMM_TX_FAIL = -1001,         // Failed transmit instruction packet
  SDK_COMM_RX_FAIL = -1002,         // Failed get status packet
  SDK_COMM_TX_ERROR = -2000,        // Incorrect instruction packet
  SDK_COMM_RX_WAITING = -3000,      // Now receiving status packet
  SDK_COMM_RX_TIMEOUT = -3001,      // There is no status packet
  SDK_COMM_RX_CORRUPT = -3002,      // Incorrect status packet
  SDK_COMM_NOT_AVAILABLE = -9000,
  SDK_ERRNUM_RESULT_FAIL = 1,       // Failed to process the instruction packet.
  SDK_ERRNUM_INSTRUCTION = 2,       // Instruction error
  SDK_ERRNUM_CRC = 3,               // CRC check error
  SDK_ERRNUM_DATA_RANGE = 4,        // Data range error
  SDK_ERRNUM_DATA_LENGTH = 5,       // Data length error
  SDK_ERRNUM_DATA_LIMIT = 6,        // Data limit error
  SDK_ERRNUM_ACCESS = 7,            // Access error
  EASY_SDK_FUNCTION_NOT_SUPPORTED = 11,  // API does not support this function
  EASY_SDK_MOTOR_TORQUE_OFF = 12,        // Motor torque is off
  EASY_SDK_OPERATING_MODE_MISMATCH = 13, // Operating mode is not appropriate for this function
  EASY_SDK_ADD_PARAM_FAIL = 21,          // Failed to add parameter
  EASY_SDK_COMMAND_IS_EMPTY = 22,        // No command to execute
  EASY_SDK_DUPLICATE_ID = 23,            // Duplicate ID in staged commands
  EASY_SDK_FAIL_TO_GET_DATA = 24         // Failed to get data from motor
};

template<typename T, typename E>
class Result
{
private:
  std::variant<T, E> result;

public:
  Result() = default;
  Result(const T & return_value)
  : result(return_value)
  {}

  Result(const E & error)
  : result(error)
  {}

  bool isSuccess() const
  {
    return std::holds_alternative<T>(result);
  }

  T & value()
  {
    if (!isSuccess()) {throw std::logic_error("Result has no value.");}
    return std::get<T>(result);
  }
  const T & value() const
  {
    if (!isSuccess()) {throw std::logic_error("Result has no value.");}
    return std::get<T>(result);
  }

  E & error()
  {
    if (isSuccess()) {throw std::logic_error("Result has no error.");}
    return std::get<E>(result);
  }

  const E & error() const
  {
    if (isSuccess()) {throw std::logic_error("Result has no error.");}
    return std::get<E>(result);
  }
};

template<typename E>
class Result<void, E>
{
private:
  std::variant<std::monostate, E> result;

public:
  Result()
  : result(std::monostate{})
  {}

  Result(const E & error)
  : result(error)
  {}

  bool isSuccess() const
  {
    return std::holds_alternative<std::monostate>(result);
  }

  E & error()
  {
    if (!std::holds_alternative<E>(result)) {
      throw std::logic_error("Result has no error.");
    }
    return std::get<E>(result);
  }

  const E & error() const
  {
    if (!std::holds_alternative<E>(result)) {
      throw std::logic_error("Result has no error.");
    }
    return std::get<E>(result);
  }
};

class DxlRuntimeError : public std::runtime_error
{
public:
  explicit DxlRuntimeError(const std::string & message)
  : std::runtime_error(message) {}
};

std::string getErrorMessage(DxlError error_code);
}  // namespace dynamixel

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_DYNAMIXEL_ERROR_HPP_ */
