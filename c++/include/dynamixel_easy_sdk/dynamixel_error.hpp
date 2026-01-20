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
#include <iostream>

namespace dynamixel
{

// Comprehensive Error Codes
enum class ErrorCode : int {
  // Communication Results (Negative)
  SUCCESS = 0,
  SDK_COMM_SUCCESS = 0,
  SDK_COMM_PORT_BUSY = -1000,
  SDK_COMM_TX_FAIL = -1001,
  SDK_COMM_RX_FAIL = -1002,
  SDK_COMM_TX_ERROR = -2000,
  SDK_COMM_RX_WAITING = -3000,
  SDK_COMM_RX_TIMEOUT = -3001,
  SDK_COMM_RX_CORRUPT = -3002,
  SDK_COMM_NOT_AVAILABLE = -9000,

  // Packet Errors (Positive - from Status Packet)
  SDK_ERRNUM_RESULT_FAIL = 1,
  SDK_ERRNUM_INSTRUCTION = 2,
  SDK_ERRNUM_CRC = 3,
  SDK_ERRNUM_DATA_RANGE = 4,
  SDK_ERRNUM_DATA_LENGTH = 5,
  SDK_ERRNUM_DATA_LIMIT = 6,
  SDK_ERRNUM_ACCESS = 7,

  // Easy SDK Specific Errors (11+)
  EASY_SDK_FUNCTION_NOT_SUPPORTED = 11,
  EASY_SDK_TORQUE_STATUS_MISMATCH = 12,
  EASY_SDK_OPERATING_MODE_MISMATCH = 13,
  EASY_SDK_ADD_PARAM_FAIL = 21,
  EASY_SDK_COMMAND_IS_EMPTY = 22,
  EASY_SDK_DUPLICATE_ID = 23,
  EASY_SDK_FAIL_TO_GET_DATA = 24
};

// The Error Object used in Results
struct DxlError : public std::runtime_error
{
  int comm_result;
  int packet_error;
  std::string msg;

  DxlError(int c, int p, std::string m) 
    : std::runtime_error(m), comm_result(c), packet_error(p), msg(std::move(m)) {}
  
  // Constructor from ErrorCode
  explicit DxlError(ErrorCode code);
};

std::string getErrorMessage(ErrorCode code);

template<typename T, typename E>
class Result
{
private:
  std::variant<T, E> result;

public:
  Result() = default;
  Result(const T & return_value) : result(return_value) {}
  Result(const E & error) : result(error) {}

  // Rust-like API
  static Result<T, E> Ok(const T & val) { return Result(val); }
  static Result<T, E> Err(const E & val) { return Result(val); }

  bool is_ok() const { return std::holds_alternative<T>(result); }
  bool isSuccess() const { return is_ok(); } // Legacy Alias

  bool is_err() const { return std::holds_alternative<E>(result); }

  T & value() {
    if (is_err()) throw std::logic_error("Result has no value");
    return std::get<T>(result);
  }

  const E & err() const {
    if (is_ok()) throw std::logic_error("Result has no error");
    return std::get<E>(result);
  }
  const E & error() const { return err(); } // Legacy Alias
};

// Specialization for void
template<typename E>
class Result<void, E>
{
private:
  std::variant<std::monostate, E> result;

public:
  Result() : result(std::monostate{}) {}
  Result(const E & error) : result(error) {}

  static Result<void, E> Ok() { return Result(); }
  static Result<void, E> Err(const E & val) { return Result(val); }

  bool is_ok() const { return std::holds_alternative<std::monostate>(result); }
  bool isSuccess() const { return is_ok(); }

  bool is_err() const { return std::holds_alternative<E>(result); }

  const E & err() const {
    if (is_ok()) throw std::logic_error("Result has no error");
    return std::get<E>(result);
  }
  const E & error() const { return err(); }
};

class DxlRuntimeError : public std::runtime_error
{
public:
  explicit DxlRuntimeError(const std::string & message)
  : std::runtime_error(message) {}
};

} // namespace dynamixel

#endif // DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_DYNAMIXEL_ERROR_HPP_
