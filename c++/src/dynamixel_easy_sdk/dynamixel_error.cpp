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

#include "dynamixel_easy_sdk/dynamixel_error.hpp"

namespace dynamixel
{
std::string getErrorMessage(DxlError error_code)
{
  switch (error_code) {
    case DxlError::SDK_COMM_SUCCESS:
      return "[TxRxResult] Communication success.";
    case DxlError::SDK_COMM_PORT_BUSY:
      return "[TxRxResult] Port is in use!";
    case DxlError::SDK_COMM_TX_FAIL:
      return "[TxRxResult] Failed transmit instruction packet!";
    case DxlError::SDK_COMM_RX_FAIL:
      return "[TxRxResult] Failed get status packet from device!";
    case DxlError::SDK_COMM_TX_ERROR:
      return "[TxRxResult] Incorrect instruction packet!";
    case DxlError::SDK_COMM_RX_WAITING:
      return "[TxRxResult] Now receiving status packet!";
    case DxlError::SDK_COMM_RX_TIMEOUT:
      return "[TxRxResult] There is no status packet!";
    case DxlError::SDK_COMM_RX_CORRUPT:
      return "[TxRxResult] Incorrect status packet!";
    case DxlError::SDK_COMM_NOT_AVAILABLE:
      return "[TxRxResult] Protocol does not support this function!";
    case DxlError::SDK_ERRNUM_RESULT_FAIL:
      return "[RxPacketError] Failed to process the instruction packet!";
    case DxlError::SDK_ERRNUM_INSTRUCTION:
      return "[RxPacketError] Undefined instruction or incorrect instruction!";
    case DxlError::SDK_ERRNUM_CRC:
      return "[RxPacketError] CRC doesn't match!";
    case DxlError::SDK_ERRNUM_DATA_RANGE:
      return "[RxPacketError] The data value is out of range!";
    case DxlError::SDK_ERRNUM_DATA_LENGTH:
      return "[RxPacketError] The data length does not match as expected!";
    case DxlError::SDK_ERRNUM_DATA_LIMIT:
      return "[RxPacketError] The data value exceeds the limit value!";
    case DxlError::SDK_ERRNUM_ACCESS:
      return "[RxPacketError] Writing or Reading is not available to target address!";
    case DxlError::EASY_SDK_FUNCTION_NOT_SUPPORTED:
      return "[EasySDKError] Easy SDK function is not supported on this model!";
    case DxlError::EASY_SDK_MOTOR_TORQUE_OFF:
      return "[EasySDKError] Motor torque is off!";
    case DxlError::EASY_SDK_OPERATING_MODE_MISMATCH:
      return "[EasySDKError] Operating mode is not appropriate for this function!";
    case DxlError::EASY_SDK_ADD_PARAM_FAIL:
      return "[EasySDKError] Failed to add parameter!";
    case DxlError::EASY_SDK_COMMAND_IS_EMPTY:
      return "[EasySDKError] No command to execute!";
    case DxlError::EASY_SDK_DUPLICATE_ID:
      return "[EasySDKError] Duplicate ID found in staged commands.";
    case DxlError::EASY_SDK_FAIL_TO_GET_DATA:
      return "[EasySDKError] Failed to get data from motor.";
    default: return "Unknown Error";
  }
}
}  // namespace dynamixel
