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
#include "dynamixel_sdk/packet_handler.h"

namespace dynamixel
{
static_assert(COMM_SUCCESS == static_cast<int>(DxlError::SDK_COMM_SUCCESS), "DxlError must match COMM_SUCCESS");
static_assert(COMM_PORT_BUSY == static_cast<int>(DxlError::SDK_COMM_PORT_BUSY), "DxlError must match COMM_PORT_BUSY");
static_assert(COMM_TX_FAIL == static_cast<int>(DxlError::SDK_COMM_TX_FAIL), "DxlError must match COMM_TX_FAIL");
static_assert(COMM_RX_FAIL == static_cast<int>(DxlError::SDK_COMM_RX_FAIL), "DxlError must match COMM_RX_FAIL");
static_assert(COMM_TX_ERROR == static_cast<int>(DxlError::SDK_COMM_TX_ERROR), "DxlError must match COMM_TX_ERROR");
static_assert(COMM_RX_WAITING == static_cast<int>(DxlError::SDK_COMM_RX_WAITING), "DxlError must match COMM_RX_WAITING");
static_assert(COMM_RX_TIMEOUT == static_cast<int>(DxlError::SDK_COMM_RX_TIMEOUT), "DxlError must match COMM_RX_TIMEOUT");
static_assert(COMM_RX_CORRUPT == static_cast<int>(DxlError::SDK_COMM_RX_CORRUPT), "DxlError must match COMM_RX_CORRUPT");
static_assert(COMM_NOT_AVAILABLE == static_cast<int>(DxlError::SDK_COMM_NOT_AVAILABLE),
  "DxlError must match COMM_NOT_AVAILABLE");

// Protocol 2 hardware error codes (status packet ERROR field); must match packetErrorToDxlError mapping.
static_assert(1 == static_cast<int>(DxlError::SDK_ERRNUM_RESULT_FAIL), "DxlError must match packet error 1");
static_assert(2 == static_cast<int>(DxlError::SDK_ERRNUM_INSTRUCTION), "DxlError must match packet error 2");
static_assert(3 == static_cast<int>(DxlError::SDK_ERRNUM_CRC), "DxlError must match packet error 3");
static_assert(4 == static_cast<int>(DxlError::SDK_ERRNUM_DATA_RANGE), "DxlError must match packet error 4");
static_assert(5 == static_cast<int>(DxlError::SDK_ERRNUM_DATA_LENGTH), "DxlError must match packet error 5");
static_assert(6 == static_cast<int>(DxlError::SDK_ERRNUM_DATA_LIMIT), "DxlError must match packet error 6");
static_assert(7 == static_cast<int>(DxlError::SDK_ERRNUM_ACCESS), "DxlError must match packet error 7");

DxlError commResultToDxlError(int comm_result)
{
  switch (comm_result) {
    case COMM_SUCCESS:
      return DxlError::SDK_COMM_SUCCESS;
    case COMM_PORT_BUSY:
      return DxlError::SDK_COMM_PORT_BUSY;
    case COMM_TX_FAIL:
      return DxlError::SDK_COMM_TX_FAIL;
    case COMM_RX_FAIL:
      return DxlError::SDK_COMM_RX_FAIL;
    case COMM_TX_ERROR:
      return DxlError::SDK_COMM_TX_ERROR;
    case COMM_RX_WAITING:
      return DxlError::SDK_COMM_RX_WAITING;
    case COMM_RX_TIMEOUT:
      return DxlError::SDK_COMM_RX_TIMEOUT;
    case COMM_RX_CORRUPT:
      return DxlError::SDK_COMM_RX_CORRUPT;
    case COMM_NOT_AVAILABLE:
      return DxlError::SDK_COMM_NOT_AVAILABLE;
    default:
      // Preserve unknown SDK return values (e.g. future COMM_* codes).
      return static_cast<DxlError>(comm_result);
  }
}

DxlError packetErrorToDxlError(uint8_t packet_error)
{
  switch (packet_error) {
    case 1:
      return DxlError::SDK_ERRNUM_RESULT_FAIL;
    case 2:
      return DxlError::SDK_ERRNUM_INSTRUCTION;
    case 3:
      return DxlError::SDK_ERRNUM_CRC;
    case 4:
      return DxlError::SDK_ERRNUM_DATA_RANGE;
    case 5:
      return DxlError::SDK_ERRNUM_DATA_LENGTH;
    case 6:
      return DxlError::SDK_ERRNUM_DATA_LIMIT;
    case 7:
      return DxlError::SDK_ERRNUM_ACCESS;
    default:
      // Preserve non-standard device error codes for logging / debugging.
      return static_cast<DxlError>(static_cast<int>(packet_error));
  }
}

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
    case DxlError::EASY_SDK_TORQUE_STATUS_MISMATCH:
      return "[EasySDKError] Motor torque status mismatch!";
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
    default:
      return "[EasySDKError] Unmapped error code; compare with DynamixelSDK COMM_* / status packet error bits.";
  }
}
}  // namespace dynamixel
