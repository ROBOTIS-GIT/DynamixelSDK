#!/usr/bin/env python3
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Hyungyu Kim

from enum import IntEnum


class DxlErrorCode(IntEnum):
    SDK_COMM_SUCCESS = 0
    SDK_COMM_PORT_BUSY = -1000
    SDK_COMM_TX_FAIL = -1001
    SDK_COMM_RX_FAIL = -1002
    SDK_COMM_TX_ERROR = -2000
    SDK_COMM_RX_WAITING = -3000
    SDK_COMM_RX_TIMEOUT = -3001
    SDK_COMM_RX_CORRUPT = -3002
    SDK_COMM_NOT_AVAILABLE = -9000
    SDK_ERRNUM_RESULT_FAIL = 1
    SDK_ERRNUM_INSTRUCTION = 2
    SDK_ERRNUM_CRC = 3
    SDK_ERRNUM_DATA_RANGE = 4
    SDK_ERRNUM_DATA_LENGTH = 5
    SDK_ERRNUM_DATA_LIMIT = 6
    SDK_ERRNUM_ACCESS = 7
    EASY_SDK_FUNCTION_NOT_SUPPORTED = 11
    EASY_SDK_TORQUE_STATUS_MISMATCH = 12
    EASY_SDK_OPERATING_MODE_MISMATCH = 13
    EASY_SDK_ADD_PARAM_FAIL = 21
    EASY_SDK_COMMAND_IS_EMPTY = 22
    EASY_SDK_DUPLICATE_ID = 23
    EASY_SDK_FAIL_TO_GET_DATA = 24


class DxlRuntimeError(Exception):

    def __init__(self, error_code: DxlErrorCode):
        if isinstance(error_code, DxlErrorCode):
            self.error_code = error_code
            message = getErrorMessage(error_code)
        else:
            self.error_code = None
            message = str(error_code)
        super().__init__(message)


def getErrorMessage(error_code):
    messages = {
        DxlErrorCode.SDK_COMM_SUCCESS:
            '[TxRxResult] Communication success!',
        DxlErrorCode.SDK_COMM_PORT_BUSY:
            '[TxRxResult] Port is in use!',
        DxlErrorCode.SDK_COMM_TX_FAIL:
            '[TxRxResult] Failed to transmit instruction packet',
        DxlErrorCode.SDK_COMM_RX_FAIL:
            '[TxRxResult] Failed to get status packet from device',
        DxlErrorCode.SDK_COMM_TX_ERROR:
            '[TxRxResult] Incorrect instruction packet',
        DxlErrorCode.SDK_COMM_RX_WAITING:
            '[TxRxResult] Receiving status packet',
        DxlErrorCode.SDK_COMM_RX_TIMEOUT:
            '[TxRxResult] No status packet received',
        DxlErrorCode.SDK_COMM_RX_CORRUPT:
            '[TxRxResult] Incorrect status packet',
        DxlErrorCode.SDK_COMM_NOT_AVAILABLE:
            '[TxRxResult] Protocol does not support this function',
        DxlErrorCode.SDK_ERRNUM_RESULT_FAIL:
            '[RxPacketError] Failed to process the instruction packet!',
        DxlErrorCode.SDK_ERRNUM_INSTRUCTION:
            '[RxPacketError] Undefined instruction or incorrect instruction!',
        DxlErrorCode.SDK_ERRNUM_CRC:
            "[RxPacketError] CRC doesn't match!",
        DxlErrorCode.SDK_ERRNUM_DATA_RANGE:
            '[RxPacketError] The data value is out of range!',
        DxlErrorCode.SDK_ERRNUM_DATA_LENGTH:
            '[RxPacketError] The data length does not match as expected!',
        DxlErrorCode.SDK_ERRNUM_DATA_LIMIT:
            '[RxPacketError] The data value exceeds the limit value!',
        DxlErrorCode.SDK_ERRNUM_ACCESS:
            '[RxPacketError] Writing or Reading is not available to target address!',
        DxlErrorCode.EASY_SDK_FUNCTION_NOT_SUPPORTED:
            '[EasySDKUsageError] Function not supported',
        DxlErrorCode.EASY_SDK_TORQUE_STATUS_MISMATCH:
            '[EasySDKUsageError] Motor torque status mismatch',
        DxlErrorCode.EASY_SDK_OPERATING_MODE_MISMATCH:
            '[EasySDKUsageError] Operating mode mismatch',
        DxlErrorCode.EASY_SDK_ADD_PARAM_FAIL:
            '[EasySDKUsageError] Failed to add parameter',
        DxlErrorCode.EASY_SDK_COMMAND_IS_EMPTY:
            '[EasySDKUsageError] No command to execute',
        DxlErrorCode.EASY_SDK_DUPLICATE_ID:
            '[EasySDKUsageError] Duplicate ID in staged commands',
        DxlErrorCode.EASY_SDK_FAIL_TO_GET_DATA:
            '[EasySDKUsageError] Failed to get data from motor',
    }
    message = messages.get(error_code, 'Unknown error code')
    return f'{error_code.name} ({error_code.value}): {message}'
