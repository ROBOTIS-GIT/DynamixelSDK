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


class DxlError(IntEnum):
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

    def __init__(self, dxl_error: DxlError):
        if isinstance(dxl_error, DxlError):
            self.dxl_error = dxl_error
            message = getErrorMessage(dxl_error)
        else:
            self.dxl_error = None
            message = str(dxl_error)
        super().__init__(message)


def getErrorMessage(dxl_error):
    messages = {
        DxlError.SDK_COMM_SUCCESS:
            '[TxRxResult] Communication success!',
        DxlError.SDK_COMM_PORT_BUSY:
            '[TxRxResult] Port is in use!',
        DxlError.SDK_COMM_TX_FAIL:
            '[TxRxResult] Failed to transmit instruction packet',
        DxlError.SDK_COMM_RX_FAIL:
            '[TxRxResult] Failed to get status packet from device',
        DxlError.SDK_COMM_TX_ERROR:
            '[TxRxResult] Incorrect instruction packet',
        DxlError.SDK_COMM_RX_WAITING:
            '[TxRxResult] Receiving status packet',
        DxlError.SDK_COMM_RX_TIMEOUT:
            '[TxRxResult] No status packet received',
        DxlError.SDK_COMM_RX_CORRUPT:
            '[TxRxResult] Incorrect status packet',
        DxlError.SDK_COMM_NOT_AVAILABLE:
            '[TxRxResult] Protocol does not support this function',
        DxlError.SDK_ERRNUM_RESULT_FAIL:
            '[RxPacketError] Failed to process the instruction packet!',
        DxlError.SDK_ERRNUM_INSTRUCTION:
            '[RxPacketError] Undefined instruction or incorrect instruction!',
        DxlError.SDK_ERRNUM_CRC:
            "[RxPacketError] CRC doesn't match!",
        DxlError.SDK_ERRNUM_DATA_RANGE:
            '[RxPacketError] The data value is out of range!',
        DxlError.SDK_ERRNUM_DATA_LENGTH:
            '[RxPacketError] The data length does not match as expected!',
        DxlError.SDK_ERRNUM_DATA_LIMIT:
            '[RxPacketError] The data value exceeds the limit value!',
        DxlError.SDK_ERRNUM_ACCESS:
            '[RxPacketError] Writing or Reading is not available to target address!',
        DxlError.EASY_SDK_FUNCTION_NOT_SUPPORTED:
            '[EasySDKUsageError] Function not supported',
        DxlError.EASY_SDK_TORQUE_STATUS_MISMATCH:
            '[EasySDKUsageError] Motor torque status mismatch',
        DxlError.EASY_SDK_OPERATING_MODE_MISMATCH:
            '[EasySDKUsageError] Operating mode mismatch',
        DxlError.EASY_SDK_ADD_PARAM_FAIL:
            '[EasySDKUsageError] Failed to add parameter',
        DxlError.EASY_SDK_COMMAND_IS_EMPTY:
            '[EasySDKUsageError] No command to execute',
        DxlError.EASY_SDK_DUPLICATE_ID:
            '[EasySDKUsageError] Duplicate ID in staged commands',
        DxlError.EASY_SDK_FAIL_TO_GET_DATA:
            '[EasySDKUsageError] Failed to get data from motor',
    }
    message = messages.get(dxl_error, 'Unknown error code')
    return f'{dxl_error.name} ({dxl_error.value}): {message}'
