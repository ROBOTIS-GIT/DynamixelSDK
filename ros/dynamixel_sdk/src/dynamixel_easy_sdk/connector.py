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

from typing import List
import serial

from dynamixel_easy_sdk.dynamixel_error import DxlError
from dynamixel_easy_sdk.dynamixel_error import DxlRuntimeError
from dynamixel_easy_sdk.group_executor import GroupExecutor
from dynamixel_easy_sdk.motor import Motor
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler


class Connector:

    PROTOCOL_VERSION = 2.0
    _packet_handler = None

    def __init__(self, port_name: str, baud_rate: int):
        self._port_handler = PortHandler(port_name)
        if Connector._packet_handler is None:
            Connector._packet_handler = PacketHandler(Connector.PROTOCOL_VERSION)

        try:
            if not self._port_handler.setBaudRate(baud_rate):
                raise DxlRuntimeError('Invalid baudrate specified')
        except serial.SerialException as e:
            errno = getattr(e, 'errno', None)
            if errno == 2:
                raise DxlRuntimeError(f'Port "{port_name}" does not exist') from e
            elif errno == 16:
                raise DxlRuntimeError(DxlError.SDK_COMM_PORT_BUSY) from e
            else:
                raise DxlRuntimeError('Failed to open port') from e
        except Exception as e:
            raise DxlRuntimeError('Failed to open port') from e

    def createMotor(self, motor_id: int) -> Motor:
        model_number = self.ping(motor_id)
        return Motor(motor_id, model_number, self)

    def createAllMotors(self, start_id: int = 0, end_id: int = 252) -> List[Motor]:
        if not (0 <= start_id <= 252 and 0 <= end_id <= 252 and start_id <= end_id):
            raise DxlRuntimeError('ID must be between 0 and 252, and start_id <= end_id')

        motor_ids = self.broadcastPing()
        motors = []
        for motor_id in motor_ids:
            if start_id <= motor_id <= end_id:
                motors.append(self.createMotor(motor_id))
        return motors

    def createGroupExecutor(self):
        return GroupExecutor(self)

    def _checkError(self, dxl_comm_result, dxl_error):
        if dxl_comm_result != DxlError.SDK_COMM_SUCCESS:
            raise DxlRuntimeError(DxlError(dxl_comm_result))
        if dxl_error != DxlError.SDK_COMM_SUCCESS:
            raise DxlRuntimeError(DxlError(dxl_error))

    def read1ByteData(self, motor_id: int, address: int) -> int:
        value, dxl_comm_result, dxl_error = Connector._packet_handler.read1ByteTxRx(
            self._port_handler,
            motor_id,
            address)
        self._checkError(dxl_comm_result, dxl_error)
        return value

    def read2ByteData(self, motor_id: int, address: int) -> int:
        value, dxl_comm_result, dxl_error = Connector._packet_handler.read2ByteTxRx(
            self._port_handler,
            motor_id,
            address)
        self._checkError(dxl_comm_result, dxl_error)
        return value

    def read4ByteData(self, motor_id: int, address: int) -> int:
        value, dxl_comm_result, dxl_error = Connector._packet_handler.read4ByteTxRx(
            self._port_handler,
            motor_id,
            address)
        self._checkError(dxl_comm_result, dxl_error)
        return value

    def write1ByteData(self, motor_id: int, address: int, value: int):
        dxl_comm_result, dxl_error = Connector._packet_handler.write1ByteTxRx(
            self._port_handler,
            motor_id,
            address,
            value)
        self._checkError(dxl_comm_result, dxl_error)

    def write2ByteData(self, motor_id: int, address: int, value: int):
        dxl_comm_result, dxl_error = Connector._packet_handler.write2ByteTxRx(
            self._port_handler,
            motor_id,
            address,
            value)
        self._checkError(dxl_comm_result, dxl_error)

    def write4ByteData(self, motor_id: int, address: int, value: int):
        dxl_comm_result, dxl_error = Connector._packet_handler.write4ByteTxRx(
            self._port_handler,
            motor_id,
            address,
            value)
        self._checkError(dxl_comm_result, dxl_error)

    def reboot(self, motor_id: int):
        dxl_comm_result, dxl_error = Connector._packet_handler.reboot(self._port_handler, motor_id)
        self._checkError(dxl_comm_result, dxl_error)

    def ping(self, motor_id: int) -> int:
        model_number, dxl_comm_result, dxl_error = Connector._packet_handler.ping(
            self._port_handler,
            motor_id)
        self._checkError(dxl_comm_result, dxl_error)
        return model_number

    def broadcastPing(self) -> List[int]:
        ids, dxl_comm_result = Connector._packet_handler.broadcastPing(self._port_handler)
        self._checkError(dxl_comm_result, DxlError.SDK_COMM_SUCCESS)
        return ids

    def factoryReset(self, motor_id: int, option: int):
        dxl_comm_result, dxl_error = Connector._packet_handler.factoryReset(
            self._port_handler,
            motor_id,
            option)
        self._checkError(dxl_comm_result, dxl_error)

    def closePort(self):
        self._port_handler.closePort()
