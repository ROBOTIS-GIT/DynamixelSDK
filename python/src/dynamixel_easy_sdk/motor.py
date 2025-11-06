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

from dynamixel_easy_sdk.dynamixel_error import *
from dynamixel_easy_sdk.control_table import ControlTable
from enum import IntEnum

class OperatingMode(IntEnum):
    POSITION = 3
    VELOCITY = 1
    CURRENT = 0
    PWM = 16
    EXTENDED_POSITION = 4

class Direction(IntEnum):
    NORMAL = 0
    REVERSE = 1

class ProfileConfiguration(IntEnum):
    VELOCITY_BASED = 0
    TIME_BASED = 1

class Motor:
    def __init__(self, motor_id: int, model_number: int, connector):
        self.id = motor_id
        self.model_number = model_number
        self.model_name = ControlTable.getModelName(model_number)
        self.connector = connector
        self.control_table = ControlTable.getControlTable(model_number)
        self.torque_status = self.isTorqueOn()
        self.operating_mode_status = self.getOperatingMode()

    def enableTorque(self) -> None:
        item = self._getControlTableItem("Torque Enable")
        self.connector.writeData(self.id, item.address, item.size, 1)
        self.torque_status = 1

    def disableTorque(self) -> None:
        item = self._getControlTableItem("Torque Enable")
        self.connector.writeData(self.id, item.address, item.size, 0)
        self.torque_status = 0

    def setGoalPosition(self, position: int) -> None:
        self._checkTorqueStatus(1)
        if self.operating_mode_status != OperatingMode.POSITION and self.operating_mode_status != OperatingMode.EXTENDED_POSITION:
            raise DxlRuntimeError(DxlErrorCode.EASY_SDK_OPERATING_MODE_MISMATCH)
        item = self._getControlTableItem("Goal Position")
        self.connector.writeData(self.id, item.address, item.size, position)

    def setGoalVelocity(self, velocity: int) -> None:
        self._checkTorqueStatus(1)
        self._checkOperatingModeStatus(OperatingMode.VELOCITY)
        item = self._getControlTableItem("Goal Velocity")
        self.connector.writeData(self.id, item.address, item.size, velocity)

    def setGoalCurrent(self, current: int) -> None:
        self._checkTorqueStatus(1)
        self._checkOperatingModeStatus(OperatingMode.CURRENT)
        item = self._getControlTableItem("Goal Current")
        self.connector.writeData(self.id, item.address, item.size, current)

    def setGoalPwm(self, pwm: int) -> None:
        self._checkTorqueStatus(1)
        self._checkOperatingModeStatus(OperatingMode.PWM)
        item = self._getControlTableItem("Goal PWM")
        self.connector.writeData(self.id, item.address, item.size, pwm)

    def ledOn(self) -> None:
        item = self._getControlTableItem("LED")
        self.connector.writeData(self.id, item.address, item.size, 1)

    def ledOff(self) -> None:
        item = self._getControlTableItem("LED")
        self.connector.writeData(self.id, item.address, item.size, 0)

    def ping(self) -> int:
        value = self.connector.ping(self.id)
        return value

    def isTorqueOn(self) -> int:
        item = self._getControlTableItem("Torque Enable")
        self.torque_status = self.connector.readData(self.id, item.address, item.size)
        return self.torque_status

    def isLedOn(self) -> int:
        item = self._getControlTableItem("LED")
        value = self.connector.readData(self.id, item.address, item.size)
        return value

    def getPresentPosition(self) -> int:
        item = self._getControlTableItem("Present Position")
        unsigned_value = self.connector.readData(self.id, item.address, item.size)
        return self._toSignedInt(unsigned_value, item.size)

    def getPresentVelocity(self) -> int:
        item = self._getControlTableItem("Present Velocity")
        unsigned_value = self.connector.readData(self.id, item.address, item.size)
        return self._toSignedInt(unsigned_value, item.size)

    def getMaxPositionLimit(self) -> int:
        item = self._getControlTableItem("Max Position Limit")
        value = self.connector.readData(self.id, item.address, item.size)
        return value

    def getMinPositionLimit(self) -> int:
        item = self._getControlTableItem("Min Position Limit")
        value = self.connector.readData(self.id, item.address, item.size)
        return value

    def getVelocityLimit(self) -> int:
        item = self._getControlTableItem("Velocity Limit")
        value = self.connector.readData(self.id, item.address, item.size)
        return value

    def getCurrentLimit(self) -> int:
        item = self._getControlTableItem("Current Limit")
        value = self.connector.readData(self.id, item.address, item.size)
        return value

    def getPwmLimit(self) -> int:
        item = self._getControlTableItem("PWM Limit")
        value = self.connector.readData(self.id, item.address, item.size)
        return value

    def getOperatingMode(self) -> OperatingMode:
        item = self._getControlTableItem("Operating Mode")
        value = self.connector.readData(self.id, item.address, item.size)
        self.operating_mode_status = OperatingMode(value)
        return self.operating_mode_status

    def changeId(self, newId: int) -> None:
        self._checkTorqueStatus(0)
        item = self._getControlTableItem("ID")
        self.connector.writeData(self.id, item.address, item.size, newId)
        self.id = newId

    def setOperatingMode(self, mode: OperatingMode) -> None:
        self._checkTorqueStatus(0)
        item = self._getControlTableItem("Operating Mode")
        self.connector.writeData(self.id, item.address, item.size, int(mode))
        self.operating_mode_status = mode

    def setProfileConfiguration(self, config: ProfileConfiguration) -> None:
        self._checkTorqueStatus(0)
        item = self._getControlTableItem("Drive Mode")
        mode_value = self.connector.readData(self.id, item.address, item.size)

        PROFILE_BIT = 0b00000100
        if config == ProfileConfiguration.TIME_BASED:
            mode_value |= PROFILE_BIT
        else:
            mode_value &= ~PROFILE_BIT

        self.connector.writeData(self.id, item.address, item.size, mode_value)

    def setDirection(self, direction: Direction) -> None:
        self._checkTorqueStatus(0)
        item = self._getControlTableItem("Drive Mode")
        mode_value = self.connector.readData(self.id, item.address, item.size)

        DIR_BIT = 0b00000001
        if direction == Direction.NORMAL:
            mode_value &= ~DIR_BIT
        else:
            mode_value |= DIR_BIT

        self.connector.writeData(self.id, item.address, item.size, mode_value)

    def setPositionPGain(self, value: int) -> None: self._writeGain("Position P Gain", value)
    def setPositionIGain(self, value: int) -> None: self._writeGain("Position I Gain", value)
    def setPositionDGain(self, value: int) -> None: self._writeGain("Position D Gain", value)
    def setVelocityPGain(self, value: int) -> None: self._writeGain("Velocity P Gain", value)
    def setVelocityIGain(self, value: int) -> None: self._writeGain("Velocity I Gain", value)

    def _getControlTableItem(self, name):
        item = self.control_table.get(name)
        if item is None:
            raise DxlRuntimeError(DxlErrorCode.EASY_SDK_FUNCTION_NOT_SUPPORTED)
        return item

    def _checkTorqueStatus(self, status: int):
        if self.torque_status != status:
            raise DxlRuntimeError(DxlErrorCode.EASY_SDK_TORQUE_STATUS_MISMATCH)

    def _checkOperatingModeStatus(self, mode: OperatingMode):
        if self.operating_mode_status != mode:
            raise DxlRuntimeError(DxlErrorCode.EASY_SDK_OPERATING_MODE_MISMATCH)

    def _writeGain(self, name, value):
        self._checkTorqueStatus(0)
        item = self._getControlTableItem(name)
        self.connector.writeData(self.id, item.address, item.size, value)

    def _toSignedInt(self, value: int, size: int) -> int:
        bits = size * 8
        if value >= (1 << (bits - 1)):
            value -= (1 << bits)
        return value
