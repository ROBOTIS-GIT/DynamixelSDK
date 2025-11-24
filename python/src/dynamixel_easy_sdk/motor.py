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

from dynamixel_easy_sdk.control_table import ControlTable
from dynamixel_easy_sdk.data_types import (
    CommandType,
    ControlTableItem,
    Direction,
    OperatingMode,
    ProfileConfiguration,
    StagedCommand,
    StatusRequest,
)
from dynamixel_easy_sdk.data_types import toSignedInt
from dynamixel_easy_sdk.dynamixel_error import DxlError
from dynamixel_easy_sdk.dynamixel_error import DxlRuntimeError


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
        item = self._getControlTableItem('Torque Enable')
        self._writeData(self.id, item.address, item.size, 1)
        self.torque_status = 1

    def disableTorque(self) -> None:
        item = self._getControlTableItem('Torque Enable')
        self._writeData(self.id, item.address, item.size, 0)
        self.torque_status = 0

    def setGoalPosition(self, position: int) -> None:
        self._checkTorqueStatus(1)
        self._checkOperatingModeStatus([OperatingMode.POSITION, OperatingMode.EXTENDED_POSITION, OperatingMode.CURRENT_BASED_POSITION])
        item = self._getControlTableItem('Goal Position')
        self._writeData(self.id, item.address, item.size, position)

    def setGoalVelocity(self, velocity: int) -> None:
        self._checkTorqueStatus(1)
        self._checkOperatingModeStatus([OperatingMode.VELOCITY])
        item = self._getControlTableItem('Goal Velocity')
        self._writeData(self.id, item.address, item.size, velocity)

    def setGoalCurrent(self, current: int) -> None:
        self._checkTorqueStatus(1)
        self._checkOperatingModeStatus([OperatingMode.CURRENT, OperatingMode.CURRENT_BASED_POSITION])
        item = self._getControlTableItem('Goal Current')
        self._writeData(self.id, item.address, item.size, current)

    def setGoalPWM(self, pwm: int) -> None:
        self._checkTorqueStatus(1)
        item = self._getControlTableItem('Goal PWM')
        self._writeData(self.id, item.address, item.size, pwm)

    def LEDOn(self) -> None:
        item = self._getControlTableItem('LED')
        self._writeData(self.id, item.address, item.size, 1)

    def LEDOff(self) -> None:
        item = self._getControlTableItem('LED')
        self._writeData(self.id, item.address, item.size, 0)

    def ping(self) -> int:
        value = self.connector.ping(self.id)
        return value

    def isTorqueOn(self) -> int:
        item = self._getControlTableItem('Torque Enable')
        self.torque_status = self._readData(self.id, item.address, item.size)
        return self.torque_status

    def isLEDOn(self) -> int:
        item = self._getControlTableItem('LED')
        value = self._readData(self.id, item.address, item.size)
        return value

    def getPresentPosition(self) -> int:
        item = self._getControlTableItem('Present Position')
        unsigned_value = self._readData(self.id, item.address, item.size)
        return toSignedInt(unsigned_value, item.size)

    def getPresentVelocity(self) -> int:
        item = self._getControlTableItem('Present Velocity')
        unsigned_value = self._readData(self.id, item.address, item.size)
        return toSignedInt(unsigned_value, item.size)

    def getPresentCurrent(self) -> int:
        item = self._getControlTableItem('Present Current')
        unsigned_value = self._readData(self.id, item.address, item.size)
        return toSignedInt(unsigned_value, item.size)

    def getPresentPWM(self) -> int:
        item = self._getControlTableItem('Present PWM')
        unsigned_value = self._readData(self.id, item.address, item.size)
        return toSignedInt(unsigned_value, item.size)

    def getMaxPositionLimit(self) -> int:
        item = self._getControlTableItem('Max Position Limit')
        value = self._readData(self.id, item.address, item.size)
        return value

    def getMinPositionLimit(self) -> int:
        item = self._getControlTableItem('Min Position Limit')
        value = self._readData(self.id, item.address, item.size)
        return value

    def getVelocityLimit(self) -> int:
        item = self._getControlTableItem('Velocity Limit')
        value = self._readData(self.id, item.address, item.size)
        return value

    def getCurrentLimit(self) -> int:
        item = self._getControlTableItem('Current Limit')
        value = self._readData(self.id, item.address, item.size)
        return value

    def getPWMLimit(self) -> int:
        item = self._getControlTableItem('PWM Limit')
        value = self._readData(self.id, item.address, item.size)
        return value

    def getOperatingMode(self) -> OperatingMode:
        item = self._getControlTableItem('Operating Mode')
        value = self._readData(self.id, item.address, item.size)
        self.operating_mode_status = OperatingMode(value)
        return self.operating_mode_status

    def changeID(self, newId: int) -> None:
        self._checkTorqueStatus(0)
        item = self._getControlTableItem('ID')
        self._writeData(self.id, item.address, item.size, newId)
        self.id = newId

    def setOperatingMode(self, mode: OperatingMode) -> None:
        self._checkTorqueStatus(0)
        item = self._getControlTableItem('Operating Mode')
        self._writeData(self.id, item.address, item.size, int(mode))
        self.operating_mode_status = mode

    def setProfileConfiguration(self, config: ProfileConfiguration) -> None:
        self._checkTorqueStatus(0)
        item = self._getControlTableItem('Drive Mode')
        mode_value = self._readData(self.id, item.address, item.size)

        PROFILE_BIT = 0b00000100
        if config == ProfileConfiguration.TIME_BASED:
            mode_value |= PROFILE_BIT
        else:
            mode_value &= ~PROFILE_BIT

        self._writeData(self.id, item.address, item.size, mode_value)

    def setDirection(self, direction: Direction) -> None:
        self._checkTorqueStatus(0)
        item = self._getControlTableItem('Drive Mode')
        mode_value = self._readData(self.id, item.address, item.size)

        DIR_BIT = 0b00000001
        if direction == Direction.NORMAL:
            mode_value &= ~DIR_BIT
        else:
            mode_value |= DIR_BIT

        self._writeData(self.id, item.address, item.size, mode_value)

    def setPositionPGain(self, value: int) -> None: self._writeGain('Position P Gain', value)
    def setPositionIGain(self, value: int) -> None: self._writeGain('Position I Gain', value)
    def setPositionDGain(self, value: int) -> None: self._writeGain('Position D Gain', value)
    def setVelocityPGain(self, value: int) -> None: self._writeGain('Velocity P Gain', value)
    def setVelocityIGain(self, value: int) -> None: self._writeGain('Velocity I Gain', value)

    def setHomingOffset(self, offset: int) -> None:
        self._checkTorqueStatus(0)
        item = self._getControlTableItem('Homing Offset')
        self._writeData(self.id, item.address, item.size, offset)

    def setMaxPositionLimit(self, limit: int) -> None:
        self._checkTorqueStatus(0)
        item = self._getControlTableItem('Max Position Limit')
        self._writeData(self.id, item.address, item.size, limit)

    def setMinPositionLimit(self, limit: int) -> None:
        self._checkTorqueStatus(0)
        item = self._getControlTableItem('Min Position Limit')
        self._writeData(self.id, item.address, item.size, limit)

    def setVelocityLimit(self, limit: int) -> None:
        self._checkTorqueStatus(0)
        item = self._getControlTableItem('Velocity Limit')
        self._writeData(self.id, item.address, item.size, limit)

    def setCurrentLimit(self, limit: int) -> None:
        self._checkTorqueStatus(0)
        item = self._getControlTableItem('Current Limit')
        self._writeData(self.id, item.address, item.size, limit)

    def setPWMLimit(self, limit: int) -> None:
        self._checkTorqueStatus(0)
        item = self._getControlTableItem('PWM Limit')
        self._writeData(self.id, item.address, item.size, limit)

    def reboot(self) -> None:
        self.connector.reboot(self.id)

    def factoryResetAll(self) -> None:
        self.connector.factoryReset(self.id, 0xFF)

    def factoryResetExceptID(self) -> None:
        self.connector.factoryReset(self.id, 0x01)

    def factoryResetExceptIDAndBaudRate(self) -> None:
        self.connector.factoryReset(self.id, 0x02)

    def stageEnableTorque(self) -> StagedCommand:
        item = self._getControlTableItem('Torque Enable')
        return StagedCommand(
            CommandType.WRITE,
            self.id,
            item.address,
            item.size,
            [1],
            [StatusRequest.UPDATE_TORQUE_STATUS],
            self
        )

    def stageDisableTorque(self) -> StagedCommand:
        item = self._getControlTableItem('Torque Enable')
        return StagedCommand(
            CommandType.WRITE,
            self.id,
            item.address,
            item.size,
            [0],
            [StatusRequest.UPDATE_TORQUE_STATUS],
            self
        )

    def stageSetGoalPosition(self, position: int) -> StagedCommand:
        item = self._getControlTableItem('Goal Position')
        data = []
        for i in range(item.size):
            data.append((position >> (8 * i)) & 0xFF)
        return StagedCommand(
            CommandType.WRITE,
            self.id,
            item.address,
            item.size,
            data,
            [StatusRequest.CHECK_TORQUE_ON, StatusRequest.CHECK_OPERATING_MODE],
            self,
            [OperatingMode.POSITION, OperatingMode.EXTENDED_POSITION, OperatingMode.CURRENT_BASED_POSITION]
        )

    def stageSetGoalVelocity(self, velocity: int) -> StagedCommand:
        item = self._getControlTableItem('Goal Velocity')
        data = []
        for i in range(item.size):
            data.append((velocity >> (8 * i)) & 0xFF)
        return StagedCommand(
            CommandType.WRITE,
            self.id,
            item.address,
            item.size,
            data,
            [StatusRequest.CHECK_TORQUE_ON, StatusRequest.CHECK_OPERATING_MODE],
            self,
            [OperatingMode.VELOCITY]
        )

    def stageSetGoalCurrent(self, current: int) -> StagedCommand:
        item = self._getControlTableItem('Goal Current')
        data = []
        for i in range(item.size):
            data.append((current >> (8 * i)) & 0xFF)
        return StagedCommand(
            CommandType.WRITE,
            self.id,
            item.address,
            item.size,
            data,
            [StatusRequest.CHECK_TORQUE_ON, StatusRequest.CHECK_OPERATING_MODE],
            self,
            [OperatingMode.CURRENT, OperatingMode.CURRENT_BASED_POSITION]
        )

    def stageSetGoalPWM(self, pwm: int) -> StagedCommand:
        item = self._getControlTableItem('Goal PWM')
        data = []
        for i in range(item.size):
            data.append((pwm >> (8 * i)) & 0xFF)
        return StagedCommand(
            CommandType.WRITE,
            self.id,
            item.address,
            item.size,
            data,
            [StatusRequest.CHECK_TORQUE_ON],
            self
        )

    def stageLEDOn(self) -> StagedCommand:
        item = self._getControlTableItem('LED')
        return StagedCommand(CommandType.WRITE, self.id, item.address, item.size, [1])

    def stageLEDOff(self) -> StagedCommand:
        item = self._getControlTableItem('LED')
        return StagedCommand(CommandType.WRITE, self.id, item.address, item.size, [0])

    def stageIsTorqueOn(self) -> StagedCommand:
        item = self._getControlTableItem('Torque Enable')
        return StagedCommand(
            CommandType.READ,
            self.id,
            item.address,
            item.size,
            [],
            [StatusRequest.UPDATE_TORQUE_STATUS],
            self
        )

    def stageIsLEDOn(self) -> StagedCommand:
        item = self._getControlTableItem('LED')
        return StagedCommand(CommandType.READ, self.id, item.address, item.size, [])

    def stageGetPresentPosition(self) -> StagedCommand:
        item = self._getControlTableItem('Present Position')
        return StagedCommand(CommandType.READ, self.id, item.address, item.size, [])

    def stageGetPresentVelocity(self) -> StagedCommand:
        item = self._getControlTableItem('Present Velocity')
        return StagedCommand(CommandType.READ, self.id, item.address, item.size, [])

    def stageGetPresentCurrent(self) -> StagedCommand:
        item = self._getControlTableItem('Present Current')
        return StagedCommand(CommandType.READ, self.id, item.address, item.size, [])

    def stageGetPresentPWM(self) -> StagedCommand:
        item = self._getControlTableItem('Present PWM')
        return StagedCommand(CommandType.READ, self.id, item.address, item.size, [])

    def _getControlTableItem(self, name) -> ControlTableItem:
        item = self.control_table.get(name)
        if item is None:
            raise DxlRuntimeError(DxlError.EASY_SDK_FUNCTION_NOT_SUPPORTED)
        return item

    def _readData(self, dxl_id: int, address: int, length: int):
        if length == 1:
            return self.connector.read1ByteData(dxl_id, address)
        elif length == 2:
            return self.connector.read2ByteData(dxl_id, address)
        elif length == 4:
            return self.connector.read4ByteData(dxl_id, address)
        else:
            raise DxlRuntimeError(DxlError.EASY_SDK_FUNCTION_NOT_SUPPORTED)

    def _writeData(self, dxl_id: int, address: int, length: int, value: int):
        if length == 1:
            return self.connector.write1ByteData(dxl_id, address, value)
        elif length == 2:
            return self.connector.write2ByteData(dxl_id, address, value)
        elif length == 4:
            return self.connector.write4ByteData(dxl_id, address, value)
        else:
            raise DxlRuntimeError(DxlError.EASY_SDK_FUNCTION_NOT_SUPPORTED)

    def _checkTorqueStatus(self, status: int):
        if self.torque_status != status:
            raise DxlRuntimeError(DxlError.EASY_SDK_TORQUE_STATUS_MISMATCH)

    def _checkOperatingModeStatus(self, mode: list[OperatingMode]):
        if self.operating_mode_status not in mode:
            raise DxlRuntimeError(DxlError.EASY_SDK_OPERATING_MODE_MISMATCH)

    def _writeGain(self, name, value):
        self._checkTorqueStatus(0)
        item = self._getControlTableItem(name)
        self._writeData(self.id, item.address, item.size, value)
