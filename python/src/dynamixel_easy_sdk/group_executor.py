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
from typing import Optional

from dynamixel_easy_sdk.data_types import CommandType
from dynamixel_easy_sdk.data_types import StagedCommand
from dynamixel_easy_sdk.data_types import StatusRequest
from dynamixel_easy_sdk.data_types import OperatingMode
from dynamixel_easy_sdk.data_types import toSignedInt
from dynamixel_easy_sdk.dynamixel_error import DxlError
from dynamixel_easy_sdk.dynamixel_error import DxlRuntimeError
from dynamixel_sdk import GroupBulkRead
from dynamixel_sdk import GroupBulkWrite
from dynamixel_sdk import GroupSyncRead
from dynamixel_sdk import GroupSyncWrite


class GroupExecutor:

    def __init__(self, connector):
        self.connector = connector
        self.port_handler = connector._port_handler
        self.packet_handler = connector._packet_handler
        self.group_bulk_write = GroupBulkWrite(self.port_handler, self.packet_handler)
        self.group_bulk_read = GroupBulkRead(self.port_handler, self.packet_handler)
        self._staged_write_commands: List[StagedCommand] = []
        self._staged_read_commands: List[StagedCommand] = []

    def addCmd(self, command: StagedCommand):
        if command.command_type == CommandType.WRITE:
            self._staged_write_commands.append(command)
        elif command.command_type == CommandType.READ:
            self._staged_read_commands.append(command)

    def clearStagedWriteCommands(self) -> None:
        self._staged_write_commands.clear()

    def clearStagedReadCommands(self) -> None:
        self._staged_read_commands.clear()

    def executeWrite(self) -> None:
        if not self._staged_write_commands:
            raise DxlRuntimeError(DxlError.EASY_SDK_COMMAND_IS_EMPTY)

        cmds = sorted(self._staged_write_commands, key=lambda c: c.id)
        ids = [cmd.id for cmd in cmds]
        if len(ids) != len(set(ids)):
            raise DxlRuntimeError(DxlError.EASY_SDK_DUPLICATE_ID)

        ref = cmds[0]
        is_sync = all(
            (c.address == ref.address and c.length == ref.length)
            for c in cmds[1:]
        )

        if is_sync:
            self._executeSyncWrite(ref.address, ref.length)
        else:
            self._executeBulkWrite()

    def _executeSyncWrite(self, address: int, length: int) -> None:
        group = GroupSyncWrite(self.port_handler, self.packet_handler, address, length)
        for cmd in self._staged_write_commands:
            self._processStatusRequests(cmd)
            if not group.addParam(cmd.id, bytes(cmd.data)):
                raise DxlRuntimeError(DxlError.EASY_SDK_ADD_PARAM_FAIL)

        dxl_comm_result = group.txPacket()
        if dxl_comm_result != DxlError.SDK_COMM_SUCCESS:
            raise DxlRuntimeError(dxl_comm_result)

    def _executeBulkWrite(self) -> None:
        self.group_bulk_write.clearParam()
        for cmd in self._staged_write_commands:
            self._processStatusRequests(cmd)
            if not self.group_bulk_write.addParam(
                cmd.id,
                cmd.address,
                cmd.length,
                bytes(cmd.data)
            ):
                raise DxlRuntimeError(DxlError.EASY_SDK_ADD_PARAM_FAIL)

        dxl_comm_result = self.group_bulk_write.txPacket()
        if dxl_comm_result != DxlError.SDK_COMM_SUCCESS:
            raise DxlRuntimeError(dxl_comm_result)

    def executeRead(self) -> List[Optional[int]]:
        if not self._staged_read_commands:
            raise DxlRuntimeError(DxlError.EASY_SDK_COMMAND_IS_EMPTY)

        cmds = sorted(self._staged_read_commands, key=lambda c: c.id)
        ids = [cmd.id for cmd in cmds]
        if len(ids) != len(set(ids)):
            raise DxlRuntimeError(DxlError.EASY_SDK_DUPLICATE_ID)

        ref = cmds[0]
        is_sync = all(
            (c.address == ref.address and c.length == ref.length)
            for c in cmds[1:]
        )

        if is_sync:
            return self._executeSyncRead(ref.address, ref.length)
        else:
            return self._executeBulkRead()

    def _executeSyncRead(self, address: int, length: int) -> List[Optional[int]]:
        group = GroupSyncRead(self.port_handler, self.packet_handler, address, length)
        for cmd in self._staged_read_commands:
            if not group.addParam(cmd.id):
                raise DxlRuntimeError(DxlError.EASY_SDK_ADD_PARAM_FAIL)

        dxl_comm_result = group.txRxPacket()
        if dxl_comm_result != DxlError.SDK_COMM_SUCCESS:
            raise DxlRuntimeError(dxl_comm_result)

        results = []
        for cmd in self._staged_read_commands:
            if not group.isAvailable(cmd.id, address, length):
                results.append(None)
                continue
            value = group.getData(cmd.id, address, length)
            self._processStatusRequests(cmd, value)
            intvalue = toSignedInt(value, length)
            results.append(intvalue)
        return results

    def _executeBulkRead(self) -> List[Optional[int]]:
        self.group_bulk_read.clearParam()
        for cmd in self._staged_read_commands:
            if not self.group_bulk_read.addParam(cmd.id, cmd.address, cmd.length):
                raise DxlRuntimeError(DxlError.EASY_SDK_ADD_PARAM_FAIL)

        dxl_comm_result = self.group_bulk_read.txRxPacket()
        if dxl_comm_result != DxlError.SDK_COMM_SUCCESS:
            raise DxlRuntimeError(dxl_comm_result)

        results = []
        for cmd in self._staged_read_commands:
            if not self.group_bulk_read.isAvailable(cmd.id, cmd.address, cmd.length):
                results.append(None)
                continue
            value = self.group_bulk_read.getData(cmd.id, cmd.address, cmd.length)
            self._processStatusRequests(cmd, value)
            intvalue = toSignedInt(value, cmd.length)
            results.append(intvalue)
        return results

    def _processStatusRequests(self, cmd: StagedCommand, data = None) -> None:
        if not cmd.status_request:
            return

        if StatusRequest.CHECK_TORQUE_ON in cmd.status_request:
            if cmd.motor.torque_status != 1:
                raise DxlRuntimeError(DxlError.EASY_SDK_TORQUE_STATUS_MISMATCH)

        if StatusRequest.CHECK_OPERATING_MODE in cmd.status_request:
            if cmd.motor.operating_mode_status not in cmd.allowable_operating_modes:
                raise DxlRuntimeError(DxlError.EASY_SDK_OPERATING_MODE_MISMATCH)

        if StatusRequest.UPDATE_TORQUE_STATUS in cmd.status_request:
            if data is not None:
                cmd.motor.torque_status = data
                return
            cmd.motor.torque_status = cmd.data[0]
