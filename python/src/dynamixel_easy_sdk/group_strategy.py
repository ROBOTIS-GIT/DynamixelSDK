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

from abc import ABC, abstractmethod
from typing import List, Tuple, Optional

from dynamixel_sdk import *
from dynamixel_easy_sdk.dynamixel_error import DxlError, DxlRuntimeError


class GroupReader(ABC):
    @abstractmethod
    def add_param(self, dxl_id: int, address: int, length: int) -> None:
        pass

    @abstractmethod
    def clear_param(self) -> None:
        pass

    @abstractmethod
    def read(self) -> None:
        pass

    @abstractmethod
    def get_data(self, dxl_id: int, address: int, length: int) -> int:
        pass


class GroupWriter(ABC):
    @abstractmethod
    def add_param(self, dxl_id: int, address: int, length: int, data: int) -> None:
        pass

    @abstractmethod
    def clear_param(self) -> None:
        pass

    @abstractmethod
    def write(self) -> None:
        pass


class SyncGroupReader(GroupReader):
    def __init__(self, port_handler, packet_handler, address: int, length: int):
        self.group_sync_read = GroupSyncRead(port_handler, packet_handler, address, length)

    def add_param(self, dxl_id: int, address: int, length: int) -> None:
        # SyncRead ignores individual address/length as they are fixed in constructor
        # But we check for consistency or just ignore assuming the caller knows
        self.group_sync_read.addParam(dxl_id)

    def clear_param(self) -> None:
        self.group_sync_read.clearParam()

    def read(self) -> None:
        comm_result = self.group_sync_read.txRxPacket()
        if comm_result != COMM_SUCCESS:
            try:
                error = DxlError(comm_result)
            except ValueError:
                # Fallback if comm_result is not in DxlError enum
                # DxlRuntimeError handles non-DxlError types gracefully
                error = comm_result 
            raise DxlRuntimeError(error)

    def get_data(self, dxl_id: int, address: int, length: int) -> int:
        if self.group_sync_read.isAvailable(dxl_id, address, length):
            return self.group_sync_read.getData(dxl_id, address, length)
        return 0


class BulkGroupReader(GroupReader):
    def __init__(self, port_handler, packet_handler):
        self.group_bulk_read = GroupBulkRead(port_handler, packet_handler)

    def add_param(self, dxl_id: int, address: int, length: int) -> None:
        self.group_bulk_read.addParam(dxl_id, address, length)

    def clear_param(self) -> None:
        self.group_bulk_read.clearParam()

    def read(self) -> None:
        comm_result = self.group_bulk_read.txRxPacket()
        if comm_result != COMM_SUCCESS:
            try:
                error = DxlError(comm_result)
            except ValueError:
                error = comm_result
            raise DxlRuntimeError(error)

    def get_data(self, dxl_id: int, address: int, length: int) -> int:
        if self.group_bulk_read.isAvailable(dxl_id, address, length):
            return self.group_bulk_read.getData(dxl_id, address, length)
        return 0


class SyncGroupWriter(GroupWriter):
    def __init__(self, port_handler, packet_handler, address: int, length: int):
        self.group_sync_write = GroupSyncWrite(port_handler, packet_handler, address, length)

    def add_param(self, dxl_id: int, address: int, length: int, data: int) -> None:
        data_bytes = [(data >> (i * 8)) & 0xFF for i in range(length)]
        self.group_sync_write.addParam(dxl_id, data_bytes)

    def clear_param(self) -> None:
        self.group_sync_write.clearParam()

    def write(self) -> None:
        comm_result = self.group_sync_write.txPacket()
        if comm_result != COMM_SUCCESS:
            try:
                error = DxlError(comm_result)
            except ValueError:
                error = comm_result
            raise DxlRuntimeError(error)


class BulkGroupWriter(GroupWriter):
    def __init__(self, port_handler, packet_handler):
        self.group_bulk_write = GroupBulkWrite(port_handler, packet_handler)

    def add_param(self, dxl_id: int, address: int, length: int, data: int) -> None:
        data_bytes = [(data >> (i * 8)) & 0xFF for i in range(length)]
        self.group_bulk_write.addParam(dxl_id, address, length, data_bytes)

    def clear_param(self) -> None:
        self.group_bulk_write.clearParam()

    def write(self) -> None:
        comm_result = self.group_bulk_write.txPacket()
        if comm_result != COMM_SUCCESS:
            try:
                error = DxlError(comm_result)
            except ValueError:
                error = comm_result
            raise DxlRuntimeError(error)
