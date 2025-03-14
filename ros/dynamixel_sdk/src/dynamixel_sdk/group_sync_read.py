#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
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

# Author: Ryu Woon Jung (Leon), Wonho Yun

from .robotis_def import *


class GroupSyncRead:
    def __init__(self, port, ph, start_address, data_length):
        self.port = port
        self.ph = ph
        self.start_address = start_address
        self.data_length = data_length

        self.last_result = False
        self.is_param_changed = False
        self.param = []
        self.data_dict = {}

        self.clearParam()

    def makeParam(self):
        if self.ph.getProtocolVersion() == 1.0:
            return

        if not self.is_param_changed:
            return

        if not self.data_dict:
            return

        self.param = list(self.data_dict.keys())

        self.is_param_changed = False


    def addParam(self, dxl_id):
        if self.ph.getProtocolVersion() == 1.0:
            return False

        if dxl_id in self.data_dict:
            return False

        self.data_dict[dxl_id] = []

        self.is_param_changed = True
        return True

    def removeParam(self, dxl_id):
        if self.ph.getProtocolVersion() == 1.0:
            return

        if dxl_id not in self.data_dict:
            return

        del self.data_dict[dxl_id]

        self.is_param_changed = True

    def clearParam(self):
        if self.ph.getProtocolVersion() == 1.0:
            return

        self.data_dict.clear()

    def txPacket(self):
        if self.ph.getProtocolVersion() == 1.0 or len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        return self.ph.syncReadTx(
            self.port,
            self.start_address,
            self.data_length,
            self.param,
            len(self.data_dict.keys()) * 1,
            False)

    def fastSyncReadTxPacket(self):
        if self.ph.getProtocolVersion() == 1.0 or len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        return self.ph.syncReadTx(
            self.port,
            self.start_address,
            self.data_length,
            self.param,
            len(self.data_dict.keys()) * 1,
            True)

    def rxPacket(self):
        self.last_result = False

        if self.ph.getProtocolVersion() == 1.0:
            return COMM_NOT_AVAILABLE

        result = COMM_RX_FAIL

        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        for dxl_id in self.data_dict:
            self.data_dict[dxl_id], result, _ = self.ph.readRx(self.port, dxl_id, self.data_length)
            if result != COMM_SUCCESS:
                return result

        if result == COMM_SUCCESS:
            self.last_result = True

        return result

    def fastSyncReadRxPacket(self):
        self.last_result = False

        if self.ph.getProtocolVersion() == 1.0:
            return COMM_NOT_AVAILABLE

        if not self.data_dict:
            return COMM_NOT_AVAILABLE

        num_devices = len(self.data_dict)
        rx_param_length = (self.data_length + 4) * num_devices  # Error(1) + ID(1) + Data(N) + CRC(2))
        raw_data, result, _ = self.ph.fastSyncReadRx(self.port, BROADCAST_ID, rx_param_length)
        if result != COMM_SUCCESS:
            return result

        raw_data = bytearray(raw_data)
        start_index = 0

        valid_ids = set(self.data_dict.keys())
        for _ in range(num_devices):
            dxl_id = raw_data[start_index + 1]
            if dxl_id not in valid_ids:
                return COMM_RX_CORRUPT

            self.data_dict[dxl_id] = bytearray(raw_data[start_index + 2 : start_index + 2 + self.data_length])
            start_index += self.data_length + 4

        self.last_result = True
        return COMM_SUCCESS

    def txRxPacket(self):
        if self.ph.getProtocolVersion() == 1.0:
            return COMM_NOT_AVAILABLE

        if (result := self.txPacket()) != COMM_SUCCESS:
            return result

        return self.rxPacket()

    def fastSyncRead(self):
        if self.ph.getProtocolVersion() == 1.0:
            return COMM_NOT_AVAILABLE

        result = self.fastSyncReadTxPacket()
        if result != COMM_SUCCESS:
            return result

        return self.fastSyncReadRxPacket()

    def isAvailable(self, dxl_id, address, data_length):
        if self.ph.getProtocolVersion() == 1.0 or self.last_result is False or dxl_id not in self.data_dict:
            return False

        if (address < self.start_address) or (self.start_address + self.data_length - data_length < address):
            return False

        return True

    def getData(self, dxl_id, address, data_length):
        if not self.isAvailable(dxl_id, address, data_length):
            return 0

        start_idx = address - self.start_address
        data = self.data_dict[dxl_id]

        if data_length == 1:
            return data[start_idx]
        elif data_length == 2:
            return (data[start_idx] | (data[start_idx + 1] << 8))
        elif data_length == 4:
            return ((data[start_idx] | (data[start_idx + 1] << 8)) |
                    ((data[start_idx + 2] | (data[start_idx + 3] << 8)) << 16))
        else:
            return 0
