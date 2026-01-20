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

from typing import List, Tuple, Dict, Union, Any

from dynamixel_sdk import *
from dynamixel_easy_sdk.data_types import toSignedInt
from dynamixel_easy_sdk.control_table import ControlTable
from dynamixel_easy_sdk.dynamixel_error import DxlRuntimeError
from dynamixel_easy_sdk.group_strategy import SyncGroupReader, BulkGroupReader, SyncGroupWriter, BulkGroupWriter


class SmartGroupReader:
    def __init__(self, port_handler, packet_handler):
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        # requests: (id, address, length, key)
        self.requests: List[Tuple[int, int, int, str]] = []
        self.data: Dict[str, int] = {}
        self._model_cache: Dict[int, int] = {}

    def _get_model_number(self, dxl_id: int) -> int:
        if dxl_id in self._model_cache:
            return self._model_cache[dxl_id]
        
        model_number, result, error = self.packet_handler.ping(self.port_handler, dxl_id)
        if result != COMM_SUCCESS:
            return 0
        
        self._model_cache[dxl_id] = model_number
        return model_number

    def add(self, dxl_id: int, address_or_name: Union[int, str], length_or_key: Union[int, str], key: str = "") -> 'SmartGroupReader':
        """
        add(id, address, length, key)
        add(id, item_name, key)
        """
        if isinstance(address_or_name, str):
            # Case: add(id, item_name, key)
            item_name = address_or_name
            actual_key = str(length_or_key) if length_or_key else item_name
            
            if actual_key:
                for req in self.requests:
                    if req[3] == actual_key:
                        raise ValueError(f"SmartGroupReader: Duplicate key '{actual_key}' detected.")

            model_number = self._get_model_number(dxl_id)
            if model_number == 0:
                raise DxlRuntimeError(f"Failed to ping motor ID {dxl_id}")
            else:
                try:
                    ct = ControlTable.getControlTable(model_number)
                    if item_name in ct:
                        item = ct[item_name]
                        self.requests.append((dxl_id, item.address, item.size, actual_key))
                    elif item_name.lower() in ct:
                        item = ct[item_name.lower()]
                        self.requests.append((dxl_id, item.address, item.size, actual_key))
                    else:
                        raise DxlRuntimeError(f"Item '{item_name}' not found in Control Table for model {model_number}")
                except Exception as e:
                    raise e
        else:
            # Case: add(id, address, length, key)
            address = address_or_name
            length = int(length_or_key)
            
            if key:
                for req in self.requests:
                    if req[3] == key:
                         raise ValueError(f"SmartGroupReader: Duplicate key '{key}' detected.")

            self.requests.append((dxl_id, address, length, key))
            
        return self

    def clear(self) -> None:
        self.requests.clear()
        self.data.clear()

    def read(self) -> None:
        if not self.requests:
            return

        # Check strategy: SyncRead if all address and length match
        first_addr = self.requests[0][1]
        first_len = self.requests[0][2]
        
        use_sync = True
        for req in self.requests:
            if req[1] != first_addr or req[2] != first_len:
                use_sync = False
                break
        
        reader = None
        if use_sync:
            reader = SyncGroupReader(self.port_handler, self.packet_handler, first_addr, first_len)
        else:
            reader = BulkGroupReader(self.port_handler, self.packet_handler)
            
        for req in self.requests:
            reader.add_param(req[0], req[1], req[2])
            
        reader.read()
        
        self.data.clear()
        for req in self.requests:
            dxl_id, addr, length, k = req
            val = reader.get_data(dxl_id, addr, length)
            signed_val = toSignedInt(val, length)
            if k:
                self.data[k] = signed_val
                
    def get(self, key: str) -> int:
        return self.data.get(key, 0)


class SmartGroupWriter:
    def __init__(self, port_handler, packet_handler):
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        # requests: (id, address, length, data)
        self.requests: List[Tuple[int, int, int, int]] = []
        self._model_cache: Dict[int, int] = {}

    def _get_model_number(self, dxl_id: int) -> int:
        if dxl_id in self._model_cache:
            return self._model_cache[dxl_id]
        
        model_number, result, error = self.packet_handler.ping(self.port_handler, dxl_id)
        if result != COMM_SUCCESS:
            return 0
        
        self._model_cache[dxl_id] = model_number
        return model_number

    def add(self, dxl_id: int, address_or_name: Union[int, str], length_or_data: int, data: int = 0) -> 'SmartGroupWriter':
        """
        add(id, address, length, data)
        add(id, item_name, data)
        """
        if isinstance(address_or_name, str):
            # Case: add(id, item_name, data)
            item_name = address_or_name
            item_data = length_or_data
            
            model_number = self._get_model_number(dxl_id)
            if model_number == 0:
                raise DxlRuntimeError(f"Failed to ping motor ID {dxl_id}")
            
            if model_number != 0:
                try:
                    ct = ControlTable.getControlTable(model_number)
                    if item_name in ct:
                        item = ct[item_name]
                        self.requests.append((dxl_id, item.address, item.size, item_data))
                    elif item_name.lower() in ct:
                        item = ct[item_name.lower()]
                        self.requests.append((dxl_id, item.address, item.size, item_data))
                    else:
                        raise DxlRuntimeError(f"Item '{item_name}' not found in Control Table for model {model_number}")
                except Exception as e:
                    raise e
        else:
             # Case: add(id, address, length, data)
             address = address_or_name
             length = length_or_data
             cmd_data = data
             self.requests.append((dxl_id, address, length, cmd_data))
             
        return self

    def clear(self) -> None:
        self.requests.clear()

    def write(self, values: List[int] = None) -> None:
        if not self.requests:
            return

        if values is not None:
             if len(values) != len(self.requests):
                 # Mismatch size, maybe warn? For now, process min count
                 pass
             
             count = min(len(values), len(self.requests))
             # Rebuild requests with new data
             # Tuple is immutable, so we must recreate list
             for i in range(count):
                 req = self.requests[i]
                 self.requests[i] = (req[0], req[1], req[2], values[i])

        # Check strategy: SyncWrite if all address and length match
        first_addr = self.requests[0][1]
        first_len = self.requests[0][2]
        
        use_sync = True
        for req in self.requests:
            if req[1] != first_addr or req[2] != first_len:
                use_sync = False
                break
        
        writer = None
        if use_sync:
            writer = SyncGroupWriter(self.port_handler, self.packet_handler, first_addr, first_len)
        else:
            writer = BulkGroupWriter(self.port_handler, self.packet_handler)
            
        for req in self.requests:
            writer.add_param(req[0], req[1], req[2], req[3])
            
        writer.write()
