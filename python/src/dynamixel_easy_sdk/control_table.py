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

try:
    from importlib.resources import files
except (ImportError, AttributeError):
    from importlib_resources import files
import os

from dynamixel_easy_sdk.data_types import ControlTableItem
from dynamixel_easy_sdk.dynamixel_error import DxlRuntimeError

CONTROL_TABLE_PATH = files('dynamixel_easy_sdk') / 'control_table'


class ControlTable:
    _model_name_list = None
    _control_tables_cache = {}

    @staticmethod
    def parsingModelList():
        tmp_model_list = {}
        file_name = os.path.join(CONTROL_TABLE_PATH, 'dynamixel.model')
        try:
            with open(file_name, encoding='utf-8') as infile:
                lines = infile.readlines()
        except Exception as e:
            raise RuntimeError(f'Error: Could not open file {file_name}') from e

        for line in lines[1:]:
            if not line.strip():
                continue
            parts = line.strip().split('\t')
            if len(parts) >= 2:
                try:
                    number = int(parts[0])
                    name = parts[1]
                    tmp_model_list[number] = name
                except ValueError:
                    raise RuntimeError(f'Invalid model number in {file_name}: {parts[0]}')
                except Exception as e:
                    raise RuntimeError(
                        f'Unknown error while parsing model list in {file_name}: {e}')
        return tmp_model_list

    @classmethod
    def getModelName(cls, model_number):
        if cls._model_name_list is None:
            cls._model_name_list = cls.parsingModelList()
        if model_number not in cls._model_name_list:
            raise DxlRuntimeError(f'Model number is not found in dynamixel.model: {model_number}')
        return cls._model_name_list[model_number]

    @classmethod
    def getControlTable(cls, model_number):
        if model_number in cls._control_tables_cache:
            return cls._control_tables_cache[model_number]

        model_filename = cls.getModelName(model_number)
        full_path = os.path.join(CONTROL_TABLE_PATH, model_filename)
        control_table = {}

        try:
            with open(full_path, encoding='utf-8') as infile:
                lines = infile.readlines()
                line_iterator = iter(lines)
        except Exception as e:
            raise RuntimeError(f'Error: Could not open model file: {full_path}') from e

        in_section = False
        for line in line_iterator:
            line = line.strip()
            if not line:
                continue
            if line == '[control table]':
                in_section = True
                next(line_iterator, None)
                continue
            if in_section:
                parts = line.split('\t')
                if len(parts) >= 3:
                    try:
                        address = int(parts[0])
                        size = int(parts[1])
                        name = parts[2]
                        control_table[name] = ControlTableItem(address, size)
                    except Exception as e:
                        raise RuntimeError(f'Error parsing control table item: {line} - {e}')
        cls._control_tables_cache[model_number] = control_table
        return control_table
