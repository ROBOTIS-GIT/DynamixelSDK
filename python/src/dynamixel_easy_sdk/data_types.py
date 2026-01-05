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

from dataclasses import dataclass
from enum import IntEnum
from typing import List
from typing import Optional


@dataclass
class ControlTableItem:
    address: int
    size: int


class OperatingMode(IntEnum):
    CURRENT = 0
    VELOCITY = 1
    POSITION = 3
    EXTENDED_POSITION = 4
    CURRENT_BASED_POSITION = 5
    PWM = 16


class Direction(IntEnum):
    NORMAL = 0
    REVERSE = 1


class ProfileConfiguration(IntEnum):
    VELOCITY_BASED = 0
    TIME_BASED = 1


class CommandType(IntEnum):
    WRITE = 0
    READ = 1


class StatusRequest(IntEnum):
    CHECK_TORQUE_ON = 1
    CHECK_OPERATING_MODE = 2
    UPDATE_TORQUE_STATUS = 3


@dataclass
class StagedCommand:
    command_type: CommandType
    id: int  # noqa: A003
    address: int
    length: int
    data: List[int]
    status_request: Optional[List[StatusRequest]] = None
    motor: Optional['Motor'] = None  # noqa: F821
    allowable_operating_modes: Optional[List[OperatingMode]] = None


def toSignedInt(value: int, size: int) -> int:
    bits = size * 8
    if value >= (1 << (bits - 1)):
        value -= (1 << bits)
    return value

