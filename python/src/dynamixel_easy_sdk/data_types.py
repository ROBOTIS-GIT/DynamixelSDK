from typing import List, Optional
from enum import IntEnum
from dataclasses import dataclass


@dataclass
class ControlTableItem:
    address: int
    size: int

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

class CommandType(IntEnum):
    WRITE = 0
    READ = 1

class StatusRequest(IntEnum):
    CHECK_TORQUE_ON = 1
    CHECK_CURRENT_MODE = 2
    CHECK_VELOCITY_MODE = 3
    CHECK_POSITION_MODE = 4
    CHECK_PWM_MODE = 5
    UPDATE_TORQUE_STATUS = 6

@dataclass
class StagedCommand:
    command_type: CommandType
    id: int
    address: int
    length: int
    data: List[int]
    status_request: Optional[StatusRequest] = None
    motor: Optional["Motor"] = None
