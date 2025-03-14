#!/usr/bin/env python
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

# Author: Wonho Yun

import os

if os.name == 'nt':
    import msvcrt

    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *

#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'
if MY_DXL == 'X_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_LED_RED                = 65
    LEN_LED_RED                 = 1         # Data Byte Length
    ADDR_GOAL_POSITION          = 116
    LEN_GOAL_POSITION           = 4         # Data Byte Length
    ADDR_PRESENT_POSITION       = 132
    LEN_PRESENT_POSITION        = 4         # Data Byte Length
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 1000000

PROTOCOL_VERSION = 2.0
DXL1_ID = 1
DXL2_ID = 2
DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MOVING_STATUS_THRESHOLD = 20

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]
dxl_led_value = [0x00, 0x01]

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Bulk Write, Fast Bulk Read
groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    getch()
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    getch()
    quit()

for dxl_id in [DXL1_ID, DXL2_ID]:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

# parameter setting (Fast Bulk Read)
groupBulkRead.addParam(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
groupBulkRead.addParam(DXL2_ID, ADDR_LED_RED, LEN_LED_RED)

while True:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index])),
                           DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index])),
                           DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index])),
                           DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]))]

    groupBulkWrite.addParam(DXL1_ID, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, param_goal_position)
    groupBulkWrite.addParam(DXL2_ID, ADDR_LED_RED, LEN_LED_RED, [dxl_led_value[index]])

    groupBulkWrite.txPacket()
    groupBulkWrite.clearParam()

    while True:
        # Fast Bulk Read
        dxl_comm_result = groupBulkRead.fastBulkRead()
        if dxl_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(dxl_comm_result))

        if not groupBulkRead.isAvailable(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
            print("[ID:%03d] Fast Bulk Read data unavailable" % DXL1_ID)
            quit()

        if not groupBulkRead.isAvailable(DXL2_ID, ADDR_LED_RED, LEN_LED_RED):
            print("[ID:%03d] Fast Bulk Read data unavailable" % DXL2_ID)
            quit()

        dxl1_present_position = groupBulkRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        dxl2_led_value_read = groupBulkRead.getData(DXL2_ID, ADDR_LED_RED, LEN_LED_RED)

        print("[ID:%03d] Present Position: %d\t[ID:%03d] LED Value: %d" % (
            DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_led_value_read))

        if not (abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD):
            break

    index = 1 - index

groupBulkRead.clearParam()

for dxl_id in [DXL1_ID, DXL2_ID]:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

portHandler.closePort()
