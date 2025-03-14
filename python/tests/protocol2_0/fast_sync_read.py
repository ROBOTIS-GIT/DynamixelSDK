#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2021 ROBOTIS CO., LTD.
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

# Author: Will Son, Wonho Yun

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

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330, X430, X540, 2X430

if MY_DXL == 'X_SERIES':
    # Control table address (May differ by model)
    # Please refer to each product eManual
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132
    # Data Length (Byte, May differ by model))
    # Please refer to each product eManual
    LEN_GOAL_POSITION = 4
    LEN_PRESENT_POSITION = 4
    # Refer to the eManual for Min / Max Position Limit
    DXL_MINIMUM_POSITION_VALUE = 0
    DXL_MAXIMUM_POSITION_VALUE = 4095
    # Factory default Baudrate
    BAUDRATE = 1000000

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# Fast Sync is only supported in Protocol 2.0
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
DXL1_ID = 1
DXL2_ID = 2

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
SERIAL_PORT = '/dev/ttyUSB0'

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
MOVING_THRESHOLD = 20  # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(SERIAL_PORT)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(
    portHandler,
    packetHandler,
    ADDR_GOAL_POSITION,
    LEN_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead = GroupSyncRead(
    portHandler,
    packetHandler,
    ADDR_PRESENT_POSITION,
    LEN_PRESENT_POSITION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to set the baudrate")
else:
    print("Failed to set the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable DYNAMIXEL#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
    portHandler,
    DXL1_ID,
    ADDR_TORQUE_ENABLE,
    TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("DYNAMIXEL#%d has been successfully connected" % DXL1_ID)

# Enable DYNAMIXEL#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
    portHandler,
    DXL2_ID,
    ADDR_TORQUE_ENABLE,
    TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("DYNAMIXEL#%d has been successfully connected" % DXL2_ID)

# Add parameter storage for DYNAMIXEL#1 present position value
dxl_addparam_result = groupSyncRead.addParam(DXL1_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL1_ID)
    quit()

# Add parameter storage for DYNAMIXEL#2 present position value
dxl_addparam_result = groupSyncRead.addParam(DXL2_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL2_ID)
    quit()

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    # Allocate goal position value into byte array
    param_goal_position = [
        DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index])),
        DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index])),
        DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index])),
        DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]))]

    # Add DYNAMIXEL#1 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
        quit()

    # Add DYNAMIXEL#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
        quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    while 1:
        # Fast Sync Read present position
        dxl_comm_result = groupSyncRead.fastSyncRead()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of DYNAMIXEL#1 is available
        dxl_getdata_result = groupSyncRead.isAvailable(
            DXL1_ID,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL1_ID)
            quit()

        # Check if groupsyncread data of DYNAMIXEL#2 is available
        dxl_getdata_result = groupSyncRead.isAvailable(
            DXL2_ID,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL2_ID)
            quit()

        # Get DYNAMIXEL#1 present position value
        dxl1_present_position = groupSyncRead.getData(
            DXL1_ID,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION)

        # Get DYNAMIXEL#2 present position value
        dxl2_present_position = groupSyncRead.getData(
            DXL2_ID,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION)

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\
            \t[ID:%03d] GoalPos:%03d  PresPos:%03d" %
            (DXL1_ID,
            dxl_goal_position[index],
            dxl1_present_position,
            DXL2_ID,
            dxl_goal_position[index],
            dxl2_present_position))

        if not (
            (abs(dxl_goal_position[index] - dxl1_present_position) > MOVING_THRESHOLD) and
            (abs(dxl_goal_position[index] - dxl2_present_position) > MOVING_THRESHOLD)):
            break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

# Clear syncread parameter storage
groupSyncRead.clearParam()

# Disable DYNAMIXEL#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
    portHandler,
    DXL1_ID,
    ADDR_TORQUE_ENABLE,
    TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable DYNAMIXEL#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
    portHandler,
    DXL2_ID,
    ADDR_TORQUE_ENABLE,
    TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
