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

# Author: Ryu Woon Jung (Leon)

import ctypes
from ctypes import cdll
from ctypes import c_int, c_char_p, c_uint8, c_uint16, c_uint32, c_double, POINTER
#dxl_lib = cdll.LoadLibrary("../../c/build/win32/output/dxl_x86_c.dll")  # for windows 32bit
# dxl_lib = cdll.LoadLibrary("../../c/build/win64/output/dxl_x64_c.dll")  # for windows 64bit
# dxl_lib = cdll.LoadLibrary("../../c/build/linux32/libdxl_x86_c.so")     # for linux 32bit
dxl_lib = cdll.LoadLibrary("../../c/build/linux64/libdxl_x64_c.so")     # for linux 64bit
# dxl_lib = cdll.LoadLibrary("../../c/build/linux_sbc/libdxl_sbc_c.so")   # for SBC linux
# dxl_lib = cdll.LoadLibrary("../../c/build/mac/libdxl_mac_c.dylib")      # for Mac OS

###
### Defined in: port_handler.h
###
portHandler = dxl_lib.portHandler
portHandler.argtypes = [c_char_p]
portHandler.restype = c_int

openPort = dxl_lib.openPort
openPort.argtypes = [c_int]
openPort.restype = c_uint8

closePort = dxl_lib.closePort
closePort.argtypes = [c_int]
closePort.restype = None

clearPort = dxl_lib.clearPort
closePort.argtypes = [c_int]
closePort.restype = None

setPortName = dxl_lib.setPortName
setPortName.argtypes = [c_int, c_char_p]
setPortName.restype = None

getPortName = dxl_lib.getPortName
getPortName.argtypes = [c_int]
getPortName.restype = c_char_p

setBaudRate = dxl_lib.setBaudRate
setBaudRate.argtypes = [c_int, c_int]
setBaudRate.restype = c_uint8

getBaudRate = dxl_lib.getBaudRate
getBaudRate.argtypes = [c_int]
getBaudRate.restype = c_int

readPort = dxl_lib.readPort
readPort.argtypes = [c_int, POINTER(c_uint8), c_int]
readPort.restype = c_int

writePort = dxl_lib.writePort
readPort.argtypes = [c_int, POINTER(c_uint8), c_int]
readPort.restype = c_int

setPacketTimeout = dxl_lib.setPacketTimeout
setPacketTimeout.argtypes = [c_int, c_uint16]
setPacketTimeout.restype = None

setPacketTimeoutMSec = dxl_lib.setPacketTimeoutMSec
setPacketTimeout.argtypes = [c_int, c_double]
setPacketTimeout.restype = None

isPacketTimeout = dxl_lib.isPacketTimeout
isPacketTimeout.argtypes = [c_int]
isPacketTimeout.restype = c_uint8

###
### Defined in: packet_handler.h
###

#class struct_PacketData(ctypes.Structure):
#  _fields_ = [("data_write", POINTER(c_uint8)),
#              ("data_read", POINTER(c_uint8)),
#              ("tx_packet", POINTER(c_uint8)),
#              ("rx_packet", POINTER(c_uint8)),
#              ("error", c_uint8),
#              ("communication_result", int),
#              ("broadcast_ping_id_list", POINTER(c_uint8))]

#PacketData = POINTER(struct_PacketData)
#packetData = PacketData.in_dll(dxl_lib, "packetData")

packetHandler = dxl_lib.packetHandler
packetHandler.argtypes = []
packetHandler.restype = None

getTxRxResult = dxl_lib.getTxRxResult
getTxRxResult.argtypes = [c_int, c_int]
getTxRxResult.restype = c_char_p

printTxRxResult = dxl_lib.printTxRxResult ## DEPRECATED
printTxRxResult.argtypes = [c_int, c_int]
printTxRxResult.restype = None

getRxPacketError = dxl_lib.getRxPacketError
getRxPacketError.argtypes = [c_int, c_uint8]
getRxPacketError.restype = c_char_p

printRxPacketError = dxl_lib.printRxPacketError ##DEPRECATED
printRxPacketError.argtypes = [c_int, c_uint8]
printRxPacketError.restype = None

getLastTxRxResult = dxl_lib.getLastTxRxResult
getLastTxRxResult.argtypes = [c_int, c_int]
getLastTxRxResult.restype = c_int

getLastRxPacketError = dxl_lib.getLastRxPacketError
getLastRxPacketError.argtypes = [c_int, c_int]
getLastRxPacketError.restype = c_uint8

setDataWrite = dxl_lib.setDataWrite
setDataWrite.argtypes = [c_int, c_int, c_uint16, c_uint16, c_uint32]
setDataWrite.restype = None

getDataRead = dxl_lib.getDataRead
getDataRead.argtypes = [c_int, c_int, c_uint16, c_uint16]
getDataRead.restype = c_uint32

_txPacket = dxl_lib._txPacket
_txPacket.argtypes = [c_int, c_int, POINTER(c_uint8)]
_txPacket.restype = None

txPacket = dxl_lib.txPacket
txPacket.argtypes = [c_int, c_int]
txPacket.restype = None

_rxPacket = dxl_lib._rxPacket
_rxPacket.argtypes = [c_int, c_int, POINTER(c_uint8)]
_rxPacket.restype = None

rxPacket = dxl_lib.rxPacket
rxPacket.argtypes = [c_int, c_int]
rxPacket.restype = None

_txRxPacket = dxl_lib._txRxPacket
_txRxPacket.argtypes = [c_int, c_int, POINTER(c_uint8), POINTER(c_uint8)]
_txRxPacket.restype = None

txRxPacket = dxl_lib.txRxPacket
txRxPacket.argtypes = [c_int, c_int]
txRxPacket.restype = None

ping = dxl_lib.ping
ping.argtypes = [c_int, c_int, c_uint8]
ping.restype = None

pingGetModelNum = dxl_lib.pingGetModelNum
pingGetModelNum.argtypes = [c_int, c_int, c_uint8]
pingGetModelNum.restype = c_uint16

broadcastPing = dxl_lib.broadcastPing
broadcastPing.argtypes = [c_int, c_int]
broadcastPing.restype = None

getBroadcastPingResult = dxl_lib.getBroadcastPingResult
getBroadcastPingResult.argtypes = [c_int, c_int, c_int]
getBroadcastPingResult.restype = c_uint8

action = dxl_lib.action
action.argtypes = [c_int, c_int, c_uint8]
action.restype = None

reboot = dxl_lib.reboot
reboot.argtypes = [c_int, c_int, c_uint8]
reboot.restype = None

factoryReset = dxl_lib.factoryReset
factoryReset.argtypes = [c_int, c_int, c_uint8, c_uint8]
factoryReset.restype = None

readTx = dxl_lib.readTx
readTx.argtypes = [c_int, c_int, c_uint8, c_uint16, c_uint16]
readTx.restype = None

readRx = dxl_lib.readRx
readRx.argtypes = [c_int, c_int, c_uint16]
readRx.restype = None

readTxRx = dxl_lib.readTxRx
readTxRx.argtypes = [c_int, c_int, c_uint8, c_uint16, c_uint16]
readTxRx.restype = None

read1ByteTx = dxl_lib.read1ByteTx
read1ByteTx.argtypes = [c_int, c_int, c_uint8, c_uint16]
read1ByteTx.restype = None

read1ByteRx = dxl_lib.read1ByteRx
read1ByteRx.argtypes = [c_int, c_int]
read1ByteRx.restype = c_uint8

read1ByteTxRx = dxl_lib.read1ByteTxRx
read1ByteTxRx.argtypes = [c_int, c_int, c_uint8, c_uint16]
read1ByteTxRx.restype = c_uint8

read2ByteTx = dxl_lib.read2ByteTx
read2ByteTx.argtypes = [c_int, c_int, c_uint8, c_uint16]
read2ByteTx.restype = None

read2ByteRx = dxl_lib.read2ByteRx
read2ByteRx.argtypes = [c_int, c_int]
read2ByteRx.restype = c_uint16

read2ByteTxRx = dxl_lib.read2ByteTxRx
read2ByteTxRx.argtypes = [c_int, c_int, c_uint8, c_uint16]
read2ByteTxRx.restype = c_uint16

read4ByteTx = dxl_lib.read4ByteTx
read4ByteTx.argtypes = [c_int, c_int, c_uint8, c_uint16]
read4ByteTx.restype = None

read4ByteRx = dxl_lib.read4ByteRx
read4ByteRx.argtypes = [c_int, c_int]
read4ByteRx.restype = c_uint32

read4ByteTxRx = dxl_lib.read4ByteTxRx
read4ByteTxRx.argtypes = [c_int, c_int, c_uint8, c_uint16]
read4ByteTxRx.restype = c_uint32

writeTxOnly = dxl_lib.writeTxOnly
writeTxOnly.argtypes = [c_int, c_int, c_uint8, c_uint16, c_uint16]
writeTxOnly.restype = None

writeTxRx = dxl_lib.writeTxRx
writeTxRx.argtypes = [c_int, c_int, c_uint8, c_uint16, c_uint16]
writeTxRx.restype = None

write1ByteTxOnly = dxl_lib.write1ByteTxOnly
write1ByteTxOnly.argtypes = [c_int, c_int, c_uint8, c_uint16, c_uint8]
write1ByteTxOnly.restype = None

write1ByteTxRx = dxl_lib.write1ByteTxRx
write1ByteTxRx.argtypes = [c_int, c_int, c_uint8, c_uint16, c_uint8]
write1ByteTxRx.restype = None

write2ByteTxOnly = dxl_lib.write2ByteTxOnly
write2ByteTxOnly.argtypes = [c_int, c_int, c_uint8, c_uint16, c_uint8]
write2ByteTxOnly.restype = None

write2ByteTxRx = dxl_lib.write2ByteTxRx
write2ByteTxRx.argtypes = [c_int, c_int, c_uint8, c_uint16, c_uint8]
write2ByteTxRx.restype = None

write4ByteTxOnly = dxl_lib.write4ByteTxOnly
write4ByteTxOnly.argtypes = [c_int, c_int, c_uint8, c_uint16, c_uint8]
write4ByteTxOnly.restype = None

write4ByteTxRx = dxl_lib.write4ByteTxRx
write4ByteTxRx.argtypes = [c_int, c_int, c_uint8, c_uint16, c_uint8]
write4ByteTxRx.restype = None

regWriteTxOnly = dxl_lib.regWriteTxOnly
regWriteTxOnly.argtypes = [c_int, c_int, c_uint8, c_uint16, c_uint8]
regWriteTxOnly.restype = None

regWriteTxRx = dxl_lib.regWriteTxRx
regWriteTxRx.argtypes = [c_int, c_int, c_uint8, c_uint16, c_uint8]
regWriteTxRx.restype = None

syncReadTx = dxl_lib.syncReadTx
syncReadTx.argtypes = [c_int, c_int, c_uint16, c_uint16, c_uint16]
syncReadTx.restype = None

# syncReadRx   -> GroupSyncRead
# syncReadTxRx -> GroupSyncRead

syncWriteTxOnly = dxl_lib.syncWriteTxOnly
syncWriteTxOnly.argtypes = [c_int, c_int, c_uint16, c_uint16, c_uint16]
syncWriteTxOnly.restype = None

bulkReadTx = dxl_lib.bulkReadTx
bulkReadTx.argtypes = [c_int, c_int, c_uint16]
bulkReadTx.restype = None

# bulkReadRx   -> GroupBulkRead
# bulkReadTxRx -> GroupBulkRead

bulkWriteTxOnly = dxl_lib.bulkWriteTxOnly
bulkWriteTxOnly.argtypes = [c_int, c_int, c_uint16]
bulkWriteTxOnly.restype = None

###
### Defined in: group_bulk_read.h
###
groupBulkRead = dxl_lib.groupBulkRead
groupBulkRead.argtypes = [c_int, c_int]
groupBulkRead.restype = c_int

groupBulkReadAddParam = dxl_lib.groupBulkReadAddParam
groupBulkReadAddParam.argtypes = [c_int, c_uint8, c_uint16, c_uint16]
groupBulkReadAddParam.restype = c_uint8

groupBulkReadRemoveParam = dxl_lib.groupBulkReadRemoveParam
groupBulkReadRemoveParam.argtypes = [c_int, c_uint8]
groupBulkReadRemoveParam.restype = None

groupBulkReadClearParam = dxl_lib.groupBulkReadClearParam
groupBulkReadClearParam.argtypes = [c_int]
groupBulkReadClearParam.restype = None

groupBulkReadTxPacket = dxl_lib.groupBulkReadTxPacket
groupBulkReadTxPacket.argtypes = [c_int]
groupBulkReadTxPacket.restype = None

groupBulkReadRxPacket = dxl_lib.groupBulkReadRxPacket
groupBulkReadRxPacket.argtypes = [c_int]
groupBulkReadRxPacket.restype = None

groupBulkReadTxRxPacket = dxl_lib.groupBulkReadTxRxPacket
groupBulkReadTxRxPacket.argtypes = [c_int]
groupBulkReadTxRxPacket.restype = None

groupBulkReadIsAvailable = dxl_lib.groupBulkReadIsAvailable
groupBulkReadIsAvailable.argtypes = [c_int, c_uint8, c_uint16, c_uint16]
groupBulkReadIsAvailable.restype = c_uint8

groupBulkReadGetData = dxl_lib.groupBulkReadGetData
groupBulkReadGetData.argtypes = [c_int, c_uint8, c_uint16, c_uint16]
groupBulkReadGetData.restype = c_uint32

###
### Defined in: group_bulk_write.h
###
groupBulkWrite = dxl_lib.groupBulkWrite
groupBulkWrite.argtypes = [c_int, c_int]
groupBulkWrite.restype = c_int

groupBulkWriteAddParam = dxl_lib.groupBulkWriteAddParam
groupBulkWriteAddParam.argtypes = [c_int, c_uint8, c_uint16, c_uint16, c_uint32, c_uint16]
groupBulkWriteAddParam.restype = c_uint8

groupBulkWriteRemoveParam = dxl_lib.groupBulkWriteRemoveParam
groupBulkWriteRemoveParam.argtypes = [c_int, c_uint8]
groupBulkWriteRemoveParam.restype = None

groupBulkWriteChangeParam = dxl_lib.groupBulkWriteChangeParam
groupBulkWriteChangeParam.argtypes = [c_int, c_uint8, c_uint16, c_uint16, c_uint32, c_uint16, c_uint16]
groupBulkWriteChangeParam.restype = c_uint8

groupBulkWriteClearParam = dxl_lib.groupBulkWriteClearParam
groupBulkWriteClearParam.argtypes = [c_int]
groupBulkWriteClearParam.restype = None

groupBulkWriteTxPacket = dxl_lib.groupBulkWriteTxPacket
groupBulkWriteTxPacket.argtypes = [c_int]
groupBulkWriteTxPacket.restype = None

###
### Defined in: group_sync_read.h
###
groupSyncRead = dxl_lib.groupSyncRead
groupSyncRead.argtypes = [c_int, c_int, c_uint16, c_uint16]
groupSyncRead.restype = c_int

groupSyncReadAddParam = dxl_lib.groupSyncReadAddParam
groupSyncReadAddParam.argtypes = [c_int, c_uint8]
groupSyncReadAddParam.restype = c_uint8

groupSyncReadRemoveParam = dxl_lib.groupSyncReadRemoveParam
groupSyncReadRemoveParam.argtypes = [c_int, c_uint8]
groupSyncReadRemoveParam.restype = None

groupSyncReadClearParam = dxl_lib.groupSyncReadClearParam
groupSyncReadClearParam.argtypes = [c_int]
groupSyncReadClearParam.restype = None

groupSyncReadTxPacket = dxl_lib.groupSyncReadTxPacket
groupSyncReadTxPacket.argtypes = [c_int]
groupSyncReadTxPacket.restype = None

groupSyncReadRxPacket = dxl_lib.groupSyncReadRxPacket
groupSyncReadRxPacket.argtypes = [c_int]
groupSyncReadRxPacket.restype = None

groupSyncReadTxRxPacket = dxl_lib.groupSyncReadTxRxPacket
groupSyncReadTxRxPacket.argtypes = [c_int]
groupSyncReadTxRxPacket.restype = None

groupSyncReadIsAvailable = dxl_lib.groupSyncReadIsAvailable
groupSyncReadIsAvailable.argtypes = [c_int, c_uint8, c_uint16, c_uint16]
groupSyncReadIsAvailable.restype = c_uint8

groupSyncReadGetData = dxl_lib.groupSyncReadGetData
groupSyncReadGetData.argtypes = [c_int, c_uint8, c_uint16, c_uint16]
groupSyncReadGetData.restype = c_uint32

###
### Defined in: group_sync_write.h
###
groupSyncWrite = dxl_lib.groupSyncWrite
groupSyncWrite.argtypes = [c_int, c_int, c_uint16, c_uint16]
groupSyncWrite.restype = c_int

groupSyncWriteAddParam = dxl_lib.groupSyncWriteAddParam
groupSyncWriteAddParam.argtypes = [c_int, c_uint8, c_uint32, c_uint16]
groupSyncWriteAddParam.restype = c_uint8

groupSyncWriteRemoveParam = dxl_lib.groupSyncWriteRemoveParam
groupSyncWriteRemoveParam.argtypes = [c_int, c_uint8]
groupSyncWriteRemoveParam.restype = None

groupSyncWriteChangeParam = dxl_lib.groupSyncWriteChangeParam
groupSyncWriteChangeParam.argtypes = [c_int, c_uint8, c_uint32, c_uint16, c_uint16]
groupSyncWriteChangeParam.restype = c_uint8

groupSyncWriteClearParam = dxl_lib.groupSyncWriteClearParam
groupSyncWriteClearParam.argtypes = [c_int]
groupSyncWriteClearParam.restype = None

groupSyncWriteTxPacket = dxl_lib.groupSyncWriteTxPacket
groupSyncWriteTxPacket.argtypes = [c_int]
groupSyncWriteTxPacket.restype = None

