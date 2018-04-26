#!/usr/bin/env python
#coding: UTF-8

import dynamixel_functions as df
import ctypes
from ctypes import c_char_p, c_uint8, create_string_buffer

COMM_SUCCESS       = 0
COMM_PORT_BUSY     = -1000
COMM_TX_FAIL       = -1001
COMM_RX_FAIL       = -1002
COMM_TX_ERROR      = -2000
COMM_RX_WAITING    = -3000
COMM_RX_TIMEOUT    = -3001
COMM_RX_CORRUPT    = -3002
COMM_NOT_AVAILABLE = -9000

class PortHandler:
  def __init__(self, port_name):
    self._handler = df.portHandler(port_name)
  
  def openPort(self):
    return int(df.openPort(self._handler))
   
  def closePort(self):
    df.closePort(self._handler)
  
  def clearPort(self):
    df.clearPort(self._handler)
    
  def setPortName(self, port_name):
    df.setPortName(self._handler, port_name)
    
  def getPortName(self):
    return c_char_p((df.getPortName(self._handler))).value
    
  def setBaudRate(self, baud_rate):
    res = df.setBaudRate(self._handler, baud_rate)
    return bool(int(res))
    
  def getBaudRate(self):
    return int(df.getBaudRate(self._handler))
    
  def readPort(self, length):
    if length < 1:
      raise Exception("[readPort] length must be greater than 0")
    packet = (c_uint8 * length)()
    ret = int(df.readPort(self._handler, packet, length))
    return int(ret), packet[:]
     
  def writePort(self, length):
    if length < 1:
      raise Exception("[writePort] length must be greater than 0")
    packet = (c_uint8 * length)(*tx_packet)
    ret = int(df.writePort(self._handler, packet, length))
    return int(ret), packet[:]

  def setPacketTimeout(self, packet_length):
    if packet_length < 1:
      raise Exception("[setPacketTimeout] packet_length must be greater than 0")
    df.setPacketTimeout(self._handler, packet_length)

  def setPacketTimeoutMSec(self, msec):
    if msec < 0:
      raise Exception("[setPacketTimeoutMSec] msec must be greater than 0")
    df.setPacketTimeoutMSec(self._handler, msec)
  
  def isPacketTimeout(self):
    res = df.isPacketTimeout(self._handler)
    return bool(int(res))  

class PacketHandler:
  __initiated = False
  
  def __init__(self, protocol_version = 1):
    if protocol_version not in [1,2]:
      raise Exception("[PacketHandler] Only protocols 1 and 2 are supported")
    self._v = protocol_version
    if not PacketHandler.__initiated:
      df.packetHandler()
      PacketHandler.__initiated = True
  
  def getTxRxResult(result):
    return c_char_p(df.getTxRxResult(self._v, result)).value

  def getRxPacketError(error):
    return c_char_p(df.getRxPacketError(self._v, error)).value

  def getLastTxRxResult(self, port):
    return int(df.getLastTxRxResult(port._handler, self._v))
  
  def getLastRxPacketError(self, port):
    return int(df.getLastRxPacketError(port._handler, self._v))
  
  #def setDataWrite(self, port, data_length, data_pos, data):
  #  df.setDataWrite(port._handler, self._v, data_length, data_pos, data)
    
  #def getDataRead(self, port, data_length, data_pos):
  #  return int(df.getDataRead(port._handler, self._v, data_length, data_pos))

  def txPacket(self, port, tx_packet):
    tx_buffer = (c_uint8 * len(tx_packet))(*tx_packet)
    df._txPacket(port._handler, self._v, tx_buffer)
    return self.getLastTxRxResult(port)
    
  def rxPacket(self, port, rx_len):
    if rx_len < 1:
      raise Exception("[rxPacket] rx_len must be greater than 0")    
    rx_buffer = (c_uint8 * rx_len)()
    df._rxPacket(port._handler, self._v, rx_buffer)
    return self.getLastTxRxResult(port), rx_buffer[:]

  def txRxPacket(self, port, tx_packet, rx_len):
    if rx_len < 1:
      raise Exception("[rxPacket] rx_len must be greater than 0")
    tx_buffer = (c_uint8 * len(tx_packet))(*tx_packet)
    rx_buffer = (c_uint8 * rx_len)()
    df._txRxPacket(port._handler, self._v, tx_buffer, rx_buffer)
    return self.getLastTxRxResult(port), rx_buffer[:], self.getLastRxPacketError(port)

  def ping(self, port, id, model_num=False):
    if model_num is True:
      res = df.pingGetModelNum(port._handler, self._v, id)
      return self.getLastTxRxResult(port), int(res), self.getLastRxPacketError(port)
    else:
      df.ping(port._handler, self._v, id)
      return self.getLastTxRxResult(port), getLastRxPacketError(port)

  def broadcastPing(self, port):
    df.broadcastPing(port._handler, self._v)
    l = []
    for i in range(0,253):
      if df.getBroadcastPingResult(port._handler, self._v, i) is True:
        l.append(i)
    return self.getLastTxRxResult(port), l

  def action(self, port, id):
    df.action(port._handler, self._v, id)
    return self.getLastTxRxResult(port)

  def reboot(self, port, id): 
    df.reboot(port._handler, self._v, id)
    return self.getLastTxRxResult(port), self.getLastRxPacketError(port)

  def factoryReset(self, port, id): 
    df.factoryReset(port._handler, self._v, id)
    return self.getLastTxRxResult(port), self.getLastRxPacketError(port)

  def readTx(self, port, id, address, length): 
    if address < 1:
      raise Exception("[readTx] address must be greater than 0")
    if length < 1:
      raise Exception("[readTx] length must be greater than 0")
    df.readTx(port._handler, self._v, id, address, length)
    return self.getLastTxRxResult(port)
  
  def readRx(self, port, length): #TODO: return data as byte array
    if length < 1:
      raise Exception("[readRx] length must be greater than 0")
    df.readRx(port._handler, self._v, length)
    data = df.getDataRead(port._handler, self._v, length, 0)
    return self.getLastTxRxResult(port), int(data), self.getLastRxPacketError(port)

  def readTxRx(self, port, id, address, length): #TODO: return data as byte array
    if address < 1:
      raise Exception("[readTxRx] address must be greater than 0")
    if length < 1:
      raise Exception("[readTxRx] length must be greater than 0")
    df.readTxRx(port._handler, self._v, id, address, length)
    data = df.getDataRead(port._handler, self._v, length, 0)
    return self.getLastTxRxResult(port), int(data), self.getLastRxPacketError(port)

  def read1ByteTx(self, port, id, address):
    if address < 1:
      raise Exception("[read1ByteTx] address must be greater than 0")
    df.read1ByteTx(port._handler, self._v, id, address)
    return self.getLastTxRxResult(port)
  
  def read1ByteRx(self, port): 
    data = df.read1ByteRx(port._handler)
    return self.getLastTxRxResult(port), int(data), self.getLastRxPacketError(port)

  def read1ByteTxRx(self, port, id, address): 
    if address < 1:
      raise Exception("[read1ByteTxRx] address must be greater than 0")
    data = df.read1ByteTxRx(port._handler, self._v, id, address)
    return self.getLastTxRxResult(port), int(data), self.getLastRxPacketError(port)
    
  def read2ByteTx(self, port, id, address):
    if address < 1:
      raise Exception("[read2ByteTx] address must be greater than 0")
    df.read2ByteTx(port._handler, self._v, id, address)
    return self.getLastTxRxResult(port)
  
  def read2ByteRx(self, port): 
    data = df.read2ByteRx(port._handler)
    return self.getLastTxRxResult(port), int(data), self.getLastRxPacketError(port)

  def read2ByteTxRx(self, port, id, address): 
    if address < 1:
      raise Exception("[read2ByteTxRx] address must be greater than 0")
    data = df.read2ByteTxRx(port._handler, self._v, id, address)
    return self.getLastTxRxResult(port), int(data), self.getLastRxPacketError(port)
    
  def read4ByteTx(self, port, id, address):
    if address < 1:
      raise Exception("[read4ByteTx] address must be greater than 0")
    df.read4ByteTx(port._handler, self._v, id, address)
    return self.getLastTxRxResult(port)
  
  def read4ByteRx(self, port): 
    data = df.read1ByteRx(port._handler)
    return self.getLastTxRxResult(port), int(data), self.getLastRxPacketError(port)

  def read4ByteTxRx(self, port, id, address): 
    if address < 1:
      raise Exception("[read4ByteTxRx] address must be greater than 0")
    data = df.read4ByteTxRx(port._handler, self._v, id, address)
    return self.getLastTxRxResult(port), int(data), self.getLastRxPacketError(port)

  def writeTxOnly(self, port, id, address, length, data):
    if address < 1:
      raise Exception("[writeTxOnly] address must be greater than 0")
    if length < 1:
      raise Exception("[writeTxOnly] length must be greater than 0")
    df.setDataWrite(port._handler, self._v, length, 0, data)
    df.writeTxOnly(port._handler, self._v, id, address, length)
    return self.getLastTxRxResult(port)
  
  def writeTxRx(self, port, id, address, length, data):
    if address < 1:
      raise Exception("[writeTxRx] address must be greater than 0")
    if length < 1:
      raise Exception("[writeTxRx] length must be greater than 0")
    df.setDataWrite(port._handler, self._v, length, 0, data)
    df.writeTxRx(port._handler, self._v, id, address, length)
    return self.getLastTxRxResult(port), self.getLastRxPacketError(port)

  def write1ByteTxOnly(self, port, id, address, data):
    if address < 1:
      raise Exception("[write1ByteTxOnly] address must be greater than 0")
    df.write1ByteTxOnly(port._handler, self._v, id, address, data)
    return self.getLastTxRxResult(port)
    
  def write1ByteTxRx(self, port, id, address, data):
    if address < 1:
      raise Exception("[write1ByteTxRx] address must be greater than 0")
    df.write1ByteTxOnly(port._handler, self._v, id, address, data)
    return self.getLastTxRxResult(port), self.getLastRxPacketError(port)

  def write2ByteTxOnly(self, port, id, address, data):
    if address < 1:
      raise Exception("[write2ByteTxOnly] address must be greater than 0")
    df.write2ByteTxOnly(port._handler, self._v, id, address, data)
    return self.getLastTxRxResult(port)
  
  def write2ByteTxRx(self, port, id, address, data): 
    if address < 1:
      raise Exception("[write2ByteTxRx] address must be greater than 0")
    df.write2ByteTxOnly(port._handler, self._v, id, address, data)
    return self.getLastTxRxResult(port), self.getLastRxPacketError(port)
  
  def write4ByteTxOnly(self, port, id, address, data):
    if address < 1:
      raise Exception("[write4ByteTxOnly] address must be greater than 0")
    df.write4ByteTxOnly(port._handler, self._v, id, address, data)
    return self.getLastTxRxResult(port)
    
  def write4ByteTxRx(self, port, id, address, data):
    if address < 1:
      raise Exception("[write4ByteTxRx] address must be greater than 0")
    df.write4ByteTxOnly(port._handler, self._v, id, address, data)
    return self.getLastTxRxResult(port), self.getLastRxPacketError(port)

  def regWriteTxOnly(self, port, id, address, length, data):
    if address < 1:
      raise Exception("[regWriteTxOnly] address must be greater than 0")
    if length < 1:
      raise Exception("[regWriteTxOnly] length must be greater than 0")
    df.setDataWrite(port._handler, self._v, length, 0, data)
    df.regWriteTxOnly(port._handler, self._v, id, address, length)
    return self.getLastTxRxResult(port)

  def regWriteTxRx(self, port, id, address, length): 
    if address < 1:
      raise Exception("[regWriteTxRx] address must be greater than 0")
    if length < 1:
      raise Exception("[regWriteTxRx] length must be greater than 0")
    df.setDataWrite(port._handler, self._v, length, 0, data)
    df.regWriteTxRx(port._handler, self._v, id, address, length)
    return self.getLastTxRxResult(port), self.getLastRxPacketError(port)

  def syncReadTx(self, port, start_address, data_length, param, param_length):
    if start_address < 1:
      raise Exception("[syncReadTx] start_address must be greater than 0")
    if data_length < 1:
      raise Exception("[syncReadTx] data_length must be greater than 0")
    if param_length < 1:
      raise Exception("[syncReadTx] param_length must be greater than 0")
    for i in range (0, param_length):
      df.setDataWrite(port._handler, self._v, 1, i, param[i])
    df.syncReadTx(port._handler, self._v, start_address, data_length, param_length)
    return self.getLastTxRxResult(port)
  
  # syncReadRx   -> GroupSyncRead
  # syncReadTxRx -> GroupSyncRead

  def syncWriteTxOnly(self, port, start_address, data_length, param, param_length):
    if start_address < 1:
      raise Exception("[syncWriteTxOnly] start_address must be greater than 0")
    if data_length < 1:
      raise Exception("[syncWriteTxOnly] data_length must be greater than 0")
    if param_length < 1:
      raise Exception("[syncWriteTxOnly] param_length must be greater than 0")
    for i in range (0, param_length):
      df.setDataWrite(port._handler, self._v, 1, i, param[i])
    df.syncWriteTxOnly(port._handler, self._v, start_address, data_length, param_length)
    return self.getLastTxRxResult(port)

  def bulkReadTx(self, port, param, param_length):
    if param_length < 1:
      raise Exception("[bulkReadTx] param_length must be greater than 0")
    for i in range (0, param_length):
      df.setDataWrite(port._handler, self._v, 1, i, param[i])
    df.bulkReadTx(port._handler, self._v, param_length)
    return self.getLastTxRxResult(port)

  # bulkReadRx   -> GroupBulkRead
  # bulkReadTxRx -> GroupBulkRead

  def bulkWriteTxOnly(self, port, param, param_length):
    if param_length < 1:
      raise Exception("[bulkReadTx] param_length must be greater than 0")
    for i in range (0, param_length):
      df.setDataWrite(port._handler, self._v, 1, i, param[i])
    df.bulkWriteTxOnly(port._handler, self._v, param_length)
    return self.getLastTxRxResult(port)


class GroupBulkRead:  
  def __init__(self, port, packet_handler):
    self._b = int(df.groupBulkRead(port._handler, packet_handler._v))
    self._packet_h = packet_handler
    self._port_h = port

  def groupBulkReadAddParam(self, id, start_address, data_length):
    res = df.groupBulkReadAddParam(self._b, id, start_address, data_length)
    return bool(int(res))
  
  def groupBulkReadRemoveParam(self, id):
    df.groupBulkReadRemoveParam(self._b, id)

  def groupBulkReadClearParam(self):
    df.groupBulkReadClearParam(self._b)
    
  def groupBulkReadTxPacket(self):
    df.groupBulkReadTxPacket(self._b)
    return self._packet_h.getLastTxRxResult(self._port_h)

  def groupBulkReadRxPacket(self):
    df.groupBulkReadRxPacket(self._b)
    return self._packet_h.getLastTxRxResult(self._port_h)
    
  def groupBulkReadTxRxPacket(self):
    df.groupBulkReadTxRxPacket(self._b)
    return self._packet_h.getLastTxRxResult(self._port_h)
    
  def groupBulkReadIsAvailable(self, id, address, data_length):
    res = df.groupBulkReadIsAvailable(self._b, id, address, data_length)
    return bool(int(res))
  
  def groupBulkReadGetData(self, id, address, data_length):
    res = df.groupBulkReadGetData(self._b, id, address, data_length)
    return int(res)
    
    
class GroupBulkWrite:
  def __init__(self, port, packet_handler):
    self._b = int(df.groupBulkWrite(port._handler, packet_handler._v))
    self._packet_h = packet_handler
    self._port_h = port

  def groupBulkWriteAddParam(self, id, start_address, data_length, data): #TODO accept data like uint8_t* instead of uint32_t
    if len(data) > 4:
      raise Exception("[groupBulkWriteAddParam] No more than 4 data bytes are allowed")
    res = df.groupBulkWriteAddParam(self._b, id, start_address, data_length, data, data_length)
    return bool(int(res))
    
  def groupBulkWriteChangeParam(self, id, start_address, data_length, data): #TODO accept data like uint8_t* instead of uint32_t
    res = df.groupBulkWriteChangeParam(self._b, id, start_address, data_length, data, data_length, 0)
    return bool(int(res))
  
  def groupBulkWriteRemoveParam(self, id):
    df.groupBulkWriteRemoveParam(self._b, id)

  def groupBulkWriteClearParam(self):
    df.groupBulkWriteClearParam(self._b)
    
  def groupBulkWriteTxPacket(self):
    df.groupBulkWriteTxPacket(self._b)
    return self._packet_h.getLastTxRxResult(self._port_h)


class GroupSyncRead:
  def __init__(self, port, packet_handler, start_address, data_length):
    self._g = int(df.groupSyncRead(port._handler, packet_handler._v, start_address, data_length))
    self._packet_h = packet_handler
    self._port_h = port

  def groupSyncReadAddParam(id):
    res = df.groupSyncReadAddParam(self._g, id)
    return bool(int(res))
    
  def groupSyncReadRemoveParam(id):
    df.groupSyncReadRemoveParam(self._g, id)
    
  def groupSyncReadCleareParam():
    df.groupSyncReadClearParam(self._g)
    
  def groupSyncReadTxPacket():
    df.groupSyncReadTxPacket(self._g)
    return self._packet_h.getLastTxRxResult(self._port_h)
    
  def groupSyncReadRxPacket():
    df.groupSyncReadRxPacket(self._g)
    return self._packet_h.getLastTxRxResult(self._port_h)
    
  def groupSyncReadTxRxPacket():
    df.groupSyncReadTxRxPacket(self._g)
    return self._packet_h.getLastTxRxResult(self._port_h)

  def groupSyncReadIsAvailable(id, address, data_length):
    res = df.groupSyncReadIsAvailable(self._g, id, address, data_length)
    return bool(int(res))

  def groupSyncReadGetData(id, address, data_length):
    return int(df.groupSyncReadGetData(self._g, id, address, data_length))


class GroupSyncWrite:
  def __init__(self, port, packet_handler, start_address, data_length):
    self._g = int(df.groupSyncWrite(port._handler, packet_handler._v, start_address, data_length))
    self._packet_h = packet_handler
    self._port_h = port
    self._data_length = data_length

  def groupSyncWriteAddParam(self, id, data):
    res = df.groupSyncWriteAddParam(self._g, id, data, self._data_length)
    return bool(int(res))
    
  def groupSyncWriteChangeParam(self, id, data, data_pos):
    res = df.groupSyncWriteChangeParam(self._g, id, data, self._data_length, 0)
    return bool(int(res))
  
  def groupSyncWriteRemoveParam(self, id):
    df.groupSyncWriteRemoveParam(self._g, id)

  def groupSyncWriteClearParam(self):
    df.groupSyncWriteClearParam(self._g)
    
  def groupSyncWriteTxPacket(self):
    df.groupSyncWriteTxPacket(self._g)
    return self._packet_h.getLastTxRxResult(self._port_h)

