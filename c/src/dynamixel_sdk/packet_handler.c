/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

#include <stdlib.h>
#include "robotis_def.h"

#if defined(__linux__)
#include "packet_handler.h"
#include "protocol1_packet_handler.h"
#include "protocol2_packet_handler.h"
#elif defined(__APPLE__)
#include "packet_handler.h"
#include "protocol1_packet_handler.h"
#include "protocol2_packet_handler.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "packet_handler.h"
#include "protocol1_packet_handler.h"
#include "protocol2_packet_handler.h"
#endif

static PacketData g_packetDataArray[DXL_MAX_PORTS];

static uint8_t g_data_write[DXL_MAX_PORTS][DXL_MAX_BUFFER_LEN];
static uint8_t g_data_read[DXL_MAX_PORTS][DXL_MAX_BUFFER_LEN];
static uint8_t g_tx_packet[DXL_MAX_PORTS][DXL_MAX_BUFFER_LEN];
static uint8_t g_rx_packet[DXL_MAX_PORTS][DXL_MAX_BUFFER_LEN];
static uint8_t g_broadcast_ping_id_list[DXL_MAX_PORTS][255];

PacketData *packetData = g_packetDataArray;

void packetHandler()
{
  static uint8_t is_initialized = False;
  int port_num;

  if (is_initialized == True)
    return;

  for (port_num = 0; port_num < DXL_MAX_PORTS; port_num++)
  {
    g_packetDataArray[port_num].data_write = g_data_write[port_num];
    g_packetDataArray[port_num].data_read = g_data_read[port_num];
    g_packetDataArray[port_num].tx_packet = g_tx_packet[port_num];
    g_packetDataArray[port_num].rx_packet = g_rx_packet[port_num];
    g_packetDataArray[port_num].broadcast_ping_id_list = g_broadcast_ping_id_list[port_num];
    
    g_packetDataArray[port_num].error = 0;
    g_packetDataArray[port_num].communication_result = 0;
  }

  is_initialized = True;
}

const char *getTxRxResult(int protocol_version, int result)
{
  if (protocol_version == 1)
  {
    return getTxRxResult1(result);
  }
  else
  {
    return getTxRxResult2(result);
  }
}


const char *getRxPacketError(int protocol_version, uint8_t error)
{
  if (protocol_version == 1)
  {
    return getRxPacketError1(error);
  }
  else
  {
    return getRxPacketError2(error);
  }
}


int getLastTxRxResult(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    return getLastTxRxResult1(port_num);
  }
  else
  {
    return getLastTxRxResult2(port_num);
  }
}
uint8_t getLastRxPacketError(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    return getLastRxPacketError1(port_num);
  }
  else
  {
    return getLastRxPacketError2(port_num);
  }
}


void setDataWrite(int port_num, int protocol_version, uint16_t data_length, uint16_t data_pos, uint32_t data)
{
  if (protocol_version == 1)
  {
    setDataWrite1(port_num, data_length, data_pos, data);
  }
  else
  {
    setDataWrite2(port_num, data_length, data_pos, data);
  }
}
uint32_t getDataRead(int port_num, int protocol_version, uint16_t data_length, uint16_t data_pos)
{
  if (protocol_version == 1)
  {
    return getDataRead1(port_num, data_length, data_pos);
  }
  else
  {
    return getDataRead2(port_num, data_length, data_pos);
  }
}

void txPacket(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    txPacket1(port_num);
  }
  else
  {
    txPacket2(port_num);
  }
}

void rxPacket(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    rxPacket1(port_num);
  }
  else
  {
    rxPacket2(port_num);
  }
}

void txRxPacket(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    txRxPacket1(port_num);
  }
  else
  {
    txRxPacket2(port_num);
  }
}

void ping(int port_num, int protocol_version, uint8_t id)
{
  if (protocol_version == 1)
  {
    ping1(port_num, id);
  }
  else
  {
    ping2(port_num, id);
  }
}

uint16_t pingGetModelNum(int port_num, int protocol_version, uint8_t id)
{
  if (protocol_version == 1)
  {
    return pingGetModelNum1(port_num, id);
  }
  else
  {
    return pingGetModelNum2(port_num, id);
  }
}

// broadcastPing
void broadcastPing(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    broadcastPing1(port_num);
  }
  else
  {
    broadcastPing2(port_num);
  }
}

uint8_t getBroadcastPingResult(int port_num, int protocol_version, int id)
{
  if (protocol_version == 1)
  {
    return getBroadcastPingResult1(port_num, id);
  }
  else
  {
    return getBroadcastPingResult2(port_num, id);
  }
}

#if defined(__APPLE__)
void rebootDXL(int port_num, int protocol_version, uint8_t id)
#else
void reboot(int port_num, int protocol_version, uint8_t id)
#endif
{
  if (protocol_version == 1)
  {
    reboot1(port_num, id);
  }
  else
  {
    reboot2(port_num, id);
  }
}

void clearMultiTurn(int port_num, int protocol_version, uint8_t id)
{
  if (protocol_version == 1)
  {
    clearMultiTurn1(port_num, id);
  }
  else
  {
    clearMultiTurn2(port_num, id);
  }
}

void factoryReset(int port_num, int protocol_version, uint8_t id, uint8_t option)
{
  if (protocol_version == 1)
  {
    factoryReset1(port_num, id, option);
  }
  else
  {
    factoryReset2(port_num, id, option);
  }
}

void readTx(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length)
{
  if (protocol_version == 1)
  {
    readTx1(port_num, id, address, length);
  }
  else
  {
    readTx2(port_num, id, address, length);
  }
}
void readRx(int port_num, int protocol_version, uint16_t length)
{
  if (protocol_version == 1)
  {
    readRx1(port_num, length);
  }
  else
  {
    readRx2(port_num, length);
  }
}
void readTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length)
{
  if (protocol_version == 1)
  {
    readTxRx1(port_num, id, address, length);
  }
  else
  {
    readTxRx2(port_num, id, address, length);
  }
}

void read1ByteTx(int port_num, int protocol_version, uint8_t id, uint16_t address)
{
  if (protocol_version == 1)
  {
    read1ByteTx1(port_num, id, address);
  }
  else
  {
    read1ByteTx2(port_num, id, address);
  }
}
uint8_t read1ByteRx(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    return read1ByteRx1(port_num);
  }
  else
  {
    return read1ByteRx2(port_num);
  }
}
uint8_t read1ByteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address)
{
  if (protocol_version == 1)
  {
    return read1ByteTxRx1(port_num, id, address);
  }
  else
  {
    return read1ByteTxRx2(port_num, id, address);
  }
}

void read2ByteTx(int port_num, int protocol_version, uint8_t id, uint16_t address)
{
  if (protocol_version == 1)
  {
    read2ByteTx1(port_num, id, address);
  }
  else
  {
    read2ByteTx2(port_num, id, address);
  }
}
uint16_t read2ByteRx(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    return read2ByteRx1(port_num);
  }
  else
  {
    return read2ByteRx2(port_num);
  }
}
uint16_t read2ByteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address)
{
  if (protocol_version == 1)
  {
    return read2ByteTxRx1(port_num, id, address);
  }
  else
  {
    return read2ByteTxRx2(port_num, id, address);
  }
}

void read4ByteTx(int port_num, int protocol_version, uint8_t id, uint16_t address)
{
  if (protocol_version == 1)
  {
    read4ByteTx1(port_num, id, address);
  }
  else
  {
    read4ByteTx2(port_num, id, address);
  }
}
uint32_t read4ByteRx(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    return read4ByteRx1(port_num);
  }
  else
  {
    return read4ByteRx2(port_num);
  }
}
uint32_t read4ByteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address)
{
  if (protocol_version == 1)
  {
    return read4ByteTxRx1(port_num, id, address);
  }
  else
  {
    return read4ByteTxRx2(port_num, id, address);
  }
}

void writeTxOnly(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length)
{
  if (protocol_version == 1)
  {
    writeTxOnly1(port_num, id, address, length);
  }
  else
  {
    writeTxOnly2(port_num, id, address, length);
  }
}
void writeTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length)
{
  if (protocol_version == 1)
  {
    writeTxRx1(port_num, id, address, length);
  }
  else
  {
    writeTxRx2(port_num, id, address, length);
  }
}

void write1ByteTxOnly(int port_num, int protocol_version, uint8_t id, uint16_t address, uint8_t data)
{
  if (protocol_version == 1)
  {
    write1ByteTxOnly1(port_num, id, address, data);
  }
  else
  {
    write1ByteTxOnly2(port_num, id, address, data);
  }
}
void write1ByteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address, uint8_t data)
{
  if (protocol_version == 1)
  {
    write1ByteTxRx1(port_num, id, address, data);
  }
  else
  {
    write1ByteTxRx2(port_num, id, address, data);
  }
}

void write2ByteTxOnly(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t data)
{
  if (protocol_version == 1)
  {
    write2ByteTxOnly1(port_num, id, address, data);
  }
  else
  {
    write2ByteTxOnly2(port_num, id, address, data);
  }
}
void write2ByteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t data)
{
  if (protocol_version == 1)
  {
    write2ByteTxRx1(port_num, id, address, data);
  }
  else
  {
    write2ByteTxRx2(port_num, id, address, data);
  }
}

void write4ByteTxOnly(int port_num, int protocol_version, uint8_t id, uint16_t address, uint32_t data)
{
  if (protocol_version == 1)
  {
    write4ByteTxOnly1(port_num, id, address, data);
  }
  else
  {
    write4ByteTxOnly2(port_num, id, address, data);
  }
}
void write4ByteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address, uint32_t data)
{
  if (protocol_version == 1)
  {
    write4ByteTxRx1(port_num, id, address, data);
  }
  else
  {
    write4ByteTxRx2(port_num, id, address, data);
  }
}

void regWriteTxOnly(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length)
{
  if (protocol_version == 1)
  {
    regWriteTxOnly1(port_num, id, address, length);
  }
  else
  {
    regWriteTxOnly2(port_num, id, address, length);
  }
}
void regWriteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length)
{
  if (protocol_version == 1)
  {
    regWriteTxRx1(port_num, id, address, length);
  }
  else
  {
    regWriteTxRx2(port_num, id, address, length);
  }
}

void syncReadTx(int port_num, int protocol_version, uint16_t start_address, uint16_t data_length, uint16_t param_length)
{
  if (protocol_version == 1)
  {
    syncReadTx1(port_num, start_address, data_length, param_length);
  }
  else
  {
    syncReadTx2(port_num, start_address, data_length, param_length);
  }
}
// syncReadRx   -> GroupSyncRead
// syncReadTxRx -> GroupSyncRead

void syncWriteTxOnly(int port_num, int protocol_version, uint16_t start_address, uint16_t data_length, uint16_t param_length)
{
  if (protocol_version == 1)
  {
    syncWriteTxOnly1(port_num, start_address, data_length, param_length);
  }
  else
  {
    syncWriteTxOnly2(port_num, start_address, data_length, param_length);
  }
}

void bulkReadTx(int port_num, int protocol_version, uint16_t param_length)
{
  if (protocol_version == 1)
  {
    bulkReadTx1(port_num, param_length);
  }
  else
  {
    bulkReadTx2(port_num, param_length);
  }
}
// bulkReadRx   -> GroupBulkRead
// bulkReadTxRx -> GroupBulkRead

void bulkWriteTxOnly(int port_num, int protocol_version, uint16_t param_length)
{
  if (protocol_version == 1)
  {
    bulkWriteTxOnly1(port_num, param_length);
  }
  else
  {
    bulkWriteTxOnly2(port_num, param_length);
  }
}
