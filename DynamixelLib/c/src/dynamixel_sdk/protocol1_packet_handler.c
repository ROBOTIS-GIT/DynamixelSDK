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

#if defined(__linux__)
#include "protocol1_packet_handler.h"
#elif defined(__APPLE__)
#include "protocol1_packet_handler.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "protocol1_packet_handler.h"
#endif

#include <string.h>
#include <stdlib.h>

#define TXPACKET_MAX_LEN    (250)
#define RXPACKET_MAX_LEN    (250)

///////////////// for Protocol 1.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_ID                  2
#define PKT_LENGTH              3
#define PKT_INSTRUCTION         4
#define PKT_ERROR               4
#define PKT_PARAMETER0          5

///////////////// Protocol 1.0 Error bit /////////////////
#define ERRBIT_VOLTAGE          1       // Supplied voltage is out of the range (operating volatage set in the control table)
#define ERRBIT_ANGLE            2       // Goal position is written out of the range (from CW angle limit to CCW angle limit)
#define ERRBIT_OVERHEAT         4       // Temperature is out of the range (operating temperature set in the control table)
#define ERRBIT_RANGE            8       // Command(setting value) is out of the range for use.
#define ERRBIT_CHECKSUM         16      // Instruction packet checksum is incorrect.
#define ERRBIT_OVERLOAD         32      // The current load cannot be controlled by the set torque.
#define ERRBIT_INSTRUCTION      64      // Undefined instruction or delivering the action command without the reg_write command.

const char *getTxRxResult1(int result)
{
  switch (result)
  {
    case COMM_SUCCESS:
      return "[TxRxResult] Communication success.";

    case COMM_PORT_BUSY:
      return "[TxRxResult] Port is in use!";

    case COMM_TX_FAIL:
      return "[TxRxResult] Failed transmit instruction packet!";

    case COMM_RX_FAIL:
      return "[TxRxResult] Failed get status packet from device!";

    case COMM_TX_ERROR:
      return "[TxRxResult] Incorrect instruction packet!";

    case COMM_RX_WAITING:
      return "[TxRxResult] Now recieving status packet!";

    case COMM_RX_TIMEOUT:
      return "[TxRxResult] There is no status packet!";

    case COMM_RX_CORRUPT:
      return "[TxRxResult] Incorrect status packet!";

    case COMM_NOT_AVAILABLE:
      return "[TxRxResult] Protocol does not support This function!";

    default:
      return "";
  }
}

const char *getRxPacketError1(uint8_t error)
{
  if (error & ERRBIT_VOLTAGE)
    return "[RxPacketError] Input voltage error!";

  if (error & ERRBIT_ANGLE)
    return "[RxPacketError] Angle limit error!";

  if (error & ERRBIT_OVERHEAT)
    return "[RxPacketError] Overheat error!";

  if (error & ERRBIT_RANGE)
    return "[RxPacketError] Out of range error!";

  if (error & ERRBIT_CHECKSUM)
    return "[RxPacketError] Checksum error!";

  if (error & ERRBIT_OVERLOAD)
    return "[RxPacketError] Overload error!";

  if (error & ERRBIT_INSTRUCTION)
    return "[RxPacketError] Instruction code error!";

  return "";
}

int getLastTxRxResult1(int port_num)
{
  return packetData[port_num].communication_result;
}
uint8_t getLastRxPacketError1(int port_num)
{
  return packetData[port_num].error;
}

void setDataWrite1(int port_num, uint16_t data_length, uint16_t data_pos, uint32_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, (data_pos + data_length) * sizeof(uint8_t));

  switch (data_length)
  {
    case 1:
      packetData[port_num].data_write[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      break;

    case 2:
      packetData[port_num].data_write[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      packetData[port_num].data_write[data_pos + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      break;

    default:
      printf("[Set Data for Write] failed");
      break;
  }
}

uint32_t getDataRead1(int port_num, uint16_t data_length, uint16_t data_pos)
{
  switch (data_length)
  {
  case 1:
    return packetData[port_num].data_read[data_pos + 0];

  case 2:
    return DXL_MAKEWORD(packetData[port_num].data_read[data_pos + 0], packetData[port_num].data_read[data_pos + 1]);

  default:
    printf("[Set Data Read] failed... ");
    return 0;
  }
}

void txPacket1(int port_num)
{
  uint16_t idx;

  uint8_t checksum = 0;
  uint8_t total_packet_length = packetData[port_num].tx_packet[PKT_LENGTH] + 4; // 4: HEADER0 HEADER1 ID LENGTH
  uint8_t written_packet_length = 0;

  if (g_is_using[port_num])
  {
    packetData[port_num].communication_result = COMM_PORT_BUSY;
    return ;
  }
  g_is_using[port_num] = True;

  // check max packet length
  if (total_packet_length > TXPACKET_MAX_LEN)
  {
    g_is_using[port_num] = False;
    packetData[port_num].communication_result = COMM_TX_ERROR;
    return;
  }

  // make packet header
  packetData[port_num].tx_packet[PKT_HEADER0] = 0xFF;
  packetData[port_num].tx_packet[PKT_HEADER1] = 0xFF;

  // add a checksum to the packet
  for (idx = 2; idx < total_packet_length - 1; idx++)   // except header, checksum
  {
    checksum += packetData[port_num].tx_packet[idx];
  }
  packetData[port_num].tx_packet[total_packet_length - 1] = ~checksum;

  // tx packet
  clearPort(port_num);
  written_packet_length = writePort(port_num, packetData[port_num].tx_packet, total_packet_length);
  if (total_packet_length != written_packet_length)
  {
    g_is_using[port_num] = False;
    packetData[port_num].communication_result = COMM_TX_FAIL;
    return;
  }

  packetData[port_num].communication_result = COMM_SUCCESS;
}

void rxPacket1(int port_num)
{
  uint16_t idx, s;
  int i;
  uint8_t checksum;
  uint8_t rx_length;
  uint8_t wait_length;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  checksum = 0;
  rx_length = 0;
  wait_length = 6;    // minimum length ( HEADER0 HEADER1 ID LENGTH ERROR CHKSUM )

  while (True)
  {
    rx_length += readPort(port_num, &packetData[port_num].rx_packet[rx_length], wait_length - rx_length);
    if (rx_length >= wait_length)
    {
      idx = 0;

      // find packet header
      for (idx = 0; idx < (rx_length - 1); idx++)
      {
        if (packetData[port_num].rx_packet[idx] == 0xFF && packetData[port_num].rx_packet[idx + 1] == 0xFF)
          break;
      }

      if (idx == 0)   // found at the beginning of the packet
      {
        if (packetData[port_num].rx_packet[PKT_ID] > 0xFD ||                   // unavailable ID
            packetData[port_num].rx_packet[PKT_LENGTH] > RXPACKET_MAX_LEN ||   // unavailable Length
            packetData[port_num].rx_packet[PKT_ERROR] > 0x7F)                  // unavailable Error
        {
          // remove the first byte in the packet
          for (s = 0; s < rx_length - 1; s++)
          {
            packetData[port_num].rx_packet[s] = packetData[port_num].rx_packet[1 + s];
          }

          rx_length -= 1;
          continue;
        }

        // re-calculate the exact length of the rx packet
        wait_length = packetData[port_num].rx_packet[PKT_LENGTH] + PKT_LENGTH + 1;
        if (rx_length < wait_length)
        {
          // check timeout
          if (isPacketTimeout(port_num) == True)
          {
            if (rx_length == 0)
              packetData[port_num].communication_result = COMM_RX_TIMEOUT;
            else
              packetData[port_num].communication_result = COMM_RX_CORRUPT;
            break;
          }
          else
          {
            continue;
          }
        }

        // calculate checksum
        for (i = 2; i < wait_length - 1; i++)   // except header, checksum
        {
          checksum += packetData[port_num].rx_packet[i];
        }
        checksum = ~checksum;

        // verify checksum
        if (packetData[port_num].rx_packet[wait_length - 1] == checksum)
        {
          packetData[port_num].communication_result = COMM_SUCCESS;
        }
        else
        {
          packetData[port_num].communication_result = COMM_RX_CORRUPT;
        }
        break;
      }
      else
      {
        // remove unnecessary packets
        for (s = 0; s < rx_length - idx; s++)
        {
          packetData[port_num].rx_packet[s] = packetData[port_num].rx_packet[idx + s];
        }
        rx_length -= idx;
      }
    }
    else
    {
      // check timeout
      if (isPacketTimeout(port_num) == True)
      {
        if (rx_length == 0)
        {
          packetData[port_num].communication_result = COMM_RX_TIMEOUT;
        }
        else
        {
          packetData[port_num].communication_result = COMM_RX_CORRUPT;
        }
        break;
      }
    }
  }
  g_is_using[port_num] = False;
}

// NOT for BulkRead instruction
void txRxPacket1(int port_num)
{
  packetData[port_num].communication_result = COMM_TX_FAIL;

  // tx packet
  txPacket1(port_num);

  if (packetData[port_num].communication_result != COMM_SUCCESS)
    return;

  // (ID == Broadcast ID && NOT BulkRead) == no need to wait for status packet
  // (Instruction == Action) == no need to wait for status packet
  if ((packetData[port_num].tx_packet[PKT_ID] == BROADCAST_ID && packetData[port_num].tx_packet[PKT_INSTRUCTION] != INST_BULK_READ) ||
    (packetData[port_num].tx_packet[PKT_INSTRUCTION] == INST_ACTION))
  {
    g_is_using[port_num] = False;
    return;
  }

  // set packet timeout
  if (packetData[port_num].tx_packet[PKT_INSTRUCTION] == INST_READ)
  {
    setPacketTimeout(port_num, (uint16_t)(packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] + 6));
  }
  else
  {
    setPacketTimeout(port_num, (uint16_t)6);
  }

  // rx packet
  rxPacket1(port_num);
  // check txpacket ID == rxpacket ID
  if (packetData[port_num].tx_packet[PKT_ID] != packetData[port_num].rx_packet[PKT_ID])
    rxPacket1(port_num);

  if (packetData[port_num].communication_result == COMM_SUCCESS && packetData[port_num].tx_packet[PKT_ID] != BROADCAST_ID)
  {
    if (packetData[port_num].error != 0)
      packetData[port_num].error = (uint8_t)packetData[port_num].rx_packet[PKT_ERROR];
  }
}

void ping1(int port_num, uint8_t id)
{
  pingGetModelNum1(port_num, id);
}

uint16_t pingGetModelNum1(int port_num, uint8_t id)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 2 * sizeof(uint8_t));
  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 6);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 6);

  if (id >= BROADCAST_ID)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return 0;
  }

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = 2;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_PING;

  txRxPacket1(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
  {
    readTxRx1(port_num, id, 0, 2);  // Address 0 : Model Number
    if (packetData[port_num].communication_result == COMM_SUCCESS)
      return DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]);
  }

  return 0;
}

void broadcastPing1(int port_num)
{
  packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
}

uint8_t getBroadcastPingResult1(int port_num, int id)
{
  return False;
}

void action1(int port_num, uint8_t id)
{
  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 6);

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = 2;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_ACTION;

  txRxPacket1(port_num);
}

void reboot1(int port_num, uint8_t id)
{
  packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
}

void clearMultiTurn1(int port_num, uint8_t id)
{
  packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
}

void factoryReset1(int port_num, uint8_t id, uint8_t option)
{
  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 6);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 6);

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = 2;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_FACTORY_RESET;

  txRxPacket1(port_num);
}

void readTx1(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 8);

  if (id >= BROADCAST_ID)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = 4;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_READ;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = (uint8_t)address;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] = (uint8_t)length;

  txPacket1(port_num);

  // set packet timeout
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    setPacketTimeout(port_num, (uint16_t)(length + 6));
}

void readRx1(int port_num, uint16_t length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, RXPACKET_MAX_LEN);

  rxPacket1(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
  {
    if (packetData[port_num].error != 0)
      packetData[port_num].error = (uint8_t)packetData[port_num].rx_packet[PKT_ERROR];
    for (s = 0; s < length; s++)
    {
      packetData[port_num].data_read[s] = packetData[port_num].rx_packet[PKT_PARAMETER0 + s];
    }
  }
}

void readTxRx1(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint16_t s;
  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 8);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, RXPACKET_MAX_LEN);

  if (id >= BROADCAST_ID)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = 4;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_READ;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = (uint8_t)address;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] = (uint8_t)length;

  txRxPacket1(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
  {
    if (packetData[port_num].error != 0)
      packetData[port_num].error = (uint8_t)packetData[port_num].rx_packet[PKT_ERROR];
    for (s = 0; s < length; s++)
    {
      packetData[port_num].data_read[s] = packetData[port_num].rx_packet[PKT_PARAMETER0 + s];
    }
  }
}

void read1ByteTx1(int port_num, uint8_t id, uint16_t address)
{
  readTx1(port_num, id, address, 1);
}
uint8_t read1ByteRx1(int port_num)
{
	packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 1 * sizeof(uint8_t));
  packetData[port_num].data_read[0] = 0;
  readRx1(port_num, 1);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return packetData[port_num].data_read[0];
  return 0;
}
uint8_t read1ByteTxRx1(int port_num, uint8_t id, uint16_t address)
{
	packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 1 * sizeof(uint8_t));
  packetData[port_num].data_read[0] = 0;
  readTxRx1(port_num, id, address, 1);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return packetData[port_num].data_read[0];
  return 0;
}

void read2ByteTx1(int port_num, uint8_t id, uint16_t address)
{
  readTx1(port_num, id, address, 2);
}
uint16_t read2ByteRx1(int port_num)
{
	packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 2 * sizeof(uint8_t));
  packetData[port_num].data_read[0] = 0;
  packetData[port_num].data_read[1] = 0;
  readRx1(port_num, 2);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]);
  return 0;
}
uint16_t read2ByteTxRx1(int port_num, uint8_t id, uint16_t address)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 2 * sizeof(uint8_t));
	packetData[port_num].data_read[0] = 0;
  packetData[port_num].data_read[1] = 0;
  readTxRx1(port_num, id, address, 2);

  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]);

  return 0;
}

void read4ByteTx1(int port_num, uint8_t id, uint16_t address)
{
  readTx1(port_num, id, address, 4);
}
uint32_t read4ByteRx1(int port_num)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 4 * sizeof(uint8_t));
  packetData[port_num].data_read[0] = 0;
  packetData[port_num].data_read[1] = 0;
  packetData[port_num].data_read[2] = 0;
  packetData[port_num].data_read[3] = 0;
  readRx1(port_num, 4);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEDWORD(DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]), DXL_MAKEWORD(packetData[port_num].data_read[2], packetData[port_num].data_read[3]));
  return 0;
}
uint32_t read4ByteTxRx1(int port_num, uint8_t id, uint16_t address)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 4 * sizeof(uint8_t));
  packetData[port_num].data_read[0] = 0;
  packetData[port_num].data_read[1] = 0;
  packetData[port_num].data_read[2] = 0;
  packetData[port_num].data_read[3] = 0;
  readTxRx1(port_num, id, address, 4);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEDWORD(DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]), DXL_MAKEWORD(packetData[port_num].data_read[2], packetData[port_num].data_read[3]));
  return 0;
}

void writeTxOnly1(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 7);

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = length + 3;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0] = (uint8_t)address;

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 1 + s] = packetData[port_num].data_write[s];
  }

  txPacket1(port_num);
  g_is_using[port_num] = False;
}

void writeTxRx1(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 7);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 6);

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = length + 3;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0] = (uint8_t)address;

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 1 + s] = packetData[port_num].data_write[s];
  }

  txRxPacket1(port_num);
}

void write1ByteTxOnly1(int port_num, uint8_t id, uint16_t address, uint8_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 1 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = data;
  writeTxOnly1(port_num, id, address, 1);
}
void write1ByteTxRx1(int port_num, uint8_t id, uint16_t address, uint8_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 1 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = data;
  writeTxRx1(port_num, id, address, 1);
}

void write2ByteTxOnly1(int port_num, uint8_t id, uint16_t address, uint16_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 2 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = DXL_LOBYTE(data);
  packetData[port_num].data_write[1] = DXL_HIBYTE(data);
  writeTxOnly1(port_num, id, address, 2);
}
void write2ByteTxRx1(int port_num, uint8_t id, uint16_t address, uint16_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 2 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = DXL_LOBYTE(data);
  packetData[port_num].data_write[1] = DXL_HIBYTE(data);
  writeTxRx1(port_num, id, address, 2);
}

void write4ByteTxOnly1(int port_num, uint8_t id, uint16_t address, uint32_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 4 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = DXL_LOBYTE(DXL_LOWORD(data));
  packetData[port_num].data_write[1] = DXL_HIBYTE(DXL_LOWORD(data));
  packetData[port_num].data_write[2] = DXL_LOBYTE(DXL_HIWORD(data));
  packetData[port_num].data_write[3] = DXL_HIBYTE(DXL_HIWORD(data));
  writeTxOnly1(port_num, id, address, 4);
}
void write4ByteTxRx1(int port_num, uint8_t id, uint16_t address, uint32_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 4 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = DXL_LOBYTE(DXL_LOWORD(data));
  packetData[port_num].data_write[1] = DXL_HIBYTE(DXL_LOWORD(data));
  packetData[port_num].data_write[2] = DXL_LOBYTE(DXL_HIWORD(data));
  packetData[port_num].data_write[3] = DXL_HIBYTE(DXL_HIWORD(data));
  writeTxRx1(port_num, id, address, 4);
}

void regWriteTxOnly1(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 6);

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = length + 3;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_REG_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0] = (uint8_t)address;

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 1 + s] = packetData[port_num].data_write[s];
  }

  txPacket1(port_num);
  
  g_is_using[port_num] = False;
}

void regWriteTxRx1(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 6);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 6);

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = length + 3;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_REG_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0] = (uint8_t)address;

  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, length * sizeof(uint8_t));

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 1 + s] = packetData[port_num].data_write[s];
  }

  txRxPacket1(port_num);
}

void syncReadTx1(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length)
{
  packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
}

void syncWriteTxOnly1(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, param_length + 8); // 8: HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN ... CHKSUM

  packetData[port_num].tx_packet[PKT_ID] = BROADCAST_ID;
  packetData[port_num].tx_packet[PKT_LENGTH] = param_length + 4; // 4: INST START_ADDR DATA_LEN ... CHKSUM
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_SYNC_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = start_address;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] = data_length;

  for (s = 0; s < param_length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 2 + s] = packetData[port_num].data_write[s];
  }

  txRxPacket1(port_num);
}

void bulkReadTx1(int port_num, uint16_t param_length)
{
  uint16_t s;
  int i;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, param_length + 7);  // 7: HEADER0 HEADER1 ID LEN INST 0x00 ... CHKSUM

  packetData[port_num].tx_packet[PKT_ID] = BROADCAST_ID;
  packetData[port_num].tx_packet[PKT_LENGTH] = param_length + 3; // 3: INST 0x00 ... CHKSUM
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_BULK_READ;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = 0x00;

  for (s = 0; s < param_length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 1 + s] = packetData[port_num].data_write[s];
  }

  txPacket1(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
  {
    int wait_length = 0;
    for (i = 0; i < param_length; i += 3)
    {
      wait_length += packetData[port_num].data_write[i] + 7;
    }

    setPacketTimeout(port_num, (uint16_t)wait_length);
  }
}

void bulkWriteTxOnly1(int port_num, uint16_t param_length)
{
  packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
}
