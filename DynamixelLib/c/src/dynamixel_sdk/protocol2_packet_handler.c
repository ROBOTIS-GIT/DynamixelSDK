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
#include <unistd.h>
#include "protocol2_packet_handler.h"
#elif defined(__APPLE__)
#include <unistd.h>
#include "protocol2_packet_handler.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include <Windows.h>
#include "protocol2_packet_handler.h"
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define TXPACKET_MAX_LEN    (1*1024)
#define RXPACKET_MAX_LEN    (1*1024)

///////////////// for Protocol 2.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_HEADER2             2
#define PKT_RESERVED            3
#define PKT_ID                  4
#define PKT_LENGTH_L            5
#define PKT_LENGTH_H            6
#define PKT_INSTRUCTION         7
#define PKT_ERROR               8
#define PKT_PARAMETER0          8

///////////////// Protocol 2.0 Error bit /////////////////
#define ERRNUM_RESULT_FAIL      1       // Failed to process the instruction packet.
#define ERRNUM_INSTRUCTION      2       // Instruction error
#define ERRNUM_CRC              3       // CRC check error
#define ERRNUM_DATA_RANGE       4       // Data range error
#define ERRNUM_DATA_LENGTH      5       // Data length error
#define ERRNUM_DATA_LIMIT       6       // Data limit error
#define ERRNUM_ACCESS           7       // Access error

#define ERRBIT_ALERT            128     //When the device has a problem, this bit is set to 1. Check "Device Status Check" value.

const char *getTxRxResult2(int result)
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

const char *getRxPacketError2(uint8_t error)
{
  int not_alert_error;
  if (error & ERRBIT_ALERT)
    return "[RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!";

  not_alert_error = error & ~ERRBIT_ALERT;

  switch (not_alert_error)
  {
    case 0:
      return "";

    case ERRNUM_RESULT_FAIL:
      return "[RxPacketError] Failed to process the instruction packet!";

    case ERRNUM_INSTRUCTION:
      return "[RxPacketError] Undefined instruction or incorrect instruction!";

    case ERRNUM_CRC:
      return "[RxPacketError] CRC doesn't match!";

    case ERRNUM_DATA_RANGE:
      return "[RxPacketError] The data value is out of range!";

    case ERRNUM_DATA_LENGTH:
      return "[RxPacketError] The data length does not match as expected!";

    case ERRNUM_DATA_LIMIT:
      return "[RxPacketError] The data value exceeds the limit value!";

    case ERRNUM_ACCESS:
      return "[RxPacketError] Writing or Reading is not available to target address!";

    default:
      return "[RxPacketError] Unknown error code!";
  }
}

int getLastTxRxResult2(int port_num)
{
  return packetData[port_num].communication_result;
}
uint8_t getLastRxPacketError2(int port_num)
{
  return packetData[port_num].error;
}

void setDataWrite2(int port_num, uint16_t data_length, uint16_t data_pos, uint32_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, (data_pos + data_length) * sizeof(uint8_t));
  if (packetData[port_num].data_write == NULL)
  {
    printf("[Set Data Write] memory allocation failed... \n");
    return;
  }

  switch (data_length)
  {
    case 1:
      packetData[port_num].data_write[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      break;

    case 2:
      packetData[port_num].data_write[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      packetData[port_num].data_write[data_pos + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      break;

    case 4:
      packetData[port_num].data_write[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      packetData[port_num].data_write[data_pos + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      packetData[port_num].data_write[data_pos + 2] = DXL_LOBYTE(DXL_HIWORD(data));
      packetData[port_num].data_write[data_pos + 3] = DXL_HIBYTE(DXL_HIWORD(data));
      break;

    default:
      printf("[Set Data Write] failed... \n");
      break;
  }
}
uint32_t getDataRead2(int port_num, uint16_t data_length, uint16_t data_pos)
{
  switch (data_length)
  {
  case 1:
    return packetData[port_num].data_read[data_pos + 0];

  case 2:
    return DXL_MAKEWORD(packetData[port_num].data_read[data_pos + 0], packetData[port_num].data_read[data_pos + 1]);

  case 4:
    return DXL_MAKEDWORD(DXL_MAKEWORD(packetData[port_num].data_read[data_pos + 0], packetData[port_num].data_read[data_pos + 1])
      , DXL_MAKEWORD(packetData[port_num].data_read[data_pos + 2], packetData[port_num].data_read[data_pos + 3]));

  default:
    printf("[Set Data Read] failed... \n");
    return 0;
  }
}

unsigned short updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
  uint16_t i, j;
  static const uint16_t crc_table[256] = { 0x0000,
    0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
    0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
    0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
    0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
    0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
    0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
    0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
    0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
    0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
    0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
    0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
    0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
    0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
    0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
    0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
    0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
    0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
    0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
    0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
    0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
    0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
    0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
    0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
    0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
    0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
    0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
    0x820D, 0x8207, 0x0202 };

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}

void addStuffing(uint8_t *packet)
{
  uint8_t *packet_ptr;
  uint16_t i;
  uint16_t packet_length_before_crc;
  uint16_t out_index, in_index;
  
  int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
  int packet_length_out = packet_length_in;
  
  if (packet_length_in < 8) // INSTRUCTION, ADDR_L, ADDR_H, CRC16_L, CRC16_H + FF FF FD
    return;
  
  packet_length_before_crc = packet_length_in - 2;
  for (i = 3; i < packet_length_before_crc; i++)
  {
    packet_ptr = &packet[i+PKT_INSTRUCTION-2];
    if (packet_ptr[0] == 0xFF && packet_ptr[1] == 0xFF && packet_ptr[2] == 0xFD)
      packet_length_out++;
  }
  
  if (packet_length_in == packet_length_out)  // no stuffing required
    return;
  
  out_index  = packet_length_out + 6 - 2;  // last index before crc
  in_index   = packet_length_in + 6 - 2;   // last index before crc
  while (out_index != in_index)
  {
    if (packet[in_index] == 0xFD && packet[in_index-1] == 0xFF && packet[in_index-2] == 0xFF)
    {
      packet[out_index--] = 0xFD; // byte stuffing
      if (out_index != in_index)
      {
        packet[out_index--] = packet[in_index--]; // FD
        packet[out_index--] = packet[in_index--]; // FF
        packet[out_index--] = packet[in_index--]; // FF
      }
    }
    else
    {
      packet[out_index--] = packet[in_index--];
    }
  }

  packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
  packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);

  return;
}

void removeStuffing(uint8_t *packet)
{
  uint16_t i = 0;
  int index = 0;
  int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
  int packet_length_out = packet_length_in;

  index = PKT_INSTRUCTION;
  for (i = 0; i < packet_length_in - 2; i++)  // except CRC
  {
    if (packet[i + PKT_INSTRUCTION] == 0xFD && packet[i + PKT_INSTRUCTION + 1] == 0xFD && packet[i + PKT_INSTRUCTION - 1] == 0xFF && packet[i + PKT_INSTRUCTION - 2] == 0xFF)
    {   // FF FF FD FD
      packet_length_out--;
      i++;
    }
    packet[index++] = packet[i + PKT_INSTRUCTION];
  }
  packet[index++] = packet[PKT_INSTRUCTION + packet_length_in - 2];
  packet[index++] = packet[PKT_INSTRUCTION + packet_length_in - 1];

  packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
  packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

void txPacket2(int port_num)
{
  uint16_t total_packet_length = 0;
  uint16_t written_packet_length = 0;
  uint16_t crc;

  if (g_is_using[port_num])
  {
    packetData[port_num].communication_result = COMM_PORT_BUSY;
    return;
  }
  g_is_using[port_num] = True;

  // byte stuffing for header
  addStuffing(packetData[port_num].tx_packet);

  // check max packet length
  total_packet_length = DXL_MAKEWORD(packetData[port_num].tx_packet[PKT_LENGTH_L], packetData[port_num].tx_packet[PKT_LENGTH_H]) + 7;
  // 7: HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H
  if (total_packet_length > TXPACKET_MAX_LEN)
  {
    g_is_using[port_num] = False;
    packetData[port_num].communication_result = COMM_TX_ERROR;
    return;
  }

  // make packet header
  packetData[port_num].tx_packet[PKT_HEADER0] = 0xFF;
  packetData[port_num].tx_packet[PKT_HEADER1] = 0xFF;
  packetData[port_num].tx_packet[PKT_HEADER2] = 0xFD;
  packetData[port_num].tx_packet[PKT_RESERVED] = 0x00;

  // add CRC16
  crc = updateCRC(0, packetData[port_num].tx_packet, total_packet_length - 2);    // 2: CRC16
  packetData[port_num].tx_packet[total_packet_length - 2] = DXL_LOBYTE(crc);
  packetData[port_num].tx_packet[total_packet_length - 1] = DXL_HIBYTE(crc);

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

void rxPacket2(int port_num)
{
  uint16_t s;
  uint16_t idx;
  uint16_t rx_length = 0;
  uint16_t wait_length = 11;
  // minimum length ( HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H )
  uint16_t crc;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  while (True)
  {
    rx_length += readPort(port_num, &packetData[port_num].rx_packet[rx_length], wait_length - rx_length);
    if (rx_length >= wait_length)
    {
      idx = 0;

      // find packet header
      for (idx = 0; idx < (rx_length - 3); idx++)
      {
        if ((packetData[port_num].rx_packet[idx] == 0xFF) && (packetData[port_num].rx_packet[idx + 1] == 0xFF) && (packetData[port_num].rx_packet[idx + 2] == 0xFD) && (packetData[port_num].rx_packet[idx + 3] != 0xFD))
          break;
      }

      if (idx == 0)   // found at the beginning of the packet
      {
        if (packetData[port_num].rx_packet[PKT_RESERVED] != 0x00 ||
          packetData[port_num].rx_packet[PKT_ID] > 0xFC ||
          DXL_MAKEWORD(packetData[port_num].rx_packet[PKT_LENGTH_L], packetData[port_num].rx_packet[PKT_LENGTH_H]) > RXPACKET_MAX_LEN ||
          packetData[port_num].rx_packet[PKT_INSTRUCTION] != 0x55)
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
        if (wait_length != DXL_MAKEWORD(packetData[port_num].rx_packet[PKT_LENGTH_L], packetData[port_num].rx_packet[PKT_LENGTH_H]) + PKT_LENGTH_H + 1)
        {
          wait_length = DXL_MAKEWORD(packetData[port_num].rx_packet[PKT_LENGTH_L], packetData[port_num].rx_packet[PKT_LENGTH_H]) + PKT_LENGTH_H + 1;
          continue;
        }

        if (rx_length < wait_length)
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
          else
          {
            continue;
          }
        }

        // verify CRC16
        crc = DXL_MAKEWORD(packetData[port_num].rx_packet[wait_length - 2], packetData[port_num].rx_packet[wait_length - 1]);
        if (updateCRC(0, packetData[port_num].rx_packet, wait_length - 2) == crc)
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
#if defined(__linux__) || defined(__APPLE__)
    usleep(0);
#elif defined(_WIN32) || defined(_WIN64)
    Sleep(0);
#endif
  }
  g_is_using[port_num] = False;

  if (packetData[port_num].communication_result == COMM_SUCCESS)
    removeStuffing(packetData[port_num].rx_packet);
}

// NOT for BulkRead / SyncRead instruction
void txRxPacket2(int port_num)
{
  packetData[port_num].communication_result = COMM_TX_FAIL;

  // tx packet
  txPacket2(port_num);
  if (packetData[port_num].communication_result != COMM_SUCCESS)
    return;

  // (ID == Broadcast ID && NOT BulkRead) == no need to wait for status packet
  // (Instruction == Action) == no need to wait for status packet
  if ((packetData[port_num].tx_packet[PKT_ID] == BROADCAST_ID && packetData[port_num].tx_packet[PKT_INSTRUCTION] != INST_BULK_READ) ||
    (packetData[port_num].tx_packet[PKT_ID] == BROADCAST_ID && packetData[port_num].tx_packet[PKT_INSTRUCTION] != INST_SYNC_READ) ||
    (packetData[port_num].tx_packet[PKT_INSTRUCTION] == INST_ACTION))
  {
    g_is_using[port_num] = False;
    return;
  }

  // set packet timeout
  if (packetData[port_num].tx_packet[PKT_INSTRUCTION] == INST_READ)
  {
    setPacketTimeout(port_num, (uint16_t)(DXL_MAKEWORD(packetData[port_num].tx_packet[PKT_PARAMETER0 + 2], packetData[port_num].tx_packet[PKT_PARAMETER0 + 3]) + 11));
  }
  else
  {
    setPacketTimeout(port_num, (uint16_t)11);   // HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H
  }

  // rx packet
  rxPacket2(port_num);
  // check txpacket ID == rxpacket ID
  if (packetData[port_num].tx_packet[PKT_ID] != packetData[port_num].rx_packet[PKT_ID])
    rxPacket2(port_num);

  if (packetData[port_num].communication_result == COMM_SUCCESS && packetData[port_num].tx_packet[PKT_ID] != BROADCAST_ID)
  {
    packetData[port_num].error = (uint8_t)packetData[port_num].rx_packet[PKT_ERROR];
  }
}

void ping2(int port_num, uint8_t id)
{
  pingGetModelNum2(port_num, id);
}

uint16_t pingGetModelNum2(int port_num, uint8_t id)
{
  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 10);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 14);
  if (packetData[port_num].tx_packet == NULL || packetData[port_num].rx_packet == NULL)
  {
    printf("[PingGetModeNum] memory allocation failed.. \n");
    return COMM_TX_FAIL;
  }
  
  if (id >= BROADCAST_ID)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return 0;
  }

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = 3;
  packetData[port_num].tx_packet[PKT_LENGTH_H] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_PING;

  txRxPacket2(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEWORD(packetData[port_num].rx_packet[PKT_PARAMETER0 + 1], packetData[port_num].rx_packet[PKT_PARAMETER0 + 2]);
  return 0;
}

void broadcastPing2(int port_num)
{
  uint16_t s;
  int id;
  uint16_t idx;
  const int STATUS_LENGTH     = 14;
  int result = COMM_TX_FAIL;

  uint16_t rx_length = 0;
  uint16_t wait_length = STATUS_LENGTH * MAX_ID;

  double tx_time_per_byte = (1000.0 / (double)getBaudRate(port_num)) * 10.0;

  packetData[port_num].broadcast_ping_id_list = (uint8_t *)calloc(255, sizeof(uint8_t));
  if (packetData[port_num].broadcast_ping_id_list == NULL)
  {
    printf("[BroadcastPing] memory allocation failed.. \n");
    return;
  }

  for (id = 0; id < 255; id++)
  {
    packetData[port_num].broadcast_ping_id_list[id] = 255;
  }

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 10 * sizeof(uint8_t));
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, STATUS_LENGTH * MAX_ID * sizeof(uint8_t));
  if (packetData[port_num].tx_packet == NULL || packetData[port_num].rx_packet == NULL)
  {
    printf("[BroadcastPing] memory allocation failed.. \n");
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = BROADCAST_ID;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = 3;
  packetData[port_num].tx_packet[PKT_LENGTH_H] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_PING;

  txPacket2(port_num);
  if (packetData[port_num].communication_result != COMM_SUCCESS)
  {
    g_is_using[port_num] = False;
    return;
  }

  // set rx timeout
  //setPacketTimeout(port_num, (uint16_t)(wait_length * 30));
  setPacketTimeoutMSec(port_num, ((double)wait_length * tx_time_per_byte) + (3.0 * (double)MAX_ID) + 16.0);

  while (1)
  {
    rx_length += readPort(port_num, &packetData[port_num].rx_packet[rx_length], wait_length - rx_length);
    if (isPacketTimeout(port_num) == True)// || rx_length >= wait_length)
      break;
  }

  g_is_using[port_num] = False;

  if (rx_length == 0)
  {
    packetData[port_num].communication_result = COMM_RX_TIMEOUT;
    return;
  }

  while (1)
  {
    if (rx_length < STATUS_LENGTH)
    {
      packetData[port_num].communication_result = COMM_RX_CORRUPT;
    }

    idx = 0;

    // find packet header
    for (idx = 0; idx < (rx_length - 2); idx++)
    {
      if (packetData[port_num].rx_packet[idx] == 0xFF && packetData[port_num].rx_packet[idx + 1] == 0xFF && packetData[port_num].rx_packet[idx + 2] == 0xFD)
        break;
    }

    if (idx == 0)   // found at the beginning of the packet
    {
      // verify CRC16
      uint16_t crc = DXL_MAKEWORD(packetData[port_num].rx_packet[STATUS_LENGTH - 2], packetData[port_num].rx_packet[STATUS_LENGTH - 1]);

      if (updateCRC(0, packetData[port_num].rx_packet, STATUS_LENGTH - 2) == crc)
      {
        packetData[port_num].communication_result = COMM_SUCCESS;

        packetData[port_num].broadcast_ping_id_list[packetData[port_num].rx_packet[PKT_ID]] = packetData[port_num].rx_packet[PKT_ID];

        for (s = 0; s < rx_length - STATUS_LENGTH; s++)
        {
          packetData[port_num].rx_packet[s] = packetData[port_num].rx_packet[STATUS_LENGTH + s];
        }
        rx_length -= STATUS_LENGTH;

        if (rx_length == 0)
          return;
      }
      else
      {
        result = COMM_RX_CORRUPT;

        // remove header (0xFF 0xFF 0xFD)
        for (s = 0; s < rx_length - 3; s++)
        {
          packetData[port_num].rx_packet[s] = packetData[port_num].rx_packet[3 + s];
        }
        rx_length -= 3;
      }
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

  packetData[port_num].communication_result = result;
  return;
}

uint8_t getBroadcastPingResult2(int port_num, int id)
{
  if (packetData[port_num].broadcast_ping_id_list[id] == id)
  {
    return True;
  }
  else
  {
    return False;
  }
}

void action2(int port_num, uint8_t id)
{
  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 10);
  if (packetData[port_num].tx_packet == NULL)
  {
    printf("[Action] memory allocation failed..\n");
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = 3;
  packetData[port_num].tx_packet[PKT_LENGTH_H] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_ACTION;

  txRxPacket2(port_num);
}

void reboot2(int port_num, uint8_t id)
{
  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 10);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 11);
  if (packetData[port_num].tx_packet == NULL || packetData[port_num].rx_packet == NULL)
  {
    printf("[Reboot] memory allocation failed..\n");
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = 3;
  packetData[port_num].tx_packet[PKT_LENGTH_H] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_REBOOT;

  txRxPacket2(port_num);
}

void clearMultiTurn2(int port_num, uint8_t id)
{
  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 15);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 11);
  if (packetData[port_num].tx_packet == NULL || packetData[port_num].rx_packet == NULL)
  {
    printf("[ClearMultiTurn] memory allocation failed..\n");
    return;
  }
  
  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = 8;
  packetData[port_num].tx_packet[PKT_LENGTH_H] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_CLEAR;
  packetData[port_num].tx_packet[PKT_PARAMETER0] = 0x01;
  packetData[port_num].tx_packet[PKT_PARAMETER0+1] = 0x44;
  packetData[port_num].tx_packet[PKT_PARAMETER0+2] = 0x58;
  packetData[port_num].tx_packet[PKT_PARAMETER0+3] = 0x4C;
  packetData[port_num].tx_packet[PKT_PARAMETER0+4] = 0x22;

  txRxPacket2(port_num);
}

void factoryReset2(int port_num, uint8_t id, uint8_t option)
{
  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 11);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 11);
  if (packetData[port_num].tx_packet == NULL || packetData[port_num].rx_packet == NULL)
  {
    printf("[FactoryReset] memory allocation failed..\n");
    return;
  }
  
  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = 4;
  packetData[port_num].tx_packet[PKT_LENGTH_H] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_FACTORY_RESET;
  packetData[port_num].tx_packet[PKT_PARAMETER0] = option;

  txRxPacket2(port_num);
}

void readTx2(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)malloc(14);
  if (packetData[port_num].tx_packet == NULL)
  {
    printf("[ReadTx] memory allocation failed..\n");
    return;
  }
  
  if (id >= BROADCAST_ID)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = 7;
  packetData[port_num].tx_packet[PKT_LENGTH_H] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_READ;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = (uint8_t)DXL_LOBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] = (uint8_t)DXL_HIBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 2] = (uint8_t)DXL_LOBYTE(length);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 3] = (uint8_t)DXL_HIBYTE(length);

  txPacket2(port_num);

  free(packetData[port_num].tx_packet);

  // set packet timeout
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    setPacketTimeout(port_num, (uint16_t)(length + 11));
}

void readRx2(int port_num, uint16_t length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, RXPACKET_MAX_LEN);  //(length + 11 + (length/3));  // (length/3): consider stuffing
  if (packetData[port_num].rx_packet == NULL)
  {
    printf("[ReadRx] memory allocation failed..\n");
    return;
  }
  
  rxPacket2(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
  {
    if (packetData[port_num].error != 0)
      packetData[port_num].error = (uint8_t)packetData[port_num].rx_packet[PKT_ERROR];
    for (s = 0; s < length; s++)
    {
      packetData[port_num].data_read[s] = packetData[port_num].rx_packet[PKT_PARAMETER0 + 1 + s];
    }
  }
}

void readTxRx2(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 14);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, RXPACKET_MAX_LEN);  //(length + 11 + (length/3));  // (length/3): consider stuffing
  if (packetData[port_num].tx_packet == NULL || packetData[port_num].rx_packet == NULL)
  {
    printf("[ReadTxRx] memory allocation failed..\n");
    return;
  }
  
  if (id >= BROADCAST_ID)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = 7;
  packetData[port_num].tx_packet[PKT_LENGTH_H] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_READ;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = (uint8_t)DXL_LOBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] = (uint8_t)DXL_HIBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 2] = (uint8_t)DXL_LOBYTE(length);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 3] = (uint8_t)DXL_HIBYTE(length);

  txRxPacket2(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
  {
    if (packetData[port_num].error != 0)
      packetData[port_num].error = (uint8_t)packetData[port_num].rx_packet[PKT_ERROR];
    for (s = 0; s < length; s++)
    {
      packetData[port_num].data_read[s] = packetData[port_num].rx_packet[PKT_PARAMETER0 + 1 + s];
    }
  }
}

void read1ByteTx2(int port_num, uint8_t id, uint16_t address)
{
  readTx2(port_num, id, address, 1);
}
uint8_t read1ByteRx2(int port_num)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 1 * sizeof(uint8_t));
  if (packetData[port_num].data_read == NULL)
  {
    printf("[Read1ByteRx] memory allocation failed..\n");
    return 0;
  }
  packetData[port_num].data_read[0] = 0;
  readRx2(port_num, 1);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return packetData[port_num].data_read[0];
  return 0;
}
uint8_t read1ByteTxRx2(int port_num, uint8_t id, uint16_t address)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 1 * sizeof(uint8_t));
  if (packetData[port_num].data_read == NULL)
  {
    printf("[Read1ByteTxRx] memory allocation failed..\n");
    return 0;
  }
  packetData[port_num].data_read[0] = 0;
  readTxRx2(port_num, id, address, 1);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return packetData[port_num].data_read[0];
  return 0;
}

void read2ByteTx2(int port_num, uint8_t id, uint16_t address)
{
  readTx2(port_num, id, address, 2);
}
uint16_t read2ByteRx2(int port_num)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 2 * sizeof(uint8_t));
  if (packetData[port_num].data_read == NULL)
  {
    printf("[Read2ByteRx] memory allocation failed..\n");
    return 0;
  }
  packetData[port_num].data_read[0] = 0;
  packetData[port_num].data_read[1] = 0;
  readRx2(port_num, 2);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]);
  return 0;
}
uint16_t read2ByteTxRx2(int port_num, uint8_t id, uint16_t address)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 2 * sizeof(uint8_t));
  if (packetData[port_num].data_read == NULL)
  {
    printf("[Read2ByteTxRx] memory allocation failed..\n");
    return 0;
  }
  packetData[port_num].data_read[0] = 0;
  packetData[port_num].data_read[1] = 0;
  readTxRx2(port_num, id, address, 2);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]);
  return 0;
}

void read4ByteTx2(int port_num, uint8_t id, uint16_t address)
{
  readTx2(port_num, id, address, 4);
}
uint32_t read4ByteRx2(int port_num)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 4 * sizeof(uint8_t));
  if (packetData[port_num].data_read == NULL)
  {
    printf("[Read4ByteRx] memory allocation failed..\n");
    return 0;
  }
  packetData[port_num].data_read[0] = 0;
  packetData[port_num].data_read[1] = 0;
  packetData[port_num].data_read[2] = 0;
  packetData[port_num].data_read[3] = 0;
  readRx2(port_num, 4);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEDWORD(DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]), DXL_MAKEWORD(packetData[port_num].data_read[2], packetData[port_num].data_read[3]));
  return 0;
}
uint32_t read4ByteTxRx2(int port_num, uint8_t id, uint16_t address)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 4 * sizeof(uint8_t));
  if (packetData[port_num].data_read == NULL)
  {
    printf("[Read2ByteTxRx] memory allocation failed..\n");
    return 0;
  }
  packetData[port_num].data_read[0] = 0;
  packetData[port_num].data_read[1] = 0;
  packetData[port_num].data_read[2] = 0;
  packetData[port_num].data_read[3] = 0;
  readTxRx2(port_num, id, address, 4);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEDWORD(DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]), DXL_MAKEWORD(packetData[port_num].data_read[2], packetData[port_num].data_read[3]));
  return 0;
}


void writeTxOnly2(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 12);
  if (packetData[port_num].tx_packet == NULL)
  {
    printf("[WriteTxOnly] memory allocation failed..\n");
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = DXL_LOBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_LENGTH_H] = DXL_HIBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = (uint8_t)DXL_LOBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] = (uint8_t)DXL_HIBYTE(address);

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 2 + s] = packetData[port_num].data_write[s];
  }

  txPacket2(port_num);
  g_is_using[port_num] = False;
}

void writeTxRx2(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 12);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 11);
  if (packetData[port_num].tx_packet == NULL || packetData[port_num].rx_packet == NULL)
  {
    printf("[WriteTxRx] memory allocation failed..\n");
    return;
  }
  
  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = DXL_LOBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_LENGTH_H] = DXL_HIBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = (uint8_t)DXL_LOBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] = (uint8_t)DXL_HIBYTE(address);

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 2 + s] = packetData[port_num].data_write[s];
  }

  txRxPacket2(port_num);
}

void write1ByteTxOnly2(int port_num, uint8_t id, uint16_t address, uint8_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 1 * sizeof(uint8_t));
  if (packetData[port_num].data_write == NULL)
  {
    printf("[Write1ByteTxOnly] memory allocation failed..\n");
    return;
  }
  packetData[port_num].data_write[0] = data;
  writeTxOnly2(port_num, id, address, 1);
}
void write1ByteTxRx2(int port_num, uint8_t id, uint16_t address, uint8_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 1 * sizeof(uint8_t));
  if (packetData[port_num].data_write == NULL)
  {
    printf("[Write1ByteTxRx] memory allocation failed..\n");
    return;
  }
  packetData[port_num].data_write[0] = data;
  writeTxRx2(port_num, id, address, 1);
}

void write2ByteTxOnly2(int port_num, uint8_t id, uint16_t address, uint16_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 2 * sizeof(uint8_t));
  if (packetData[port_num].data_write == NULL)
  {
    printf("[Write2ByteTxOnly] memory allocation failed..\n");
    return;
  }
  packetData[port_num].data_write[0] = DXL_LOBYTE(data);
  packetData[port_num].data_write[1] = DXL_HIBYTE(data);
  writeTxOnly2(port_num, id, address, 2);
}
void write2ByteTxRx2(int port_num, uint8_t id, uint16_t address, uint16_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 2 * sizeof(uint8_t));
  if (packetData[port_num].data_write == NULL)
  {
    printf("[Write2ByteTxRx] memory allocation failed..\n");
    return;
  }
  packetData[port_num].data_write[0] = DXL_LOBYTE(data);
  packetData[port_num].data_write[1] = DXL_HIBYTE(data);
  writeTxRx2(port_num, id, address, 2);
}

void write4ByteTxOnly2(int port_num, uint8_t id, uint16_t address, uint32_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 4 * sizeof(uint8_t));
  if (packetData[port_num].data_write == NULL)
  {
    printf("[Write4ByteTxOnly] memory allocation failed..\n");
    return;
  }
  packetData[port_num].data_write[0] = DXL_LOBYTE(DXL_LOWORD(data));
  packetData[port_num].data_write[1] = DXL_HIBYTE(DXL_LOWORD(data));
  packetData[port_num].data_write[2] = DXL_LOBYTE(DXL_HIWORD(data));
  packetData[port_num].data_write[3] = DXL_HIBYTE(DXL_HIWORD(data));
  writeTxOnly2(port_num, id, address, 4);
}
void write4ByteTxRx2(int port_num, uint8_t id, uint16_t address, uint32_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 4 * sizeof(uint8_t));
  if (packetData[port_num].data_write == NULL)
  {
    printf("[Write4ByteTxRx] memory allocation failed..\n");
    return;
  }
  packetData[port_num].data_write[0] = DXL_LOBYTE(DXL_LOWORD(data));
  packetData[port_num].data_write[1] = DXL_HIBYTE(DXL_LOWORD(data));
  packetData[port_num].data_write[2] = DXL_LOBYTE(DXL_HIWORD(data));
  packetData[port_num].data_write[3] = DXL_HIBYTE(DXL_HIWORD(data));
  writeTxRx2(port_num, id, address, 4);
}

void regWriteTxOnly2(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 12);
  if (packetData[port_num].tx_packet == NULL)
  {
    printf("[RegWriteTxOnly] memory allocation failed..\n");
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = DXL_LOBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_LENGTH_H] = DXL_HIBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_REG_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = (uint8_t)DXL_LOBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] = (uint8_t)DXL_HIBYTE(address);

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 2 + s] = packetData[port_num].data_write[s];
  }

  txPacket2(port_num);
  g_is_using[port_num] = False;
}

void regWriteTxRx2(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 12);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 11);
  if (packetData[port_num].tx_packet == NULL || packetData[port_num].rx_packet == NULL)
  {
    printf("[RegWriteTxRx] memory allocation failed..\n");
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = DXL_LOBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_LENGTH_H] = DXL_HIBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_REG_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = (uint8_t)DXL_LOBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] = (uint8_t)DXL_HIBYTE(address);

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 2 + s] = packetData[port_num].data_write[s];
  }

  txRxPacket2(port_num);
}

void syncReadTx2(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, param_length + 14);
  // 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  if (packetData[port_num].tx_packet == NULL)
  {
    printf("[SyncReadTx] memory allocation failed..\n");
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = BROADCAST_ID;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = DXL_LOBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_LENGTH_H] = DXL_HIBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_SYNC_READ;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = DXL_LOBYTE(start_address);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] = DXL_HIBYTE(start_address);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 2] = DXL_LOBYTE(data_length);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 3] = DXL_HIBYTE(data_length);

  for (s = 0; s < param_length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 4 + s] = packetData[port_num].data_write[s];
  }

  txPacket2(port_num);

  if (packetData[port_num].communication_result == COMM_SUCCESS)
    setPacketTimeout(port_num, (uint16_t)((11 + data_length) * param_length));
}

void syncWriteTxOnly2(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, param_length + 14);
  // 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  if (packetData[port_num].tx_packet == NULL)
  {
    printf("[SyncWriteTxOnly] memory allocation failed..\n");
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = BROADCAST_ID;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = DXL_LOBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_LENGTH_H] = DXL_HIBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_SYNC_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = DXL_LOBYTE(start_address);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] = DXL_HIBYTE(start_address);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 2] = DXL_LOBYTE(data_length);
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 3] = DXL_HIBYTE(data_length);

  for (s = 0; s < param_length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 4 + s] = packetData[port_num].data_write[s];
  }

  txRxPacket2(port_num);
}

void bulkReadTx2(int port_num, uint16_t param_length)
{
  uint16_t s;
  uint16_t i;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, param_length + 10);
  // 10: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST CRC16_L CRC16_H
  if (packetData[port_num].tx_packet == NULL)
  {
    printf("[BulkReadTx] memory allocation failed..\n");
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = BROADCAST_ID;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = DXL_LOBYTE(param_length + 3); // 3: INST CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_LENGTH_H] = DXL_HIBYTE(param_length + 3); // 3: INST CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_BULK_READ;

  for (s = 0; s < param_length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + s] = packetData[port_num].data_write[s];
  }

  txPacket2(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
  {
    int wait_length = 0;
    for (i = 0; i < param_length; i += 5)
    {
      wait_length += DXL_MAKEWORD(packetData[port_num].data_write[i + 3], packetData[port_num].data_write[i + 4]) + 10;
    }
    setPacketTimeout(port_num, (uint16_t)wait_length);
  }
}

void bulkWriteTxOnly2(int port_num, uint16_t param_length)
{
  uint16_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, param_length + 10);
  // 10: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST CRC16_L CRC16_H
  if (packetData[port_num].tx_packet == NULL)
  {
    printf("[BulkWriteTxOnly] memory allocation failed..\n");
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = BROADCAST_ID;
  packetData[port_num].tx_packet[PKT_LENGTH_L] = DXL_LOBYTE(param_length + 3); // 3: INST CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_LENGTH_H] = DXL_HIBYTE(param_length + 3); // 3: INST CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_BULK_WRITE;

  for (s = 0; s < param_length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + s] = packetData[port_num].data_write[s];
  }

  txRxPacket2(port_num);
}
