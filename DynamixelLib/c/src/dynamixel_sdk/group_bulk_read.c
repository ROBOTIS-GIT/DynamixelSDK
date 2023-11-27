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

#include <stdio.h>
#include <stdlib.h>

#if defined(__linux__)
#include "group_bulk_read.h"
#elif defined(__APPLE__)
#include "group_bulk_read.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "group_bulk_read.h"
#endif

typedef struct
{
  uint8_t     id;
  uint16_t    start_address;
  uint16_t    data_length;
  uint8_t     *data;
}DataList;

typedef struct
{
  int         port_num;
  int         protocol_version;

  int         data_list_length;

  uint8_t     last_result;
  uint8_t     is_param_changed;

  DataList   *data_list;
}GroupData;

static GroupData *groupData;
static int g_used_group_num = 0;

static int size(int group_num)
{
  int data_num;
  int real_size = 0;

  for (data_num = 0; data_num < groupData[group_num].data_list_length; data_num++)
  {
    if (groupData[group_num].data_list[data_num].id != NOT_USED_ID)
      real_size++;
  }
  return real_size;
}

static int find(int group_num, int id)
{
  int data_num;

  for (data_num = 0; data_num < groupData[group_num].data_list_length; data_num++)
  {
    if (groupData[group_num].data_list[data_num].id == id)
      break;
  }

  return data_num;
}

int groupBulkRead(int port_num, int protocol_version)
{
  int group_num = 0;

  if (g_used_group_num != 0)
  {
    for (group_num = 0; group_num < g_used_group_num; group_num++)
    {
      if (groupData[group_num].is_param_changed != True
          && groupData[group_num].port_num == port_num
          && groupData[group_num].protocol_version == protocol_version)
        break;
    }
  }

  if (group_num == g_used_group_num)
  {
    g_used_group_num++;
    groupData = (GroupData *)realloc(groupData, g_used_group_num * sizeof(GroupData));
  }

  groupData[group_num].port_num = port_num;
  groupData[group_num].protocol_version = protocol_version;
  groupData[group_num].data_list_length = 0;
  groupData[group_num].last_result = False;
  groupData[group_num].is_param_changed = False;
  groupData[group_num].data_list = 0;

  groupBulkReadClearParam(group_num);

  return group_num;
}

void groupBulkReadMakeParam(int group_num)
{
  int data_num, idx;
  int port_num = groupData[group_num].port_num;

  if (size(group_num) == 0)
    return;

  if (groupData[group_num].protocol_version == 1)
  {
    packetData[port_num].data_write = (uint8_t*)realloc(packetData[port_num].data_write, size(group_num) * sizeof(uint8_t) * 3); // ID(1) + ADDR(1) + LENGTH(1)
  }
  else    // 2.0
  {
    packetData[port_num].data_write = (uint8_t*)realloc(packetData[port_num].data_write, size(group_num) * sizeof(uint8_t) * 5); // ID(1) + ADDR(2) + LENGTH(2)
  }

  idx = 0;
  for (data_num = 0; data_num < groupData[group_num].data_list_length; data_num++)
  {
    if (groupData[group_num].data_list[data_num].id == NOT_USED_ID)
      continue;

    if (groupData[group_num].protocol_version == 1)
    {
      packetData[port_num].data_write[idx++] = (uint8_t)groupData[group_num].data_list[data_num].data_length;       // LEN
      packetData[port_num].data_write[idx++] = groupData[group_num].data_list[data_num].id;                         // ID
      packetData[port_num].data_write[idx++] = (uint8_t)groupData[group_num].data_list[data_num].start_address;     // ADDR
    }
    else    // 2.0
    {
      packetData[port_num].data_write[idx++] = groupData[group_num].data_list[data_num].id;                         // ID
      packetData[port_num].data_write[idx++] = DXL_LOBYTE(groupData[group_num].data_list[data_num].start_address);  // ADDR_L
      packetData[port_num].data_write[idx++] = DXL_HIBYTE(groupData[group_num].data_list[data_num].start_address);  // ADDR_H
      packetData[port_num].data_write[idx++] = DXL_LOBYTE(groupData[group_num].data_list[data_num].data_length);    // LEN_L
      packetData[port_num].data_write[idx++] = DXL_HIBYTE(groupData[group_num].data_list[data_num].data_length);    // LEN_H
    }
  }
}

uint8_t groupBulkReadAddParam(int group_num, uint8_t id, uint16_t start_address, uint16_t data_length)
{
  int data_num = 0;

  if (id == NOT_USED_ID)
    return False;

  if (groupData[group_num].data_list_length != 0)
    data_num = find(group_num, id);

  if (groupData[group_num].data_list_length == data_num)
  {
    groupData[group_num].data_list_length++;
    groupData[group_num].data_list = (DataList *)realloc(groupData[group_num].data_list, groupData[group_num].data_list_length * sizeof(DataList));

    groupData[group_num].data_list[data_num].id = id;
    groupData[group_num].data_list[data_num].data_length = data_length;
    groupData[group_num].data_list[data_num].start_address = start_address;
    groupData[group_num].data_list[data_num].data = (uint8_t *)calloc(groupData[group_num].data_list[data_num].data_length, sizeof(uint8_t));
  }

  groupData[group_num].is_param_changed = True;
  return True;
}

void groupBulkReadRemoveParam(int group_num, uint8_t id)
{
  int data_num = find(group_num, id);

  if (groupData[group_num].data_list[data_num].id == NOT_USED_ID)  // NOT exist
    return;

  free(groupData[group_num].data_list[data_num].data);
  groupData[group_num].data_list[data_num].data = 0;

  groupData[group_num].data_list[data_num].id = NOT_USED_ID;
  groupData[group_num].data_list[data_num].data_length = 0;
  groupData[group_num].data_list[data_num].start_address = 0;

  groupData[group_num].is_param_changed = True;
}

void groupBulkReadClearParam(int group_num)
{
  int data_num = 0;
  int port_num = groupData[group_num].port_num;

  if (size(group_num) == 0)
    return;

  for (data_num = 0; data_num < groupData[group_num].data_list_length; data_num++)
  {
    free(groupData[group_num].data_list[data_num].data);
    groupData[group_num].data_list[data_num].data = 0;
  }

  free(groupData[group_num].data_list);
  groupData[group_num].data_list = 0;

  free(packetData[port_num].data_write);
  packetData[port_num].data_write = 0;

  groupData[group_num].data_list_length = 0;

  groupData[group_num].is_param_changed = False;
}

void groupBulkReadTxPacket(int group_num)
{
  int port_num = groupData[group_num].port_num;

  if (size(group_num) == 0)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  if (groupData[group_num].is_param_changed == True)
    groupBulkReadMakeParam(group_num);

  if (groupData[group_num].protocol_version == 1)
  {
    bulkReadTx(groupData[group_num].port_num, groupData[group_num].protocol_version, size(group_num) * 3);
  }
  else
  {
    bulkReadTx(groupData[group_num].port_num, groupData[group_num].protocol_version, size(group_num) * 5);
  }
}

void groupBulkReadRxPacket(int group_num)
{
  int data_num, c;
  int port_num = groupData[group_num].port_num;

  packetData[port_num].communication_result = COMM_RX_FAIL;

  groupData[group_num].last_result = False;

  if (size(group_num) == 0)
  {
    packetData[groupData[group_num].port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  for (data_num = 0; data_num < groupData[group_num].data_list_length; data_num++)
  {
    if (groupData[group_num].data_list[data_num].id == NOT_USED_ID)
      continue;

    packetData[port_num].data_read
      = (uint8_t *)realloc(packetData[port_num].data_read, groupData[group_num].data_list[data_num].data_length * sizeof(uint8_t));

    readRx(groupData[group_num].port_num, groupData[group_num].protocol_version, groupData[group_num].data_list[data_num].data_length);
    if (packetData[groupData[group_num].port_num].communication_result != COMM_SUCCESS)
      return;

    for (c = 0; c < groupData[group_num].data_list[data_num].data_length; c++)
    {
      groupData[group_num].data_list[data_num].data[c] = packetData[port_num].data_read[c];
    }
  }

  if (packetData[port_num].communication_result == COMM_SUCCESS)
    groupData[group_num].last_result = True;
}

void groupBulkReadTxRxPacket(int group_num)
{
  int port_num = groupData[group_num].port_num;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  groupBulkReadTxPacket(group_num);
  if (packetData[port_num].communication_result != COMM_SUCCESS)
    return;

  groupBulkReadRxPacket(group_num);
}

uint8_t groupBulkReadIsAvailable(int group_num, uint8_t id, uint16_t address, uint16_t data_length)
{
  int data_num = find(group_num, id);
  uint16_t start_addr;

  if (groupData[group_num].last_result == False || groupData[group_num].data_list[data_num].id == NOT_USED_ID)
    return False;

  start_addr = groupData[group_num].data_list[data_num].start_address;

  if (address < start_addr || start_addr + groupData[group_num].data_list[data_num].data_length - data_length < address)
    return False;

  return True;
}

uint32_t groupBulkReadGetData(int group_num, uint8_t id, uint16_t address, uint16_t data_length)
{
  int data_num = find(group_num, id);
  uint16_t start_addr = groupData[group_num].data_list[data_num].start_address;

  if (groupBulkReadIsAvailable(group_num, id, address, data_length) == False)
    return 0;

  switch (data_length)
  {
    case 1:
      return groupData[group_num].data_list[data_num].data[address - start_addr];

    case 2:
      return DXL_MAKEWORD(groupData[group_num].data_list[data_num].data[address - start_addr], groupData[group_num].data_list[data_num].data[address - start_addr + 1]);

    case 4:
      return DXL_MAKEDWORD(DXL_MAKEWORD(groupData[group_num].data_list[data_num].data[address - start_addr + 0], groupData[group_num].data_list[data_num].data[address - start_addr + 1]),
        DXL_MAKEWORD(groupData[group_num].data_list[data_num].data[address - start_addr + 2], groupData[group_num].data_list[data_num].data[address - start_addr + 3]));

    default:
      return 0;
  }
}
