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

#if defined(__linux__)
#include "group_sync_write.h"
#elif defined(__APPLE__)
#include "group_sync_write.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "group_sync_write.h"
#endif

typedef struct
{
  uint8_t     id;
  uint16_t    data_end;
  uint8_t     *data;
}DataList;

typedef struct
{
  int         port_num;
  int         protocol_version;

  int         data_list_length;

  uint8_t     is_param_changed;

  uint16_t    start_address;
  uint16_t    data_length;

  DataList   *data_list;
}GroupData;

static GroupData *groupData;
static int g_used_group_num = 0;

int size(int group_num)
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

int find(int group_num, int id)
{
  int data_num;

  for (data_num = 0; data_num < groupData[group_num].data_list_length; data_num++)
  {
    if (groupData[group_num].data_list[data_num].id == id)
      break;
  }

  return data_num;
}

int groupSyncWrite(int port_num, int protocol_version, uint16_t start_address, uint16_t data_length)
{
  int group_num = 0;

  if (g_used_group_num != 0)
  {
    for (group_num = 0; group_num < g_used_group_num; group_num++)
    {
      if (groupData[group_num].is_param_changed != True
          && groupData[group_num].port_num == port_num
          && groupData[group_num].protocol_version == protocol_version
          && groupData[group_num].start_address == start_address
          && groupData[group_num].data_length == data_length)
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
  groupData[group_num].is_param_changed = False;
  groupData[group_num].start_address = start_address;
  groupData[group_num].data_length = data_length;
  groupData[group_num].data_list = 0;

  groupSyncWriteClearParam(group_num);

  return group_num;
}

void groupSyncWriteMakeParam(int group_num)
{
  int data_num, c, idx;
  int port_num = groupData[group_num].port_num;

  if (size(group_num) == 0)
    return;

  packetData[port_num].data_write = (uint8_t*)realloc(packetData[port_num].data_write, size(group_num) * (1 + groupData[group_num].data_length) * sizeof(uint8_t)); // ID(1) + DATA(data_length)

  idx = 0;
  for (data_num = 0; data_num < groupData[group_num].data_list_length; data_num++)
  {
    if (groupData[group_num].data_list[data_num].id == NOT_USED_ID)
      continue;

    packetData[port_num].data_write[idx++] = groupData[group_num].data_list[data_num].id;
    for (c = 0; c < groupData[group_num].data_length; c++)
    {
      packetData[port_num].data_write[idx++] = groupData[group_num].data_list[data_num].data[c];
    }
  }
}

uint8_t groupSyncWriteAddParam(int group_num, uint8_t id, uint32_t data, uint16_t input_length)
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
    groupData[group_num].data_list[data_num].data = (uint8_t *)calloc(groupData[group_num].data_length, sizeof(uint8_t));
    groupData[group_num].data_list[data_num].data_end = 0;
  }
  else
  {
    if (groupData[group_num].data_list[data_num].data_end + input_length > groupData[group_num].data_length)
      return False;
  }

  switch (input_length)
  {
    case 1:
      groupData[group_num].data_list[data_num].data[groupData[group_num].data_list[data_num].data_end + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      break;

    case 2:
      groupData[group_num].data_list[data_num].data[groupData[group_num].data_list[data_num].data_end + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      groupData[group_num].data_list[data_num].data[groupData[group_num].data_list[data_num].data_end + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      break;

    case 4:
      groupData[group_num].data_list[data_num].data[groupData[group_num].data_list[data_num].data_end + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      groupData[group_num].data_list[data_num].data[groupData[group_num].data_list[data_num].data_end + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      groupData[group_num].data_list[data_num].data[groupData[group_num].data_list[data_num].data_end + 2] = DXL_LOBYTE(DXL_HIWORD(data));
      groupData[group_num].data_list[data_num].data[groupData[group_num].data_list[data_num].data_end + 3] = DXL_HIBYTE(DXL_HIWORD(data));
      break;

    default:
      return False;
  }
  groupData[group_num].data_list[data_num].data_end = input_length;

  groupData[group_num].is_param_changed = True;
  return True;
}

void groupSyncWriteRemoveParam(int group_num, uint8_t id)
{
  int data_num = find(group_num, id);

  if (data_num == groupData[group_num].data_list_length)
    return;

  if (groupData[group_num].data_list[data_num].id == NOT_USED_ID)  // NOT exist
    return;

  groupData[group_num].data_list[data_num].data_end = 0;

  free(groupData[group_num].data_list[data_num].data);
  groupData[group_num].data_list[data_num].data = 0;

  groupData[group_num].data_list[data_num].id = NOT_USED_ID;

  groupData[group_num].is_param_changed = True;
}

uint8_t groupSyncWriteChangeParam(int group_num, uint8_t id, uint32_t data, uint16_t input_length, uint16_t data_pos)
{
  int data_num = 0;
  if (id == NOT_USED_ID)  // NOT exist
    return False;

  data_num = find(group_num, id);

  if (data_num == groupData[group_num].data_list_length)
    return False;

  if (data_pos + input_length > groupData[group_num].data_length)
    return False;

  switch (input_length)
  {
    case 1:
      groupData[group_num].data_list[data_num].data[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      break;

    case 2:
      groupData[group_num].data_list[data_num].data[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      groupData[group_num].data_list[data_num].data[data_pos + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      break;

    case 4:
      groupData[group_num].data_list[data_num].data[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      groupData[group_num].data_list[data_num].data[data_pos + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      groupData[group_num].data_list[data_num].data[data_pos + 2] = DXL_LOBYTE(DXL_HIWORD(data));
      groupData[group_num].data_list[data_num].data[data_pos + 3] = DXL_HIBYTE(DXL_HIWORD(data));
      break;

    default:
      return False;
  }

  groupData[group_num].is_param_changed = True;
  return True;
}

void groupSyncWriteClearParam(int group_num)
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

void groupSyncWriteTxPacket(int group_num)
{
  int port_num = groupData[group_num].port_num;

  if (size(group_num) == 0)
  {
  	packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
  	return;
  }

  if (groupData[group_num].is_param_changed == True)
    groupSyncWriteMakeParam(group_num);

	syncWriteTxOnly(
    groupData[group_num].port_num
    , groupData[group_num].protocol_version
    , groupData[group_num].start_address
    , groupData[group_num].data_length
    , size(group_num) * (1 + groupData[group_num].data_length));
}
