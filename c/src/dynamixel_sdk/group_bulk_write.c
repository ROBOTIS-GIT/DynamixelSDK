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
#include "group_bulk_write.h"
#elif defined(__APPLE__)
#include "group_bulk_write.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "group_bulk_write.h"
#endif

typedef struct
{
  uint8_t     id;
  uint16_t    data_end;
  uint16_t    start_address;
  uint16_t    data_length;
  uint8_t     data[DXL_MAX_NODE_BUFFER_SIZE];
}DataList;

typedef struct
{
  int         port_num;
  int         protocol_version;

  int         data_list_length;

  uint8_t     is_param_changed;

  uint16_t    param_length;

  DataList    data_list[DXL_MAX_NODES];
}GroupData;

static GroupData groupData[DXL_MAX_GROUPS];
static int g_used_group_num = 0;

static int size(int group_num)
{
  return groupData[group_num].data_list_length;
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

int groupBulkWrite(int port_num, int protocol_version)
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
    if (g_used_group_num < DXL_MAX_GROUPS)
    {
      g_used_group_num++;
    }
    else
    {
      return -1;
    }
  }

  groupData[group_num].port_num = port_num;
  groupData[group_num].protocol_version = protocol_version;
  groupData[group_num].data_list_length = 0;
  groupData[group_num].is_param_changed = False;
  groupData[group_num].param_length = 0;
  // groupData[group_num].data_list = 0; // Removed

  groupBulkWriteClearParam(group_num);

  return group_num;
}

void groupBulkWriteMakeParam(int group_num)
{
  int data_num, idx, c;
  int port_num = groupData[group_num].port_num;

  if (groupData[group_num].protocol_version == 1)
    return;

  if (size(group_num) == 0)
    return;

  groupData[group_num].param_length = 0;

  idx = 0;
  for (data_num = 0; data_num < groupData[group_num].data_list_length; data_num++)
  {
    if (groupData[group_num].data_list[data_num].id == NOT_USED_ID)
      continue;

    groupData[group_num].param_length += 1 + 2 + 2 + groupData[group_num].data_list[data_num].data_length;

    // [Optimization] Removed realloc. Use pre-allocated buffer.
    // packetData[port_num].data_write = (uint8_t*)realloc(packetData[port_num].data_write, groupData[group_num].param_length * sizeof(uint8_t));
    if (idx + 5 + groupData[group_num].data_list[data_num].data_length > DXL_MAX_BUFFER_LEN) break;

    packetData[port_num].data_write[idx++] = groupData[group_num].data_list[data_num].id;
    packetData[port_num].data_write[idx++] = DXL_LOBYTE(groupData[group_num].data_list[data_num].start_address);
    packetData[port_num].data_write[idx++] = DXL_HIBYTE(groupData[group_num].data_list[data_num].start_address);
    packetData[port_num].data_write[idx++] = DXL_LOBYTE(groupData[group_num].data_list[data_num].data_length);
    packetData[port_num].data_write[idx++] = DXL_HIBYTE(groupData[group_num].data_list[data_num].data_length);

    for (c = 0; c < groupData[group_num].data_list[data_num].data_length; c++)
    {
      if (c < DXL_MAX_NODE_BUFFER_SIZE)
        packetData[port_num].data_write[idx++] = groupData[group_num].data_list[data_num].data[c];
    }
  }
}

uint8_t groupBulkWriteAddParam(int group_num, uint8_t id, uint16_t start_address, uint16_t data_length, uint32_t data, uint16_t input_length)
{
  int data_num = 0;

  if (groupData[group_num].protocol_version == 1)
    return False;

  if (id == NOT_USED_ID)
    return False;

  data_num = find(group_num, id);

  if (data_num == groupData[group_num].data_list_length)
  {
    if (groupData[group_num].data_list_length < DXL_MAX_NODES)
      groupData[group_num].data_list_length++;
    else
      return False;

    groupData[group_num].data_list[data_num].id = id;
    groupData[group_num].data_list[data_num].data_length = data_length;
    groupData[group_num].data_list[data_num].start_address = start_address;
    groupData[group_num].data_list[data_num].data_end = 0;
  }
  else
  {
    if (groupData[group_num].data_list[data_num].data_end + input_length > groupData[group_num].data_list[data_num].data_length)
      return False;
  }

  if (groupData[group_num].data_list[data_num].data_end + input_length > DXL_MAX_NODE_BUFFER_SIZE)
      return False;

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
  groupData[group_num].data_list[data_num].data_end += input_length;

  groupData[group_num].is_param_changed = True;
  return True;
}
void groupBulkWriteRemoveParam(int group_num, uint8_t id)
{
  int data_num = find(group_num, id);

  if (groupData[group_num].protocol_version == 1)
    return;

  if (data_num == groupData[group_num].data_list_length)
    return;

  if (groupData[group_num].data_list[data_num].id == NOT_USED_ID)  // NOT exist
    return;

  groupData[group_num].data_list[data_num].data_end = 0;

  groupData[group_num].data_list[data_num].data_length = 0;
  groupData[group_num].data_list[data_num].start_address = 0;
  groupData[group_num].data_list[data_num].id = NOT_USED_ID;

  // Shift
  for (; data_num < groupData[group_num].data_list_length - 1; data_num++)
  {
    groupData[group_num].data_list[data_num] = groupData[group_num].data_list[data_num + 1];
  }

  if (groupData[group_num].data_list_length > 0)
    groupData[group_num].data_list_length--;

  groupData[group_num].is_param_changed = True;
}

void groupBulkWriteClearParam(int group_num)
{
  int data_num = 0;

  if (groupData[group_num].protocol_version == 1)
    return;

  if (size(group_num) == 0)
    return;

  for (data_num = 0; data_num < groupData[group_num].data_list_length; data_num++)
  {
    groupData[group_num].data_list[data_num].id = NOT_USED_ID;
    groupData[group_num].data_list[data_num].data_length = 0;
    groupData[group_num].data_list[data_num].start_address = 0;
    groupData[group_num].data_list[data_num].data_end = 0;
  }

  groupData[group_num].data_list_length = 0;

  groupData[group_num].is_param_changed = False;
}
void groupBulkWriteTxPacket(int group_num)
{
  if (groupData[group_num].protocol_version == 1)
  {
    packetData[groupData[group_num].port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  if (size(group_num) == 0)
  {
    packetData[groupData[group_num].port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  if (groupData[group_num].is_param_changed == True)
    groupBulkWriteMakeParam(group_num);

  bulkWriteTxOnly(groupData[group_num].port_num, groupData[group_num].protocol_version, groupData[group_num].param_length);
}
