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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCWRITE_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCWRITE_C_H_

#include "robotis_def.h"
#include "port_handler.h"
#include "packet_handler.h"

WINDECLSPEC int     groupSyncWrite              (int port_num, int protocol_version, uint16_t start_address, uint16_t data_length);

WINDECLSPEC uint8_t groupSyncWriteAddParam      (int group_num, uint8_t id, uint32_t data, uint16_t data_length);
WINDECLSPEC void    groupSyncWriteRemoveParam   (int group_num, uint8_t id);
WINDECLSPEC uint8_t groupSyncWriteChangeParam   (int group_num, uint8_t id, uint32_t data, uint16_t data_length, uint16_t data_pos);
WINDECLSPEC void    groupSyncWriteClearParam    (int group_num);

WINDECLSPEC void    groupSyncWriteTxPacket      (int group_num);


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCWRITE_C_H_ */
