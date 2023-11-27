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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPBULKREAD_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPBULKREAD_C_H_


#include "robotis_def.h"
#include "port_handler.h"
#include "packet_handler.h"

WINDECLSPEC int         groupBulkRead               (int port_num, int protocol_version);

WINDECLSPEC uint8_t     groupBulkReadAddParam       (int group_num, uint8_t id, uint16_t start_address, uint16_t data_length);
WINDECLSPEC void        groupBulkReadRemoveParam    (int group_num, uint8_t id);
WINDECLSPEC void        groupBulkReadClearParam     (int group_num);

WINDECLSPEC void        groupBulkReadTxPacket       (int group_num);
WINDECLSPEC void        groupBulkReadRxPacket       (int group_num);
WINDECLSPEC void        groupBulkReadTxRxPacket     (int group_num);

WINDECLSPEC uint8_t     groupBulkReadIsAvailable    (int group_num, uint8_t id, uint16_t address, uint16_t data_length);
WINDECLSPEC uint32_t    groupBulkReadGetData        (int group_num, uint8_t id, uint16_t address, uint16_t data_length);

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPBULKREAD_C_H_ */
