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

////////////////////////////////////////////////////////////////////////////////
/// @file The file for Dynamixel Fast Sync Read
/// @author Honghyun Kim
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPFASTSYNCREAD_H
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPFASTSYNCREAD_H


#include "group_sync_read.h"

namespace dynamixel
{

class WINDECLSPEC GroupFastSyncRead : public GroupSyncRead
{
public:
    GroupFastSyncRead(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length);
    ~GroupFastSyncRead() { clearParam(); }

    int txPacket();
    int rxPacket();
    int txRxPacket();
};

}


#endif // DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPFASTSYNCREAD_H
