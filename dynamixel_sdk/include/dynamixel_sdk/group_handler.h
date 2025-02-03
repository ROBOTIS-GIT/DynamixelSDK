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
/// @file The file for Group Handler
/// @author Honghyun Kim
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPHANDLER_H
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPHANDLER_H


#include <map>
#include <vector>
#include "port_handler.h"
#include "packet_handler.h"

namespace dynamixel
{

class WINDECLSPEC GroupHandler
{
public:
    GroupHandler(PortHandler *port, PacketHandler *ph);

    PortHandler *getPortHandler() { return port_; }
    PacketHandler *getPacketHandler() { return ph_; }

protected:
    PortHandler *port_;
    PacketHandler *ph_;

    std::vector<uint8_t> id_list_;
    std::map<uint8_t, uint8_t *> data_list_;     // <id, data>

    bool is_param_changed_;

    uint8_t *param_;
};

}


#endif // DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPHANDLER_H
