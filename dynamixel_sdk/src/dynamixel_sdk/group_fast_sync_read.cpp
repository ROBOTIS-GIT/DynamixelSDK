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

/* Author: Honghyun Kim */

#include <algorithm>

#if defined(__linux__)
#include "group_fast_sync_read.h"
#elif defined(__APPLE__)
#include "group_fast_sync_read.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "group_fast_sync_read.h"
#elif defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__)
#include "../../include/dynamixel_sdk/group_fast_sync_read.h"
#endif

const int RXPACKET_MAX_LEN = 1024;
const int PKT_ID = 4;
const int PKT_PARAMETER0 = 8;

using namespace dynamixel;

GroupFastSyncRead::GroupFastSyncRead(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length)
  : GroupSyncRead(port, ph, start_address, data_length)
{
    clearParam();
}

int GroupFastSyncRead::txPacket()
{
    if ((1.0 == ph_->getProtocolVersion()) || (id_list_.empty()))
        return COMM_NOT_AVAILABLE;

    if ((true == is_param_changed_) || (0 == param_))
        makeParam();

    return ph_->fastSyncReadTx(port_, start_address_, data_length_, param_, (uint16_t)id_list_.size() * 1);
}

int GroupFastSyncRead::rxPacket()
{
    last_result_ = false;

    if ((1.0 == ph_->getProtocolVersion()) || (id_list_.empty()))
        return COMM_NOT_AVAILABLE;

    int count = id_list_.size();
    int result = COMM_RX_FAIL;
    uint8_t *rxpacket = (uint8_t *)malloc(RXPACKET_MAX_LEN);
    if (NULL == rxpacket)
        return result;

    do {
        result = ph_->rxPacket(port_, rxpacket, true);
    } while ((COMM_SUCCESS == result) && (BROADCAST_ID != rxpacket[PKT_ID]));

    if ((COMM_SUCCESS == result) && (BROADCAST_ID == rxpacket[PKT_ID])) {
        int index = PKT_PARAMETER0;
        for (int i = 0; i < count; ++i) {
            uint8_t id = id_list_[i];
            *error_list_[id] = (uint8_t)rxpacket[index];
            for (uint16_t s = 0; s < data_length_; s++) {
                data_list_[id][s] = rxpacket[index + 2 + s];
            }
            index += (data_length_ + 4);
        }
        last_result_ = true;
    }

    free(rxpacket);
    return result;
}

int GroupFastSyncRead::txRxPacket()
{
    if (1.0 == ph_->getProtocolVersion())
        return COMM_NOT_AVAILABLE;

    int result = txPacket();
    if (COMM_SUCCESS != result)
        return result;
    return rxPacket();
}
