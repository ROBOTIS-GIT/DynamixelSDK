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

/* Author: zerom, Ryu Woon Jung (Leon) */

#include <algorithm>

#if defined(__linux__)
#include "group_sync_write.h"
#elif defined(__APPLE__)
#include "group_sync_write.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "group_sync_write.h"
#elif defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__) || defined(ARDUINO_OpenRB)
#include "../../include/dynamixel_sdk/group_sync_write.h"
#endif

using namespace dynamixel;

GroupSyncWrite::GroupSyncWrite(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length)
  : GroupHandler(port, ph),
    start_address_(start_address),
    data_length_(data_length)
{
  clearParam();
}

void GroupSyncWrite::makeParam()
{
  if (id_list_.empty()) return;

  param_.clear();
  param_.reserve(id_list_.size() * (1 + data_length_)); // ID(1) + DATA(data_length)

  for (unsigned int i = 0; i < id_list_.size(); i++)
  {
    uint8_t id = id_list_[i];
    if (data_list_[id].empty())
      return;

    param_.push_back(id);
    for (int c = 0; c < data_length_; c++)
      param_.push_back((data_list_[id])[c]);
  }
}

bool GroupSyncWrite::addParam(uint8_t id, uint8_t *data)
{
  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
    return false;

  id_list_.push_back(id);
  data_list_[id].assign(data, data + data_length_);

  is_param_changed_   = true;
  return true;
}

void GroupSyncWrite::removeParam(uint8_t id)
{
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return;

  id_list_.erase(it);
  data_list_.erase(id);

  is_param_changed_   = true;
}

bool GroupSyncWrite::changeParam(uint8_t id, uint8_t *data)
{
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return false;

  data_list_[id].assign(data, data + data_length_);

  is_param_changed_   = true;
  return true;
}

void GroupSyncWrite::clearParam()
{
  if (id_list_.empty())
    return;

  id_list_.clear();
  data_list_.clear();
  param_.clear();
}

int GroupSyncWrite::txPacket()
{
  if (id_list_.empty())
    return COMM_NOT_AVAILABLE;

  if (is_param_changed_ == true || param_.empty())
    makeParam();

  return ph_->syncWriteTxOnly(port_, start_address_, data_length_, param_.data(), id_list_.size() * (1 + data_length_));
}
