// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Hyungyu Kim

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_CONNECTOR_HPP_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_CONNECTOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_easy_sdk/dynamixel_error.hpp"
#include "dynamixel_easy_sdk/motor.hpp"
#include "dynamixel_easy_sdk/smart_group.hpp"

namespace dynamixel
{
class GroupExecutor;

class Connector
{
public:
  Connector(const std::string & port_name, int baud_rate);

  virtual ~Connector();

  std::unique_ptr<Motor> createMotor(uint8_t id);
  std::vector<std::unique_ptr<Motor>> createAllMotors(int start_id = 0, int end_id = 252);

  // Legacy Support
  std::unique_ptr<GroupExecutor> createGroupExecutor();
  
  // Smart Factory Methods
  std::unique_ptr<SmartGroupReader> createReader() {
      std::lock_guard<std::mutex> lock(mutex_);
      return std::make_unique<SmartGroupReader>(port_handler_.get(), packet_handler_);
  }

  std::unique_ptr<SmartGroupReader> createReader(const std::vector<uint8_t>& ids, const std::string& item_name, const std::string& key = "") {
      std::lock_guard<std::mutex> lock(mutex_);
      auto reader = std::make_unique<SmartGroupReader>(port_handler_.get(), packet_handler_);
      reader->add(ids, item_name, key.empty() ? item_name : key);
      return reader;
  }

  // Complex Overload
  std::unique_ptr<SmartGroupReader> createReader(const std::vector<std::pair<uint8_t, std::vector<std::string>>>& requests) {
      std::lock_guard<std::mutex> lock(mutex_);
      auto reader = std::make_unique<SmartGroupReader>(port_handler_.get(), packet_handler_);
      for (const auto& req : requests) {
          uint8_t id = req.first;
          for (const auto& item : req.second) {
              reader->add(id, item, item); // Use item name as key
          }
      }
      return reader;
  }

  std::unique_ptr<SmartGroupWriter> createWriter() {
      std::lock_guard<std::mutex> lock(mutex_);
      return std::make_unique<SmartGroupWriter>(port_handler_.get(), packet_handler_);
  }

  std::unique_ptr<SmartGroupWriter> createWriter(const std::vector<uint8_t>& ids, const std::string& item_name) {
      std::lock_guard<std::mutex> lock(mutex_);
      auto writer = std::make_unique<SmartGroupWriter>(port_handler_.get(), packet_handler_);
      writer->add(ids, item_name, 0);
      return writer;
  }

  Result<void, DxlError> write1ByteData(uint8_t id, uint16_t address, uint8_t value);
  Result<void, DxlError> write2ByteData(uint8_t id, uint16_t address, uint16_t value);
  Result<void, DxlError> write4ByteData(uint8_t id, uint16_t address, uint32_t value);
  Result<uint8_t, DxlError> read1ByteData(uint8_t id, uint16_t address);
  Result<uint16_t, DxlError> read2ByteData(uint8_t id, uint16_t address);
  Result<uint32_t, DxlError> read4ByteData(uint8_t id, uint16_t address);

  Result<void, DxlError> reboot(uint8_t id);
  Result<uint16_t, DxlError> ping(uint8_t id);
  Result<std::vector<uint8_t>, DxlError> broadcastPing();
  Result<void, DxlError> factoryReset(uint8_t id, uint8_t option);
  void closePort() {port_handler_->closePort();}

  PortHandler * getPortHandler() const {return port_handler_.get();}
  PacketHandler * getPacketHandler() const {return packet_handler_;}

private:
  std::unique_ptr<PortHandler> port_handler_;
  PacketHandler * packet_handler_;
  mutable std::mutex mutex_;
};
}  // namespace dynamixel
#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_CONNECTOR_HPP_ */
