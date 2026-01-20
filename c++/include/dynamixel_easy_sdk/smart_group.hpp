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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_SMART_GROUP_HPP_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_SMART_GROUP_HPP_

#include <vector>
#include <string>
#include <tuple>
#include <memory>
#include <algorithm>
#include <mutex>
#include <optional>
#include <map>
#include <stdexcept>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_easy_sdk/group_strategy.hpp"
#include "dynamixel_easy_sdk/control_table.hpp"
#include "dynamixel_easy_sdk/dynamixel_error.hpp"

namespace dynamixel {

class SmartGroupReader {
public:
    SmartGroupReader(PortHandler* port, PacketHandler* ph)
        : port_(port), ph_(ph) {}

    // Fluent Interface
    SmartGroupReader& add(uint8_t id, uint16_t addr, uint16_t len, const std::string& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        checkDuplicateKey(id, key);
        requests_.emplace_back(id, addr, len, key);
        return *this;
    }

    // Overload for using Item Name directly (looks up ControlTable)
    SmartGroupReader& add(uint8_t id, const std::string& item_name, const std::string& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        checkDuplicateKey(id, key);

        uint16_t model_number = getModelNumber(id);
        if (model_number == 0) {
            // Failed to get model number (Ping failed)
            return *this;
        }

        auto item = ControlTable::getItemInfo(model_number, item_name);
        if (item) {
            requests_.emplace_back(id, item->address, item->size, key);
        } else {
            throw DxlRuntimeError("[SmartGroup] Item not found in Control Table: " + item_name);
        }
        return *this; 
    }
    
    // Template version for type safety (mostly for the key/return type, but here we just store raw request)
    template <typename T>
    SmartGroupReader& add(uint8_t id, uint16_t addr, const std::string& key) {
        return add(id, addr, sizeof(T), key);
    }

    // Vector Overloads
    SmartGroupReader& add(const std::vector<uint8_t>& ids, uint16_t addr, uint16_t len, const std::string& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        // Note: For vector add, we allow same key for different IDs. 
        // checkDuplicateKey checks (ID, Key) pair, so it works correctly.
        for (auto id : ids) {
            checkDuplicateKey(id, key);
            requests_.emplace_back(id, addr, len, key);
        }
        return *this;
    }

    SmartGroupReader& add(const std::vector<uint8_t>& ids, const std::string& item_name, const std::string& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto id : ids) {
            checkDuplicateKey(id, key);
            uint16_t model_number = getModelNumber(id);
            if (model_number == 0) continue;

            auto item = ControlTable::getItemInfo(model_number, item_name);
            if (item) {
                requests_.emplace_back(id, item->address, item->size, key);
            } else {
                throw DxlRuntimeError("[SmartGroup] Item not found in Control Table: " + item_name);
            }
        }
        return *this;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        requests_.clear();
    }

    struct ItemResult {
        std::map<std::string, int32_t> key_map;
        std::vector<int32_t> values;
        
        int32_t operator[](const std::string& key) {
            return key_map[key];
        }
        
        int32_t operator[](size_t index) {
            if (index < values.size()) return values[index];
            return 0;
        }
    };

    struct ReadResult {
        std::map<uint8_t, ItemResult> items;
        std::vector<int32_t> values; // All values in request order

        ItemResult& operator[](uint8_t id) {
            return items[id];
        }

        std::vector<int32_t> toVector() const {
            return values;
        }
        
        template <typename T>
        std::optional<T> get(uint8_t id, const std::string& key) const {
            auto it = items.find(id);
            if (it != items.end()) {
                auto kit = it->second.key_map.find(key);
                if (kit != it->second.key_map.end()) {
                    return static_cast<T>(kit->second);
                }
            }
            return std::nullopt;
        }
    };

    struct Status {
        bool success;
        std::string error_message;
    };

    std::pair<ReadResult, Status> read() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (requests_.empty()) {
            return {{}, {true, ""}};
        }

        // Analyze Topology
        bool use_sync = true;
        uint16_t first_addr = std::get<1>(requests_[0]);
        uint16_t first_len = std::get<2>(requests_[0]);

        for (const auto& req : requests_) {
            if (std::get<1>(req) != first_addr || std::get<2>(req) != first_len) {
                use_sync = false;
                break;
            }
        }

        std::unique_ptr<GroupReader> strategy;
        if (use_sync) {
            strategy = std::make_unique<SyncGroupReader>(port_, ph_, first_addr, first_len);
        } else {
            strategy = std::make_unique<BulkGroupReader>(port_, ph_);
        }

        for (const auto& req : requests_) {
            strategy->addParam(std::get<0>(req), std::get<1>(req), std::get<2>(req));
        }

        auto result = strategy->read();
        
        ReadResult read_result;
        Status status;

        if (result.is_ok()) {
            status.success = true;
            
            for (const auto& req : requests_) {
                uint8_t id = std::get<0>(req);
                uint16_t addr = std::get<1>(req);
                uint16_t len = std::get<2>(req);
                std::string key = std::get<3>(req);
                
                int32_t val = strategy->getData(id, addr, len);
                
                read_result.values.push_back(val);
                read_result.items[id].values.push_back(val);
                if (!key.empty()) {
                    read_result.items[id].key_map[key] = val;
                }
            }
        } else {
            status.success = false;
            status.error_message = result.err().msg;
        }

        return {read_result, status};
    }

    ReadResult read_or_throw() {
        auto result = read();
        if (!result.second.success) {
            throw DxlError(0, 0, result.second.error_message);
        }
        return result.first;
    }

private:
    PortHandler* port_;
    PacketHandler* ph_;
    mutable std::mutex mutex_;
    std::map<uint8_t, uint16_t> model_number_cache_;
    // ID, Address, Length, Key
    std::vector<std::tuple<uint8_t, uint16_t, uint16_t, std::string>> requests_;

    uint16_t getModelNumber(uint8_t id) {
        if (model_number_cache_.find(id) != model_number_cache_.end()) {
            return model_number_cache_[id];
        }
        
        uint16_t model_number = 0;
        int result = ph_->ping(port_, id, &model_number);
        if (result == COMM_SUCCESS) {
            model_number_cache_[id] = model_number;
            return model_number;
        }
        return 0;
    }

    void checkDuplicateKey(uint8_t id, const std::string& key) {
        if (key.empty()) return;
        for (const auto& req : requests_) {
            if (std::get<0>(req) == id && std::get<3>(req) == key) {
                throw std::invalid_argument("Duplicate key '" + key + "' for ID " + std::to_string(id));
            }
        }
    }
};

class SmartGroupWriter {
public:
    struct WriteStatus {
        int success_count;
        bool success;
        std::string error_message;
    };

    SmartGroupWriter(PortHandler* port, PacketHandler* ph)
        : port_(port), ph_(ph) {}

    template <typename T>
    SmartGroupWriter& add(uint8_t id, uint16_t addr, T data) {
        std::lock_guard<std::mutex> lock(mutex_);
        requests_.emplace_back(id, addr, sizeof(T), static_cast<int32_t>(data));
        return *this;
    }

    template <typename T>
    SmartGroupWriter& add(uint8_t id, const std::string& item_name, T data) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        uint16_t model_number = getModelNumber(id);
        if (model_number == 0) {
            return *this;
        }

        auto item = ControlTable::getItemInfo(model_number, item_name);
        if (item) {
            requests_.emplace_back(id, item->address, item->size, static_cast<int32_t>(data));
        } else {
            throw DxlRuntimeError("[SmartGroup] Item not found in Control Table: " + item_name);
        }
        return *this;
    }

    // Vector Overloads
    template <typename T>
    SmartGroupWriter& add(const std::vector<uint8_t>& ids, uint16_t addr, T data) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto id : ids) {
            requests_.emplace_back(id, addr, sizeof(T), static_cast<int32_t>(data));
        }
        return *this;
    }

    template <typename T>
    SmartGroupWriter& add(const std::vector<uint8_t>& ids, const std::string& item_name, T data) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto id : ids) {
            uint16_t model_number = getModelNumber(id);
            if (model_number == 0) continue;

            auto item = ControlTable::getItemInfo(model_number, item_name);
            if (item) {
                requests_.emplace_back(id, item->address, item->size, static_cast<int32_t>(data));
            } else {
                throw DxlRuntimeError("[SmartGroup] Item not found in Control Table: " + item_name);
            }
        }
        return *this;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        requests_.clear();
    }

    // Update values for existing requests in order
    void update(const std::vector<int32_t>& values) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (values.size() != requests_.size()) {
            // Size mismatch, maybe throw or ignore?
            // For safety, we can update as many as possible or throw.
            // Let's update min(values.size(), requests_.size())
        }
        size_t count = std::min(values.size(), requests_.size());
        for (size_t i = 0; i < count; ++i) {
            std::get<3>(requests_[i]) = values[i];
        }
    }

    // Convenience: Update and Write
    WriteStatus write(const std::vector<int32_t>& values) {
        update(values);
        return write_or_throw();
    }

    WriteStatus write_or_throw() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (requests_.empty()) return {0, true, ""};

        // Analyze Topology
        bool use_sync = true;
        uint16_t first_addr = std::get<1>(requests_[0]);
        uint16_t first_len = std::get<2>(requests_[0]);

        for (const auto& req : requests_) {
            if (std::get<1>(req) != first_addr || std::get<2>(req) != first_len) {
                use_sync = false;
                break;
            }
        }

        std::unique_ptr<GroupWriter> strategy;
        if (use_sync) {
            strategy = std::make_unique<SyncGroupWriter>(port_, ph_, first_addr, first_len);
        } else {
            strategy = std::make_unique<BulkGroupWriter>(port_, ph_);
        }

        for (const auto& req : requests_) {
            strategy->addParam(std::get<0>(req), std::get<1>(req), std::get<2>(req), std::get<3>(req));
        }

        auto result = strategy->write();
        
        if (result.is_ok()) {
            return {static_cast<int>(requests_.size()), true, ""};
        } else {
            throw DxlError(result.err()); // Or return status
        }
    }

private:
    PortHandler* port_;
    PacketHandler* ph_;
    mutable std::mutex mutex_;
    std::map<uint8_t, uint16_t> model_number_cache_;
    // ID, Address, Length, Data
    std::vector<std::tuple<uint8_t, uint16_t, uint16_t, int32_t>> requests_;

    uint16_t getModelNumber(uint8_t id) {
        if (model_number_cache_.find(id) != model_number_cache_.end()) {
            return model_number_cache_[id];
        }
        
        uint16_t model_number = 0;
        int result = ph_->ping(port_, id, &model_number);
        if (result == COMM_SUCCESS) {
            model_number_cache_[id] = model_number;
            return model_number;
        }
        return 0;
    }
};

} // namespace dynamixel

#endif // DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_SMART_GROUP_HPP_
