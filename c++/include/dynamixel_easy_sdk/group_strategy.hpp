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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_GROUP_STRATEGY_HPP_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_GROUP_STRATEGY_HPP_

#include <vector>
#include <memory>
#include <map>
#include "dynamixel_easy_sdk/data_types.hpp"
#include "dynamixel_easy_sdk/dynamixel_error.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

namespace dynamixel {

// Abstract Strategy Interface for Reading
class GroupReader {
public:
    virtual ~GroupReader() = default;
    virtual void addParam(uint8_t id, uint16_t addr, uint16_t len) = 0;
    virtual void clearParam() = 0;
    virtual Result<void, DxlError> read() = 0;
    virtual int32_t getData(uint8_t id, uint16_t addr, uint16_t len) = 0;
};

// Abstract Strategy Interface for Writing
class GroupWriter {
public:
    virtual ~GroupWriter() = default;
    virtual void addParam(uint8_t id, uint16_t addr, uint16_t len, int32_t data) = 0;
    virtual void clearParam() = 0;
    virtual Result<void, DxlError> write() = 0;
};

// Concrete Strategy: Sync Read (Fastest, requires identical address/length)
class SyncGroupReader : public GroupReader {
public:
    SyncGroupReader(PortHandler* port, PacketHandler* ph, uint16_t addr, uint16_t len)
        : group_sync_read_(port, ph, addr, len) {}

    void addParam(uint8_t id, uint16_t addr, uint16_t len) override {
        // SyncRead ignores addr/len per motor, assumes they match constructor
        group_sync_read_.addParam(id);
    }

    void clearParam() override {
        group_sync_read_.clearParam();
    }

    Result<void, DxlError> read() override {
        int dxl_comm_result = group_sync_read_.txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            return Result<void, DxlError>::Err(
                DxlError(dxl_comm_result, 0, "SyncRead TxRx Failed"));
        }
        return Result<void, DxlError>::Ok();
    }

    int32_t getData(uint8_t id, uint16_t addr, uint16_t len) override {
        if (group_sync_read_.isAvailable(id, addr, len)) {
            return group_sync_read_.getData(id, addr, len);
        }
        return 0;
    }

private:
    ::dynamixel::GroupSyncRead group_sync_read_;
};

// Concrete Strategy: Bulk Read (Flexible)
class BulkGroupReader : public GroupReader {
public:
    BulkGroupReader(PortHandler* port, PacketHandler* ph)
        : group_bulk_read_(port, ph) {}

    void addParam(uint8_t id, uint16_t addr, uint16_t len) override {
        group_bulk_read_.addParam(id, addr, len);
    }

    void clearParam() override {
        group_bulk_read_.clearParam();
    }

    Result<void, DxlError> read() override {
        int dxl_comm_result = group_bulk_read_.txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
             return Result<void, DxlError>::Err(
                DxlError(dxl_comm_result, 0, "BulkRead TxRx Failed"));
        }
        return Result<void, DxlError>::Ok();
    }

    int32_t getData(uint8_t id, uint16_t addr, uint16_t len) override {
        if (group_bulk_read_.isAvailable(id, addr, len)) {
            return group_bulk_read_.getData(id, addr, len);
        }
        return 0;
    }

private:
    ::dynamixel::GroupBulkRead group_bulk_read_;
};

// Concrete Strategy: Sync Write (Fastest)
class SyncGroupWriter : public GroupWriter {
public:
    SyncGroupWriter(PortHandler* port, PacketHandler* ph, uint16_t addr, uint16_t len)
        : group_sync_write_(port, ph, addr, len) {}

    void addParam(uint8_t id, uint16_t addr, uint16_t len, int32_t data) override {
        // SyncWrite ignores addr/len per motor, assumes they match constructor
        // We need to convert data to byte array
        uint8_t param[4];
        // TODO: Handle endianness properly or use SDK utility
        // The SDK has makeParam functions usually, or we do it manually.
        // For simplicity, assuming Little Endian (standard for DXL)
        param[0] = DXL_LOBYTE(DXL_LOWORD(data));
        param[1] = DXL_HIBYTE(DXL_LOWORD(data));
        param[2] = DXL_LOBYTE(DXL_HIWORD(data));
        param[3] = DXL_HIBYTE(DXL_HIWORD(data));
        
        // Adjust for length? GroupSyncWrite expects fixed length.
        // If len < 4, we should only copy relevant bytes?
        // The SDK's addParam takes uint8_t* data.
        group_sync_write_.addParam(id, param);
    }

    void clearParam() override {
        group_sync_write_.clearParam();
    }

    Result<void, DxlError> write() override {
        int dxl_comm_result = group_sync_write_.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            return Result<void, DxlError>::Err(
                DxlError(dxl_comm_result, 0, "SyncWrite Tx Failed"));
        }
        return Result<void, DxlError>::Ok();
    }

private:
    ::dynamixel::GroupSyncWrite group_sync_write_;
};

// Concrete Strategy: Bulk Write (Flexible)
class BulkGroupWriter : public GroupWriter {
public:
    BulkGroupWriter(PortHandler* port, PacketHandler* ph)
        : group_bulk_write_(port, ph) {}

    void addParam(uint8_t id, uint16_t addr, uint16_t len, int32_t data) override {
        uint8_t param[4];
        param[0] = DXL_LOBYTE(DXL_LOWORD(data));
        param[1] = DXL_HIBYTE(DXL_LOWORD(data));
        param[2] = DXL_LOBYTE(DXL_HIWORD(data));
        param[3] = DXL_HIBYTE(DXL_HIWORD(data));
        
        group_bulk_write_.addParam(id, addr, len, param);
    }

    void clearParam() override {
        group_bulk_write_.clearParam();
    }

    Result<void, DxlError> write() override {
        int dxl_comm_result = group_bulk_write_.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            return Result<void, DxlError>::Err(
                DxlError(dxl_comm_result, 0, "BulkWrite Tx Failed"));
        }
        return Result<void, DxlError>::Ok();
    }

private:
    ::dynamixel::GroupBulkWrite group_bulk_write_;
};

} // namespace dynamixel

#endif // DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_GROUP_STRATEGY_HPP_
