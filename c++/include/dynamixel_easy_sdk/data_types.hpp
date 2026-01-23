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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_DATA_TYPES_HPP_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_DATA_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <optional>

namespace dynamixel {

class Motor;

struct ControlTableItem {
    std::string item_name;
    uint16_t address;
    uint8_t size;
};

enum class OperatingMode : uint8_t {
    CURRENT = 0,
    VELOCITY = 1,
    POSITION = 3,
    EXTENDED_POSITION = 4,
    CURRENT_BASED_POSITION = 5,
    PWM = 16
};

enum class ProfileConfiguration {
    TIME_BASED,
    VELOCITY_BASED
};

enum class Direction {
    NORMAL,
    REVERSE
};

enum class CommandType {
    READ,
    WRITE
};

enum class StatusRequest {
    UPDATE_TORQUE_STATUS,
    CHECK_OPERATING_MODE
};

struct StagedCommand {
    CommandType command_type;
    uint8_t id;
    uint16_t address;
    uint8_t length;
    std::vector<uint8_t> data;
    std::vector<StatusRequest> status_check_functions;
    Motor* motor_ref;
    std::optional<std::vector<OperatingMode>> operating_mode_check;

    StagedCommand(
        CommandType t,
        uint8_t i,
        uint16_t addr,
        uint8_t len,
        std::vector<uint8_t> d = {},
        std::vector<StatusRequest> checks = {},
        Motor* m = nullptr,
        std::vector<OperatingMode> modes = {}
    ) : command_type(t), id(i), address(addr), length(len), data(d), status_check_functions(checks), motor_ref(m)
    {
        if (!modes.empty()) {
            operating_mode_check = modes;
        }
    }
};

} // namespace dynamixel

#endif // DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_DATA_TYPES_HPP_