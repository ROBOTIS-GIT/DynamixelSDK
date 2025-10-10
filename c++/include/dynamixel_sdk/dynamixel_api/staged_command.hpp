#ifndef DYNAMIXEL_SDK_DYNAMIXEL_API_STAGED_COMMAND_HPP_
#define DYNAMIXEL_SDK_DYNAMIXEL_API_STAGED_COMMAND_HPP_

#include <cstdint>
#include <vector>

namespace dynamixel
{
enum class CommandType {
    WRITE,
    READ
};

struct StagedCommand
{
  StagedCommand(
    CommandType _command_type,
    uint8_t _id,
    uint16_t _address,
    uint16_t _length,
    const std::vector<uint8_t>& _data)
    : command_type(_command_type),
      id(_id),
      address(_address),
      length(_length),
      data(_data) {}

  CommandType command_type;
  uint8_t id;
  uint16_t address;
  uint16_t length;
  std::vector<uint8_t> data;
};

}  // namespace dynamixel

#endif  // DYNAMIXEL_SDK_DYNAMIXEL_API_STAGED_COMMAND_HPP_
