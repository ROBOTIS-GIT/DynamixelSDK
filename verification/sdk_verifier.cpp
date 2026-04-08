#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <thread>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace {

struct Options {
  std::string device = "/dev/ttyUSB0";
  std::string control_table_dir = VERIFICATION_CONTROL_TABLE_DIR;
  int baudrate = 57600;
  float protocol = 2.0f;
  std::vector<int> ids = {1};
  std::set<std::string> tests = {"all"};
  int loop_count = 50;
  int move_delta = 500;
  int move_timeout_ms = 3000;
  bool skip_write = false;
};

struct FieldInfo {
  uint16_t address = 0;
  uint16_t size = 0;
  std::string name;
};

struct ModelInfo {
  int model_number = 0;
  std::string filename;
  std::map<std::string, FieldInfo> fields;
};

struct DeviceInfo {
  uint8_t id = 0;
  uint16_t model_number = 0;
  uint8_t firmware_version = 0;
  const ModelInfo* model = nullptr;
};

struct StepTiming {
  std::string name;
  bool success = false;
  double seconds = 0.0;
  bool skipped = false;
};

struct CommTiming {
  std::string op;
  int id = -1;
  std::string field_name;
  uint16_t address = 0;
  uint16_t size = 0;
  double milliseconds = 0.0;
};

struct CommTimingAggregate {
  int count = 0;
  double total_milliseconds = 0.0;
};

std::vector<CommTiming> g_comm_timings;

std::string Trim(const std::string& text) {
  const size_t first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const size_t last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::vector<std::string> Split(const std::string& text, char delimiter) {
  std::vector<std::string> parts;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, delimiter)) {
    item = Trim(item);
    if (!item.empty()) {
      parts.push_back(item);
    }
  }
  return parts;
}

std::string DescribeField(const FieldInfo& field) {
  std::ostringstream oss;
  oss << field.name << " (addr=" << field.address << ", size=" << field.size << ")";
  return oss.str();
}

std::string JoinFieldDescriptions(const std::vector<const FieldInfo*>& fields) {
  std::ostringstream oss;
  bool first = true;
  for (const FieldInfo* field : fields) {
    if (field == nullptr) {
      continue;
    }
    if (!first) {
      oss << ", ";
    }
    first = false;
    oss << DescribeField(*field);
  }
  return oss.str();
}

void PrintFieldInfo(uint8_t id, const std::string& action, const FieldInfo& field) {
  std::cout << "[INFO] id=" << static_cast<int>(id) << " " << action
            << " control table field " << DescribeField(field) << "\n";
}

void PrintAccessSummary(const std::string& label,
                        const std::vector<const FieldInfo*>& read_fields,
                        const std::vector<const FieldInfo*>& write_fields = {}) {
  const std::string reads = JoinFieldDescriptions(read_fields);
  const std::string writes = JoinFieldDescriptions(write_fields);
  if (!reads.empty()) {
    std::cout << "[INFO] " << label << " read control tables: " << reads << "\n";
  }
  if (!writes.empty()) {
    std::cout << "[INFO] " << label << " write control tables: " << writes << "\n";
  }
}

bool ParseInt(const std::string& text, int* value) {
  try {
    size_t index = 0;
    const int parsed = std::stoi(text, &index, 0);
    if (index != text.size()) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseFloat(const std::string& text, float* value) {
  try {
    size_t index = 0;
    const float parsed = std::stof(text, &index);
    if (index != text.size()) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

void PrintUsage(const char* argv0) {
  std::cout
      << "Usage: " << argv0 << " [options]\n"
      << "  --device <path>              Serial device path\n"
      << "  --baud <rate>                Baudrate\n"
      << "  --protocol <1.0|2.0>         Protocol version\n"
      << "  --ids <id[,id...]>           Comma-separated Dynamixel IDs\n"
      << "  --tests <name[,name...]>     Tests to run\n"
      << "  --loop-count <count>         Repeat count for loop test\n"
      << "  --move-delta <value>         Position delta for move test\n"
      << "  --move-timeout-ms <ms>       Timeout for move test\n"
      << "  --control-table-dir <path>   Control table directory\n"
      << "  --skip-write                 Skip write/sync-write/bulk-write tests\n"
      << "  --help                       Show this help message\n"
      << "\n"
      << "Available tests:\n"
      << "  all, ping, read, broadcast, sync-read, bulk-read,\n"
      << "  fast-sync-read, fast-bulk-read, write, write-2, write-4,\n"
      << "  sync-write, bulk-write, move, loop\n";
}

bool ParseIds(const std::string& text, std::vector<int>* ids) {
  std::vector<int> parsed_ids;
  for (const std::string& part : Split(text, ',')) {
    int value = 0;
    if (!ParseInt(part, &value) || value < 0 || value > 252) {
      return false;
    }
    parsed_ids.push_back(value);
  }
  if (parsed_ids.empty()) {
    return false;
  }
  ids->swap(parsed_ids);
  return true;
}

bool ParseTests(const std::string& text, std::set<std::string>* tests) {
  static const std::set<std::string> kAllowed = {
      "all", "ping", "read", "broadcast", "sync-read", "bulk-read",
      "fast-sync-read", "fast-bulk-read", "write", "write-2", "write-4",
      "sync-write", "bulk-write", "move", "loop"};
  std::set<std::string> parsed;
  for (const std::string& part : Split(text, ',')) {
    if (kAllowed.find(part) == kAllowed.end()) {
      return false;
    }
    parsed.insert(part);
  }
  if (parsed.empty()) {
    return false;
  }
  tests->swap(parsed);
  return true;
}

bool ParseArgs(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto require_value = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << name << "\n";
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      return false;
    }
    if (arg == "--device") {
      const char* value = require_value("--device");
      if (value == nullptr) return false;
      options->device = value;
      continue;
    }
    if (arg == "--control-table-dir") {
      const char* value = require_value("--control-table-dir");
      if (value == nullptr) return false;
      options->control_table_dir = value;
      continue;
    }
    if (arg == "--baud") {
      const char* value = require_value("--baud");
      if (value == nullptr) return false;
      if (!ParseInt(value, &options->baudrate) || options->baudrate <= 0) {
        std::cerr << "Invalid baudrate: " << value << "\n";
        return false;
      }
      continue;
    }
    if (arg == "--protocol") {
      const char* value = require_value("--protocol");
      if (value == nullptr) return false;
      if (!ParseFloat(value, &options->protocol) ||
          (options->protocol != 1.0f && options->protocol != 2.0f)) {
        std::cerr << "Invalid protocol: " << value << "\n";
        return false;
      }
      continue;
    }
    if (arg == "--ids") {
      const char* value = require_value("--ids");
      if (value == nullptr) return false;
      if (!ParseIds(value, &options->ids)) {
        std::cerr << "Invalid ids: " << value << "\n";
        return false;
      }
      continue;
    }
    if (arg == "--tests") {
      const char* value = require_value("--tests");
      if (value == nullptr) return false;
      if (!ParseTests(value, &options->tests)) {
        std::cerr << "Invalid tests: " << value << "\n";
        return false;
      }
      continue;
    }
    if (arg == "--loop-count") {
      const char* value = require_value("--loop-count");
      if (value == nullptr) return false;
      if (!ParseInt(value, &options->loop_count) || options->loop_count <= 0) {
        std::cerr << "Invalid loop count: " << value << "\n";
        return false;
      }
      continue;
    }
    if (arg == "--move-delta") {
      const char* value = require_value("--move-delta");
      if (value == nullptr) return false;
      if (!ParseInt(value, &options->move_delta) || options->move_delta <= 0) {
        std::cerr << "Invalid move delta: " << value << "\n";
        return false;
      }
      continue;
    }
    if (arg == "--move-timeout-ms") {
      const char* value = require_value("--move-timeout-ms");
      if (value == nullptr) return false;
      if (!ParseInt(value, &options->move_timeout_ms) || options->move_timeout_ms <= 0) {
        std::cerr << "Invalid move timeout: " << value << "\n";
        return false;
      }
      continue;
    }
    if (arg == "--skip-write") {
      options->skip_write = true;
      continue;
    }

    std::cerr << "Unknown argument: " << arg << "\n";
    return false;
  }
  return true;
}

bool ShouldRun(const Options& options, const std::string& test_name) {
  return options.tests.count("all") > 0 || options.tests.count(test_name) > 0;
}

uint32_t ComparisonTolerance(const std::string& field_name) {
  static const std::set<std::string> kDynamicFields = {
      "Present Position", "Present Velocity", "Present Current", "Present Load"};
  return (kDynamicFields.count(field_name) > 0) ? 10u : 0u;
}

bool ValuesMatch(const FieldInfo& field, uint32_t expected, uint32_t actual) {
  const uint32_t tolerance = ComparisonTolerance(field.name);
  const uint32_t diff = (expected > actual) ? (expected - actual) : (actual - expected);
  return diff <= tolerance;
}

std::map<int, std::string> LoadModelMap(const std::string& control_table_dir) {
  std::map<int, std::string> model_map;
  std::ifstream input(control_table_dir + "/dynamixel.model");
  if (!input) {
    return model_map;
  }

  std::string line;
  while (std::getline(input, line)) {
    line = Trim(line);
    if (line.empty() || !std::isdigit(static_cast<unsigned char>(line[0]))) {
      continue;
    }
    std::istringstream iss(line);
    int model_number = 0;
    std::string filename;
    if (iss >> model_number >> filename) {
      model_map[model_number] = filename;
    }
  }
  return model_map;
}

bool LoadControlTableFile(const std::string& path, int model_number, const std::string& filename,
                          ModelInfo* model) {
  std::ifstream input(path);
  if (!input) {
    return false;
  }

  model->model_number = model_number;
  model->filename = filename;
  model->fields.clear();

  std::string line;
  bool in_control_table = false;
  while (std::getline(input, line)) {
    line = Trim(line);
    if (line.empty()) {
      continue;
    }
    if (line == "[control table]") {
      in_control_table = true;
      continue;
    }
    if (!in_control_table) {
      continue;
    }
    if (line[0] == '[') {
      break;
    }
    if (!std::isdigit(static_cast<unsigned char>(line[0]))) {
      continue;
    }

    std::istringstream iss(line);
    int address = 0;
    int size = 0;
    if (!(iss >> address >> size)) {
      continue;
    }
    std::string name;
    std::getline(iss, name);
    name = Trim(name);
    if (name.empty()) {
      continue;
    }
    model->fields[name] = FieldInfo{static_cast<uint16_t>(address), static_cast<uint16_t>(size), name};
  }

  return !model->fields.empty();
}

const FieldInfo* FindField(const ModelInfo& model, const std::string& name) {
  const auto it = model.fields.find(name);
  return it == model.fields.end() ? nullptr : &it->second;
}

const FieldInfo* FindFirstField(const ModelInfo& model, const std::vector<std::string>& names) {
  for (const std::string& name : names) {
    const FieldInfo* field = FindField(model, name);
    if (field != nullptr) {
      return field;
    }
  }
  return nullptr;
}

const FieldInfo* FindCommonField(const std::vector<DeviceInfo>& devices,
                                 const std::vector<std::string>& names) {
  if (devices.empty()) {
    return nullptr;
  }
  for (const std::string& name : names) {
    const FieldInfo* reference = FindField(*devices.front().model, name);
    if (reference == nullptr) {
      continue;
    }
    bool matches = true;
    for (size_t i = 1; i < devices.size(); ++i) {
      const FieldInfo* field = FindField(*devices[i].model, name);
      if (field == nullptr || field->address != reference->address || field->size != reference->size) {
        matches = false;
        break;
      }
    }
    if (matches) {
      return reference;
    }
  }
  return nullptr;
}

std::string FormatCommError(dynamixel::PacketHandler* packet_handler, int comm_result, uint8_t dxl_error) {
  if (comm_result != COMM_SUCCESS) {
    return packet_handler->getTxRxResult(comm_result);
  }
  if (dxl_error != 0) {
    return packet_handler->getRxPacketError(dxl_error);
  }
  return "Unknown error";
}

void PrintTestHeader(const std::string& name) {
  std::cout << "\n== " << name << " ==\n";
}

void PrintTestDescription(const std::string& text) {
  std::cout << "[INFO] " << text << "\n";
}

template <typename Func>
bool RunTimedStep(const std::string& name, std::vector<StepTiming>* timings, Func&& func) {
  const auto started_at = std::chrono::steady_clock::now();
  const bool success = func();
  const auto finished_at = std::chrono::steady_clock::now();
  const double seconds =
      std::chrono::duration_cast<std::chrono::duration<double>>(finished_at - started_at).count();

  timings->push_back(StepTiming{name, success, seconds, false});
  std::cout << "[TIME] " << name << ": " << std::fixed << std::setprecision(3)
            << seconds << " sec" << (success ? "" : " (failed)") << "\n";
  return success;
}

void RecordSkippedStep(const std::string& name, std::vector<StepTiming>* timings) {
  timings->push_back(StepTiming{name, false, 0.0, true});
  std::cout << "[TIME] " << name << ": skipped\n";
}

void PrintCommTiming(const std::string& op, uint8_t id, const FieldInfo* field,
                     std::chrono::steady_clock::time_point started_at,
                     std::chrono::steady_clock::time_point finished_at) {
  const double milliseconds =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(finished_at - started_at).count();
  g_comm_timings.push_back(CommTiming{
      op,
      (id <= 252) ? static_cast<int>(id) : -1,
      field != nullptr ? field->name : "",
      static_cast<uint16_t>(field != nullptr ? field->address : 0),
      static_cast<uint16_t>(field != nullptr ? field->size : 0),
      milliseconds});
  std::cout << "[COMM] " << op;
  if (id <= 252) {
    std::cout << " id=" << static_cast<int>(id);
  }
  if (field != nullptr) {
    std::cout << " field=" << field->name << " addr=" << field->address << " size=" << field->size;
  }
  std::cout << " " << std::fixed << std::setprecision(3) << milliseconds << " ms\n";
}

bool ReadValue(dynamixel::PacketHandler* packet_handler, dynamixel::PortHandler* port_handler, uint8_t id,
               const FieldInfo& field, uint32_t* value, std::string* error_message) {
  int comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;
  *value = 0;
  const auto started_at = std::chrono::steady_clock::now();

  switch (field.size) {
    case 1: {
      uint8_t out = 0;
      comm_result = packet_handler->read1ByteTxRx(port_handler, id, field.address, &out, &dxl_error);
      *value = out;
      break;
    }
    case 2: {
      uint16_t out = 0;
      comm_result = packet_handler->read2ByteTxRx(port_handler, id, field.address, &out, &dxl_error);
      *value = out;
      break;
    }
    case 4: {
      uint32_t out = 0;
      comm_result = packet_handler->read4ByteTxRx(port_handler, id, field.address, &out, &dxl_error);
      *value = out;
      break;
    }
    default:
      *error_message = "Unsupported field size: " + std::to_string(field.size);
      return false;
  }
  const auto finished_at = std::chrono::steady_clock::now();
  PrintCommTiming("read", id, &field, started_at, finished_at);

  if (comm_result != COMM_SUCCESS || dxl_error != 0) {
    *error_message = FormatCommError(packet_handler, comm_result, dxl_error);
    return false;
  }
  return true;
}

bool WriteValue(dynamixel::PacketHandler* packet_handler, dynamixel::PortHandler* port_handler, uint8_t id,
                const FieldInfo& field, uint32_t value, std::string* error_message) {
  int comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;
  const auto started_at = std::chrono::steady_clock::now();

  switch (field.size) {
    case 1:
      comm_result = packet_handler->write1ByteTxRx(port_handler, id, field.address,
                                                   static_cast<uint8_t>(value), &dxl_error);
      break;
    case 2:
      comm_result = packet_handler->write2ByteTxRx(port_handler, id, field.address,
                                                   static_cast<uint16_t>(value), &dxl_error);
      break;
    case 4:
      comm_result = packet_handler->write4ByteTxRx(port_handler, id, field.address, value, &dxl_error);
      break;
    default:
      *error_message = "Unsupported field size: " + std::to_string(field.size);
      return false;
  }
  const auto finished_at = std::chrono::steady_clock::now();
  PrintCommTiming("write", id, &field, started_at, finished_at);

  if (comm_result != COMM_SUCCESS || dxl_error != 0) {
    *error_message = FormatCommError(packet_handler, comm_result, dxl_error);
    return false;
  }
  return true;
}

std::vector<uint8_t> EncodeValue(uint32_t value, uint16_t size) {
  std::vector<uint8_t> encoded(size, 0);
  for (uint16_t i = 0; i < size; ++i) {
    encoded[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);
  }
  return encoded;
}

bool SetTorqueEnabled(dynamixel::PacketHandler* packet_handler, dynamixel::PortHandler* port_handler,
                      uint8_t id, const ModelInfo& model, bool enabled, std::string* error_message) {
  const FieldInfo* torque_field = FindField(model, "Torque Enable");
  if (torque_field == nullptr || torque_field->size != 1) {
    *error_message = "Torque Enable field is unavailable";
    return false;
  }
  return WriteValue(packet_handler, port_handler, id, *torque_field, enabled ? 1u : 0u, error_message);
}

bool ReadTorqueEnabled(dynamixel::PacketHandler* packet_handler, dynamixel::PortHandler* port_handler,
                       uint8_t id, const ModelInfo& model, bool* enabled, std::string* error_message) {
  const FieldInfo* torque_field = FindField(model, "Torque Enable");
  if (torque_field == nullptr || torque_field->size != 1) {
    *error_message = "Torque Enable field is unavailable";
    return false;
  }
  uint32_t value = 0;
  if (!ReadValue(packet_handler, port_handler, id, *torque_field, &value, error_message)) {
    return false;
  }
  *enabled = (value != 0);
  return true;
}

uint32_t ClampValue(uint32_t value, uint32_t min_value, uint32_t max_value);

bool SelectTargetPosition(dynamixel::PacketHandler* packet_handler, dynamixel::PortHandler* port_handler,
                          uint8_t id, const ModelInfo& model, const FieldInfo& goal_position,
                          uint32_t present, int move_delta, uint32_t* target,
                          std::string* error_message) {
  uint32_t min_limit = 0;
  uint32_t max_limit = (goal_position.size == 2) ? 0x03FFu : 0x000FFFFFu;
  if (const FieldInfo* field = FindField(model, "CW Angle Limit")) {
    ReadValue(packet_handler, port_handler, id, *field, &min_limit, error_message);
  }
  if (const FieldInfo* field = FindField(model, "CCW Angle Limit")) {
    ReadValue(packet_handler, port_handler, id, *field, &max_limit, error_message);
  }
  if (const FieldInfo* field = FindField(model, "Min Position Limit")) {
    ReadValue(packet_handler, port_handler, id, *field, &min_limit, error_message);
  }
  if (const FieldInfo* field = FindField(model, "Max Position Limit")) {
    ReadValue(packet_handler, port_handler, id, *field, &max_limit, error_message);
  }

  *target = ClampValue(
      static_cast<uint32_t>(std::max<int64_t>(0, static_cast<int64_t>(present) + move_delta)),
      min_limit, max_limit);
  if (*target == present) {
    *target = ClampValue(
        static_cast<uint32_t>(std::max<int64_t>(0, static_cast<int64_t>(present) - move_delta)),
        min_limit, max_limit);
  }
  return *target != present;
}

bool WaitForPosition(dynamixel::PacketHandler* packet_handler, dynamixel::PortHandler* port_handler,
                     uint8_t id, const FieldInfo& present_position, uint32_t target,
                     int timeout_ms, std::string* error_message) {
  const uint32_t tolerance = (present_position.size == 2) ? 10u : 20u;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    uint32_t observed = 0;
    if (!ReadValue(packet_handler, port_handler, id, present_position, &observed, error_message)) {
      return false;
    }
    const uint32_t diff = (observed > target) ? (observed - target) : (target - observed);
    if (diff <= tolerance) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  *error_message = "timeout";
  return false;
}

const FieldInfo* FindWritable2ByteField(const ModelInfo& model) {
  return FindFirstField(model, {
      "Velocity I Gain", "Velocity P Gain", "Position D Gain",
      "Position I Gain", "Position P Gain", "Feedforward 2nd Gain",
      "Feedforward 1st Gain", "Moving Speed", "Max Torque"});
}

const FieldInfo* FindWritable4ByteField(const ModelInfo& model) {
  return FindFirstField(model, {
      "Profile Acceleration", "Profile Velocity", "Goal Position", "Homing Offset"});
}

bool VerifyWriteFieldWithTorqueOff(dynamixel::PacketHandler* packet_handler,
                                   dynamixel::PortHandler* port_handler,
                                   const DeviceInfo& device,
                                   const FieldInfo& field,
                                   uint32_t candidate_value,
                                   const std::string& label) {
  std::string error_message;
  bool original_torque_enabled = false;
  if (!ReadTorqueEnabled(packet_handler, port_handler, device.id, *device.model,
                         &original_torque_enabled, &error_message)) {
    std::cerr << "[FAIL] read torque state before " << label
              << " id=" << static_cast<int>(device.id) << ": " << error_message << "\n";
    return false;
  }

  if (original_torque_enabled &&
      !SetTorqueEnabled(packet_handler, port_handler, device.id, *device.model, false, &error_message)) {
    std::cerr << "[FAIL] disable torque before " << label
              << " id=" << static_cast<int>(device.id) << ": " << error_message << "\n";
    return false;
  }

  uint32_t original_value = 0;
  if (!ReadValue(packet_handler, port_handler, device.id, field, &original_value, &error_message)) {
    std::cerr << "[FAIL] read " << field.name << " before " << label
              << " id=" << static_cast<int>(device.id) << ": " << error_message << "\n";
    if (original_torque_enabled) {
      SetTorqueEnabled(packet_handler, port_handler, device.id, *device.model, true, &error_message);
    }
    return false;
  }

  const uint32_t mask = (field.size == 2) ? 0xFFFFu : 0xFFFFFFFFu;
  uint32_t toggled = (candidate_value == original_value)
      ? ((original_value + 1u) & mask)
      : (candidate_value & mask);
  if (toggled == original_value) {
    toggled = (original_value ^ 0x1u) & mask;
  }

  if (!WriteValue(packet_handler, port_handler, device.id, field, toggled, &error_message)) {
    std::cerr << "[FAIL] write " << field.name << " during " << label
              << " id=" << static_cast<int>(device.id) << ": " << error_message << "\n";
    if (original_torque_enabled) {
      SetTorqueEnabled(packet_handler, port_handler, device.id, *device.model, true, &error_message);
    }
    return false;
  }

  uint32_t observed = 0;
  if (!ReadValue(packet_handler, port_handler, device.id, field, &observed, &error_message)) {
    std::cerr << "[FAIL] read " << field.name << " after " << label
              << " id=" << static_cast<int>(device.id) << ": " << error_message << "\n";
    if (original_torque_enabled) {
      SetTorqueEnabled(packet_handler, port_handler, device.id, *device.model, true, &error_message);
    }
    return false;
  }
  if (observed != toggled) {
    std::cerr << "[FAIL] " << label << " mismatch id=" << static_cast<int>(device.id)
              << " field=" << field.name << " expected=" << toggled
              << " actual=" << observed << "\n";
    if (original_torque_enabled) {
      SetTorqueEnabled(packet_handler, port_handler, device.id, *device.model, true, &error_message);
    }
    return false;
  }

  if (!WriteValue(packet_handler, port_handler, device.id, field, original_value, &error_message)) {
    std::cerr << "[FAIL] restore " << field.name << " after " << label
              << " id=" << static_cast<int>(device.id) << ": " << error_message << "\n";
    if (original_torque_enabled) {
      SetTorqueEnabled(packet_handler, port_handler, device.id, *device.model, true, &error_message);
    }
    return false;
  }

  if (original_torque_enabled &&
      !SetTorqueEnabled(packet_handler, port_handler, device.id, *device.model, true, &error_message)) {
    std::cerr << "[FAIL] restore torque after " << label
              << " id=" << static_cast<int>(device.id) << ": " << error_message << "\n";
    return false;
  }

  std::cout << "[PASS] " << label << " " << field.name
            << " write/read/restore id=" << static_cast<int>(device.id) << "\n";
  return true;
}

uint32_t ClampValue(uint32_t value, uint32_t min_value, uint32_t max_value) {
  return std::max(min_value, std::min(max_value, value));
}

bool VerifyPingAndLoadModels(const Options& options, dynamixel::PacketHandler* packet_handler,
                             dynamixel::PortHandler* port_handler, std::vector<ModelInfo>* models,
                             std::vector<DeviceInfo>* devices) {
  PrintTestHeader("ping");
  PrintTestDescription("Ping each requested ID, identify the model, and load its control table.");
  const std::map<int, std::string> model_map = LoadModelMap(options.control_table_dir);
  if (model_map.empty()) {
    std::cerr << "[FAIL] could not load " << options.control_table_dir << "/dynamixel.model\n";
    return false;
  }

  // DeviceInfo keeps raw pointers to entries in models.
  models->reserve(options.ids.size());
  devices->reserve(options.ids.size());

  std::map<int, size_t> loaded_models;
  for (int raw_id : options.ids) {
    uint8_t dxl_error = 0;
    uint16_t model_number = 0;
    const uint8_t id = static_cast<uint8_t>(raw_id);
    const auto ping_started_at = std::chrono::steady_clock::now();
    const int comm_result = packet_handler->ping(port_handler, id, &model_number, &dxl_error);
    const auto ping_finished_at = std::chrono::steady_clock::now();
    PrintCommTiming("ping", id, nullptr, ping_started_at, ping_finished_at);
    if (comm_result != COMM_SUCCESS || dxl_error != 0) {
      std::cerr << "[FAIL] ping id=" << raw_id << ": "
                << FormatCommError(packet_handler, comm_result, dxl_error) << "\n";
      return false;
    }

    const auto file_it = model_map.find(model_number);
    if (file_it == model_map.end()) {
      std::cerr << "[FAIL] model number " << model_number << " is missing from dynamixel.model\n";
      return false;
    }

    size_t model_index = 0;
    const auto loaded_it = loaded_models.find(model_number);
    if (loaded_it == loaded_models.end()) {
      ModelInfo model;
      const std::string model_path = options.control_table_dir + "/" + file_it->second;
      if (!LoadControlTableFile(model_path, model_number, file_it->second, &model)) {
        std::cerr << "[FAIL] failed to load control table: " << model_path << "\n";
        return false;
      }
      models->push_back(model);
      model_index = models->size() - 1;
      loaded_models[model_number] = model_index;
    } else {
      model_index = loaded_it->second;
    }

    const FieldInfo* firmware_field = FindField((*models)[model_index], "Firmware Version");
    uint32_t firmware_version = 0;
    if (firmware_field != nullptr) {
      std::string error_message;
      if (!ReadValue(packet_handler, port_handler, id, *firmware_field, &firmware_version, &error_message)) {
        std::cerr << "[FAIL] read firmware version id=" << raw_id << ": " << error_message << "\n";
        return false;
      }
    }

    devices->push_back(DeviceInfo{id, model_number, static_cast<uint8_t>(firmware_version), &(*models)[model_index]});
    std::cout << "[PASS] id=" << raw_id << " model=" << model_number
              << " file=" << (*models)[model_index].filename
              << " firmware=" << static_cast<int>(firmware_version) << "\n";
  }
  return true;
}

bool VerifyBroadcastPing(const Options& options, dynamixel::PacketHandler* packet_handler,
                         dynamixel::PortHandler* port_handler, const std::vector<DeviceInfo>& devices) {
  if (!ShouldRun(options, "broadcast")) {
    return true;
  }
  PrintTestHeader("broadcast");
  PrintTestDescription("Use broadcast ping and verify that all requested IDs are discovered.");
  if (options.protocol != 2.0f) {
    std::cout << "[SKIP] broadcast ping is protocol 2.0 only\n";
    return true;
  }

  std::vector<uint8_t> discovered;
  const auto broadcast_started_at = std::chrono::steady_clock::now();
  const int comm_result = packet_handler->broadcastPing(port_handler, discovered);
  const auto broadcast_finished_at = std::chrono::steady_clock::now();
  PrintCommTiming("broadcastPing", 253, nullptr, broadcast_started_at, broadcast_finished_at);
  if (comm_result != COMM_SUCCESS) {
    std::cerr << "[FAIL] broadcastPing(): " << packet_handler->getTxRxResult(comm_result) << "\n";
    return false;
  }

  for (const DeviceInfo& device : devices) {
    if (std::find(discovered.begin(), discovered.end(), device.id) == discovered.end()) {
      std::cerr << "[FAIL] broadcast ping did not discover id=" << static_cast<int>(device.id) << "\n";
      return false;
    }
  }

  std::cout << "[PASS] broadcastPing() discovered " << discovered.size() << " device(s)\n";
  return true;
}

bool VerifySingleRead(const Options& options, dynamixel::PacketHandler* packet_handler,
                      dynamixel::PortHandler* port_handler, const std::vector<DeviceInfo>& devices) {
  if (!ShouldRun(options, "read")) {
    return true;
  }
  PrintTestHeader("read");
  PrintTestDescription("Validate single read APIs against model number, firmware, and one live status field.");

  for (const DeviceInfo& device : devices) {
    const FieldInfo* model_field = FindField(*device.model, "Model Number");
    if (model_field == nullptr || model_field->size != 2) {
      std::cerr << "[FAIL] missing Model Number field for id=" << static_cast<int>(device.id) << "\n";
      return false;
    }

    uint32_t model_value = 0;
    std::string error_message;
    if (!ReadValue(packet_handler, port_handler, device.id, *model_field, &model_value, &error_message)) {
      std::cerr << "[FAIL] read model number id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }
    if (model_value != device.model_number) {
      std::cerr << "[FAIL] model number mismatch for id=" << static_cast<int>(device.id)
                << ": ping=" << device.model_number << " read=" << model_value << "\n";
      return false;
    }

    const FieldInfo* firmware_field = FindField(*device.model, "Firmware Version");
    if (firmware_field != nullptr) {
      uint32_t firmware_value = 0;
      if (!ReadValue(packet_handler, port_handler, device.id, *firmware_field, &firmware_value, &error_message)) {
        std::cerr << "[FAIL] read firmware version id=" << static_cast<int>(device.id)
                  << ": " << error_message << "\n";
        return false;
      }
    }

    const FieldInfo* any_read_field = FindFirstField(
        *device.model, {"Present Position", "Present Velocity", "Present Input Voltage",
                        "Present Load", "Present Temperature", "Hardware Error Status", "Moving"});
    if (any_read_field == nullptr) {
      std::cout << "[SKIP] id=" << static_cast<int>(device.id)
                << " has no additional candidate read field\n";
      continue;
    }

    PrintAccessSummary("read",
                       {model_field, firmware_field, any_read_field},
                       {});
    PrintFieldInfo(device.id, "read", *model_field);
    if (firmware_field != nullptr) {
      PrintFieldInfo(device.id, "read", *firmware_field);
    }
    PrintFieldInfo(device.id, "read", *any_read_field);

    uint32_t value = 0;
    if (!ReadValue(packet_handler, port_handler, device.id, *any_read_field, &value, &error_message)) {
      std::cerr << "[FAIL] read " << any_read_field->name << " id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }

    std::cout << "[PASS] id=" << static_cast<int>(device.id)
              << " model=" << device.model_number
              << " " << any_read_field->name << "=" << value << "\n";
  }
  return true;
}

bool VerifySyncRead(const Options& options, dynamixel::PacketHandler* packet_handler,
                    dynamixel::PortHandler* port_handler, const std::vector<DeviceInfo>& devices) {
  if (!ShouldRun(options, "sync-read")) {
    return true;
  }
  PrintTestHeader("sync-read");
  PrintTestDescription("Read one common field from multiple IDs with GroupSyncRead and compare to single reads.");
  if (options.protocol != 2.0f) {
    std::cout << "[SKIP] sync-read is protocol 2.0 only\n";
    return true;
  }

  const FieldInfo* field = FindCommonField(devices, {"Present Position", "Hardware Error Status", "Moving", "LED"});
  if (field == nullptr) {
    std::cout << "[SKIP] no common field for sync-read across selected ids\n";
    return true;
  }

  PrintAccessSummary("sync-read", {field}, {});
  std::cout << "[INFO] sync-read common field " << DescribeField(*field) << "\n";

  dynamixel::GroupSyncRead group(port_handler, packet_handler, field->address, field->size);
  for (const DeviceInfo& device : devices) {
    if (!group.addParam(device.id)) {
      std::cerr << "[FAIL] GroupSyncRead::addParam(" << static_cast<int>(device.id) << ")\n";
      return false;
    }
  }

  const auto sync_read_started_at = std::chrono::steady_clock::now();
  const int sync_read_result = group.txRxPacket();
  const auto sync_read_finished_at = std::chrono::steady_clock::now();
  PrintCommTiming("GroupSyncRead::txRxPacket", 253, field, sync_read_started_at, sync_read_finished_at);
  if (sync_read_result != COMM_SUCCESS) {
    std::cerr << "[FAIL] GroupSyncRead::txRxPacket()\n";
    return false;
  }

  for (const DeviceInfo& device : devices) {
    std::string error_message;
    uint32_t expected = 0;
    if (!ReadValue(packet_handler, port_handler, device.id, *field, &expected, &error_message)) {
      std::cerr << "[FAIL] single read for sync-read compare id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }
    const uint32_t actual = group.getData(device.id, field->address, field->size);
    if (!ValuesMatch(*field, expected, actual)) {
      std::cerr << "[FAIL] sync-read mismatch id=" << static_cast<int>(device.id)
                << " field=" << field->name << " expected=" << expected
                << " actual=" << actual
                << " tolerance=" << ComparisonTolerance(field->name) << "\n";
      return false;
    }
  }

  std::cout << "[PASS] GroupSyncRead verified field " << field->name << "\n";
  return true;
}

bool VerifyBulkRead(const Options& options, dynamixel::PacketHandler* packet_handler,
                    dynamixel::PortHandler* port_handler, const std::vector<DeviceInfo>& devices) {
  if (!ShouldRun(options, "bulk-read")) {
    return true;
  }
  PrintTestHeader("bulk-read");
  PrintTestDescription("Read one or more IDs with GroupBulkRead and compare results to single reads.");

  dynamixel::GroupBulkRead group(port_handler, packet_handler);
  std::vector<const FieldInfo*> fields;
  std::vector<uint32_t> expected_values;
  fields.reserve(devices.size());
  expected_values.reserve(devices.size());

  for (size_t i = 0; i < devices.size(); ++i) {
    const DeviceInfo& device = devices[i];
    const FieldInfo* field = (i % 2 == 0)
        ? FindFirstField(*device.model, {"Firmware Version", "Present Temperature", "LED"})
        : FindFirstField(*device.model, {"Present Position", "Present Velocity", "Present Input Voltage"});
    if (field == nullptr) {
      field = FindFirstField(*device.model, {"Model Number", "Moving", "Hardware Error Status"});
    }
    if (field == nullptr) {
      std::cout << "[SKIP] no readable field found for bulk-read id=" << static_cast<int>(device.id) << "\n";
      return true;
    }
    PrintAccessSummary("bulk-read", {field}, {});
    PrintFieldInfo(device.id, "bulk-read", *field);
    if (!group.addParam(device.id, field->address, field->size)) {
      std::cerr << "[FAIL] GroupBulkRead::addParam(" << static_cast<int>(device.id) << ")\n";
      return false;
    }
    std::string error_message;
    uint32_t expected = 0;
    if (!ReadValue(packet_handler, port_handler, device.id, *field, &expected, &error_message)) {
      std::cerr << "[FAIL] single read for bulk-read compare id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }
    fields.push_back(field);
    expected_values.push_back(expected);
  }

  const auto bulk_read_started_at = std::chrono::steady_clock::now();
  const int bulk_read_result = group.txRxPacket();
  const auto bulk_read_finished_at = std::chrono::steady_clock::now();
  PrintCommTiming("GroupBulkRead::txRxPacket", 253, nullptr, bulk_read_started_at, bulk_read_finished_at);
  if (bulk_read_result != COMM_SUCCESS) {
    std::cerr << "[FAIL] GroupBulkRead::txRxPacket()\n";
    return false;
  }

  for (size_t i = 0; i < devices.size(); ++i) {
    const uint32_t actual = group.getData(devices[i].id, fields[i]->address, fields[i]->size);
    if (!ValuesMatch(*fields[i], expected_values[i], actual)) {
      std::cerr << "[FAIL] bulk-read mismatch id=" << static_cast<int>(devices[i].id)
                << " field=" << fields[i]->name << " expected=" << expected_values[i]
                << " actual=" << actual
                << " tolerance=" << ComparisonTolerance(fields[i]->name) << "\n";
      return false;
    }
  }

  std::cout << "[PASS] GroupBulkRead verified " << devices.size() << " device(s)\n";
  return true;
}

bool VerifyFastSyncRead(const Options& options, dynamixel::PacketHandler* packet_handler,
                        dynamixel::PortHandler* port_handler, const std::vector<DeviceInfo>& devices) {
  if (!ShouldRun(options, "fast-sync-read")) {
    return true;
  }
  PrintTestHeader("fast-sync-read");
  PrintTestDescription("Read one common field from multiple IDs with GroupFastSyncRead and compare to single reads.");
  if (options.protocol != 2.0f) {
    std::cout << "[SKIP] fast-sync-read is protocol 2.0 only\n";
    return true;
  }

  const FieldInfo* field = FindCommonField(devices, {"Present Position", "Hardware Error Status", "Moving", "LED"});
  if (field == nullptr) {
    std::cout << "[SKIP] no common field for fast-sync-read across selected ids\n";
    return true;
  }

  PrintAccessSummary("fast-sync-read", {field}, {});
  std::cout << "[INFO] fast-sync-read common field " << DescribeField(*field) << "\n";

  dynamixel::GroupFastSyncRead group(port_handler, packet_handler, field->address, field->size);
  for (const DeviceInfo& device : devices) {
    if (!group.addParam(device.id)) {
      std::cerr << "[FAIL] GroupFastSyncRead::addParam(" << static_cast<int>(device.id) << ")\n";
      return false;
    }
  }

  const auto fast_sync_read_started_at = std::chrono::steady_clock::now();
  const int fast_sync_read_result = group.txRxPacket();
  const auto fast_sync_read_finished_at = std::chrono::steady_clock::now();
  PrintCommTiming("GroupFastSyncRead::txRxPacket", 253, field,
                  fast_sync_read_started_at, fast_sync_read_finished_at);
  if (fast_sync_read_result != COMM_SUCCESS) {
    std::cerr << "[FAIL] GroupFastSyncRead::txRxPacket()\n";
    return false;
  }

  for (const DeviceInfo& device : devices) {
    std::string error_message;
    uint32_t expected = 0;
    if (!ReadValue(packet_handler, port_handler, device.id, *field, &expected, &error_message)) {
      std::cerr << "[FAIL] single read for fast-sync-read compare id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }
    const uint32_t actual = group.getData(device.id, field->address, field->size);
    if (!ValuesMatch(*field, expected, actual)) {
      std::cerr << "[FAIL] fast-sync-read mismatch id=" << static_cast<int>(device.id)
                << " field=" << field->name << " expected=" << expected
                << " actual=" << actual
                << " tolerance=" << ComparisonTolerance(field->name) << "\n";
      return false;
    }
  }

  std::cout << "[PASS] GroupFastSyncRead verified field " << field->name << "\n";
  return true;
}

bool VerifyFastBulkRead(const Options& options, dynamixel::PacketHandler* packet_handler,
                        dynamixel::PortHandler* port_handler, const std::vector<DeviceInfo>& devices) {
  if (!ShouldRun(options, "fast-bulk-read")) {
    return true;
  }
  PrintTestHeader("fast-bulk-read");
  PrintTestDescription("Read one or more IDs with GroupFastBulkRead and compare results to single reads.");
  if (options.protocol != 2.0f) {
    std::cout << "[SKIP] fast-bulk-read is protocol 2.0 only\n";
    return true;
  }

  dynamixel::GroupFastBulkRead group(port_handler, packet_handler);
  std::vector<const FieldInfo*> fields;
  std::vector<uint32_t> expected_values;
  for (size_t i = 0; i < devices.size(); ++i) {
    const DeviceInfo& device = devices[i];
    const FieldInfo* field = (i % 2 == 0)
        ? FindFirstField(*device.model, {"Firmware Version", "Present Temperature", "LED"})
        : FindFirstField(*device.model, {"Present Position", "Present Velocity", "Present Input Voltage"});
    if (field == nullptr) {
      field = FindFirstField(*device.model, {"Model Number", "Moving", "Hardware Error Status"});
    }
    if (field == nullptr) {
      std::cout << "[SKIP] no readable field found for fast-bulk-read id=" << static_cast<int>(device.id) << "\n";
      return true;
    }
    PrintAccessSummary("fast-bulk-read", {field}, {});
    PrintFieldInfo(device.id, "fast-bulk-read", *field);
    if (!group.addParam(device.id, field->address, field->size)) {
      std::cerr << "[FAIL] GroupFastBulkRead::addParam(" << static_cast<int>(device.id) << ")\n";
      return false;
    }
    std::string error_message;
    uint32_t expected = 0;
    if (!ReadValue(packet_handler, port_handler, device.id, *field, &expected, &error_message)) {
      std::cerr << "[FAIL] single read for fast-bulk-read compare id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }
    fields.push_back(field);
    expected_values.push_back(expected);
  }

  const auto fast_bulk_read_started_at = std::chrono::steady_clock::now();
  const int fast_bulk_read_result = group.txRxPacket();
  const auto fast_bulk_read_finished_at = std::chrono::steady_clock::now();
  PrintCommTiming("GroupFastBulkRead::txRxPacket", 253, nullptr,
                  fast_bulk_read_started_at, fast_bulk_read_finished_at);
  if (fast_bulk_read_result != COMM_SUCCESS) {
    std::cerr << "[FAIL] GroupFastBulkRead::txRxPacket()\n";
    return false;
  }

  for (size_t i = 0; i < devices.size(); ++i) {
    const uint32_t actual = group.getData(devices[i].id, fields[i]->address, fields[i]->size);
    if (!ValuesMatch(*fields[i], expected_values[i], actual)) {
      std::cerr << "[FAIL] fast-bulk-read mismatch id=" << static_cast<int>(devices[i].id)
                << " field=" << fields[i]->name << " expected=" << expected_values[i]
                << " actual=" << actual
                << " tolerance=" << ComparisonTolerance(fields[i]->name) << "\n";
      return false;
    }
  }

  std::cout << "[PASS] GroupFastBulkRead verified " << devices.size() << " device(s)\n";
  return true;
}

bool VerifySingleWrite(const Options& options, dynamixel::PacketHandler* packet_handler,
                       dynamixel::PortHandler* port_handler, const std::vector<DeviceInfo>& devices) {
  if (!ShouldRun(options, "write")) {
    return true;
  }
  PrintTestHeader("write");
  PrintTestDescription("Toggle a 1-byte writable field such as LED, verify read-back, then restore it.");
  if (options.skip_write) {
    std::cout << "[SKIP] write tests disabled by --skip-write\n";
    return true;
  }

  for (const DeviceInfo& device : devices) {
    const FieldInfo* led_field = FindField(*device.model, "LED");
    if (led_field == nullptr || led_field->size != 1) {
      std::cout << "[SKIP] id=" << static_cast<int>(device.id) << " has no writable LED field\n";
      continue;
    }

    PrintAccessSummary("write", {led_field}, {led_field});
    PrintFieldInfo(device.id, "write/read/restore", *led_field);

    std::string error_message;
    uint32_t original = 0;
    if (!ReadValue(packet_handler, port_handler, device.id, *led_field, &original, &error_message)) {
      std::cerr << "[FAIL] read LED before write id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }

    const uint32_t toggled = (original == 0) ? 1 : 0;
    if (!WriteValue(packet_handler, port_handler, device.id, *led_field, toggled, &error_message)) {
      std::cerr << "[FAIL] write LED id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }

    uint32_t observed = 0;
    if (!ReadValue(packet_handler, port_handler, device.id, *led_field, &observed, &error_message)) {
      std::cerr << "[FAIL] read LED after write id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }
    if (observed != toggled) {
      std::cerr << "[FAIL] LED write mismatch id=" << static_cast<int>(device.id)
                << " expected=" << toggled << " actual=" << observed << "\n";
      return false;
    }

    if (!WriteValue(packet_handler, port_handler, device.id, *led_field, original, &error_message)) {
      std::cerr << "[FAIL] restore LED id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }

    std::cout << "[PASS] LED write/read/restore id=" << static_cast<int>(device.id) << "\n";
  }
  return true;
}

bool Verify2ByteWrite(const Options& options, dynamixel::PacketHandler* packet_handler,
                      dynamixel::PortHandler* port_handler, const std::vector<DeviceInfo>& devices) {
  if (!ShouldRun(options, "write-2")) {
    return true;
  }
  PrintTestHeader("write-2");
  PrintTestDescription("Write a safe 2-byte register with torque off, verify read-back, then restore it.");
  if (options.skip_write) {
    std::cout << "[SKIP] write tests disabled by --skip-write\n";
    return true;
  }

  bool executed = false;
  for (const DeviceInfo& device : devices) {
    const FieldInfo* field = FindWritable2ByteField(*device.model);
    if (field == nullptr || field->size != 2) {
      std::cout << "[SKIP] id=" << static_cast<int>(device.id) << " has no 2-byte writable field candidate\n";
      continue;
    }
    if (const FieldInfo* mode_field = FindField(*device.model, "Operating Mode")) {
      std::cout << "[INFO] id=" << static_cast<int>(device.id)
                << " write-2 excludes Goal PWM because it requires Operating Mode changes; mode field="
                << DescribeField(*mode_field) << "\n";
    }
    PrintAccessSummary("write-2", {field}, {field});
    PrintFieldInfo(device.id, "write/read/restore", *field);
    executed = true;
    if (!VerifyWriteFieldWithTorqueOff(packet_handler, port_handler, device, *field, 1u, "write-2")) {
      return false;
    }
  }

  if (!executed) {
    std::cout << "[SKIP] no device provided a 2-byte write field\n";
  }
  return true;
}

bool Verify4ByteWrite(const Options& options, dynamixel::PacketHandler* packet_handler,
                      dynamixel::PortHandler* port_handler, const std::vector<DeviceInfo>& devices) {
  if (!ShouldRun(options, "write-4")) {
    return true;
  }
  PrintTestHeader("write-4");
  PrintTestDescription("Write a safe 4-byte register with torque off, verify read-back, then restore it.");
  if (options.skip_write) {
    std::cout << "[SKIP] write tests disabled by --skip-write\n";
    return true;
  }

  bool executed = false;
  for (const DeviceInfo& device : devices) {
    const FieldInfo* field = FindWritable4ByteField(*device.model);
    if (field == nullptr || field->size != 4) {
      std::cout << "[SKIP] id=" << static_cast<int>(device.id) << " has no 4-byte writable field candidate\n";
      continue;
    }
    if (const FieldInfo* mode_field = FindField(*device.model, "Operating Mode")) {
      std::cout << "[INFO] id=" << static_cast<int>(device.id)
                << " write-4 excludes Goal Velocity because it requires Operating Mode changes; mode field="
                << DescribeField(*mode_field) << "\n";
    }
    PrintAccessSummary("write-4", {field}, {field});
    PrintFieldInfo(device.id, "write/read/restore", *field);
    executed = true;
    if (!VerifyWriteFieldWithTorqueOff(packet_handler, port_handler, device, *field, 1u, "write-4")) {
      return false;
    }
  }

  if (!executed) {
    std::cout << "[SKIP] no device provided a 4-byte write field\n";
  }
  return true;
}

bool VerifySyncWrite(const Options& options, dynamixel::PacketHandler* packet_handler,
                     dynamixel::PortHandler* port_handler, const std::vector<DeviceInfo>& devices) {
  if (!ShouldRun(options, "sync-write")) {
    return true;
  }
  PrintTestHeader("sync-write");
  PrintTestDescription("Use GroupSyncWrite to toggle a common LED field, then move all motors together and restore.");
  if (options.skip_write) {
    std::cout << "[SKIP] write tests disabled by --skip-write\n";
    return true;
  }

  const FieldInfo* led_field = FindCommonField(devices, {"LED"});
  if (led_field == nullptr || led_field->size != 1) {
    std::cout << "[SKIP] no common LED field for sync-write across selected ids\n";
    return true;
  }

  PrintAccessSummary("sync-write", {led_field}, {led_field});
  std::cout << "[INFO] sync-write common write field " << DescribeField(*led_field) << "\n";

  dynamixel::GroupSyncWrite group(port_handler, packet_handler, led_field->address, led_field->size);
  std::vector<uint32_t> originals(devices.size(), 0);
  std::vector<std::vector<uint8_t>> values(devices.size());
  std::string error_message;

  for (size_t i = 0; i < devices.size(); ++i) {
    if (!ReadValue(packet_handler, port_handler, devices[i].id, *led_field, &originals[i], &error_message)) {
      std::cerr << "[FAIL] read LED before sync-write id=" << static_cast<int>(devices[i].id)
                << ": " << error_message << "\n";
      return false;
    }
    const uint32_t toggled = (originals[i] == 0) ? 1 : 0;
    values[i] = EncodeValue(toggled, led_field->size);
    if (!group.addParam(devices[i].id, values[i].data())) {
      std::cerr << "[FAIL] GroupSyncWrite::addParam(" << static_cast<int>(devices[i].id) << ")\n";
      return false;
    }
  }

  const auto sync_write_started_at = std::chrono::steady_clock::now();
  const int sync_write_result = group.txPacket();
  const auto sync_write_finished_at = std::chrono::steady_clock::now();
  PrintCommTiming("GroupSyncWrite::txPacket", 253, led_field, sync_write_started_at, sync_write_finished_at);
  if (sync_write_result != COMM_SUCCESS) {
    std::cerr << "[FAIL] GroupSyncWrite::txPacket()\n";
    return false;
  }

  for (size_t i = 0; i < devices.size(); ++i) {
    uint32_t observed = 0;
    if (!ReadValue(packet_handler, port_handler, devices[i].id, *led_field, &observed, &error_message)) {
      std::cerr << "[FAIL] read LED after sync-write id=" << static_cast<int>(devices[i].id)
                << ": " << error_message << "\n";
      return false;
    }
    const uint32_t expected = values[i][0];
    if (observed != expected) {
      std::cerr << "[FAIL] sync-write mismatch id=" << static_cast<int>(devices[i].id)
                << " expected=" << expected << " actual=" << observed << "\n";
      return false;
    }
  }

  for (size_t i = 0; i < devices.size(); ++i) {
    if (!WriteValue(packet_handler, port_handler, devices[i].id, *led_field, originals[i], &error_message)) {
      std::cerr << "[FAIL] restore LED after sync-write id=" << static_cast<int>(devices[i].id)
                << ": " << error_message << "\n";
      return false;
    }
  }

  std::cout << "[PASS] GroupSyncWrite verified and restored LED values\n";

  const FieldInfo* goal_field = FindCommonField(devices, {"Goal Position"});
  const FieldInfo* present_field = FindCommonField(devices, {"Present Position"});
  const FieldInfo* torque_field = FindCommonField(devices, {"Torque Enable"});
  if (goal_field == nullptr || present_field == nullptr || torque_field == nullptr ||
      goal_field->size != present_field->size || (goal_field->size != 2 && goal_field->size != 4)) {
    std::cout << "[SKIP] no common Goal/Present Position fields for sync-write group move\n";
    return true;
  }

  PrintAccessSummary("sync-write group move",
                     {present_field, goal_field, torque_field},
                     {goal_field, torque_field});
  std::cout << "[INFO] sync-write move fields goal=" << DescribeField(*goal_field)
            << ", present=" << DescribeField(*present_field)
            << ", torque=" << DescribeField(*torque_field) << "\n";

  dynamixel::GroupSyncWrite move_group(port_handler, packet_handler, goal_field->address, goal_field->size);
  std::vector<uint32_t> original_goals(devices.size(), 0);
  std::vector<std::vector<uint8_t>> move_values(devices.size());
  std::vector<uint8_t> torque_states(devices.size(), 0);
  for (size_t i = 0; i < devices.size(); ++i) {
    bool torque_enabled = false;
    if (!ReadTorqueEnabled(packet_handler, port_handler, devices[i].id, *devices[i].model,
                           &torque_enabled, &error_message)) {
      std::cerr << "[FAIL] read torque state before sync group move id="
                << static_cast<int>(devices[i].id) << ": " << error_message << "\n";
      return false;
    }
    torque_states[i] = torque_enabled ? 1u : 0u;
    if (!torque_states[i] &&
        !SetTorqueEnabled(packet_handler, port_handler, devices[i].id, *devices[i].model, true, &error_message)) {
      std::cerr << "[FAIL] enable torque before sync group move id="
                << static_cast<int>(devices[i].id) << ": " << error_message << "\n";
      return false;
    }
    uint32_t present = 0;
    if (!ReadValue(packet_handler, port_handler, devices[i].id, *present_field, &present, &error_message) ||
        !ReadValue(packet_handler, port_handler, devices[i].id, *goal_field, &original_goals[i], &error_message)) {
      std::cerr << "[FAIL] read position before sync group move id="
                << static_cast<int>(devices[i].id) << ": " << error_message << "\n";
      return false;
    }
    uint32_t target = 0;
    if (!SelectTargetPosition(packet_handler, port_handler, devices[i].id, *devices[i].model,
                              *goal_field, present, options.move_delta, &target, &error_message)) {
      std::cout << "[SKIP] id=" << static_cast<int>(devices[i].id)
                << " cannot choose a safe alternate target position for sync group move\n";
      return true;
    }
    std::cout << "[INFO] id=" << static_cast<int>(devices[i].id)
              << " sync-write move target=" << target
              << " restore_goal=" << original_goals[i] << "\n";
    move_values[i] = EncodeValue(target, goal_field->size);
    if (!move_group.addParam(devices[i].id, move_values[i].data())) {
      std::cerr << "[FAIL] GroupSyncWrite::addParam(sync group move, id="
                << static_cast<int>(devices[i].id) << ")\n";
      return false;
    }
  }

  const auto sync_move_started_at = std::chrono::steady_clock::now();
  const int sync_move_result = move_group.txPacket();
  const auto sync_move_finished_at = std::chrono::steady_clock::now();
  PrintCommTiming("GroupSyncWrite::txPacket(sync group move)", 253, goal_field,
                  sync_move_started_at, sync_move_finished_at);
  if (sync_move_result != COMM_SUCCESS) {
    std::cerr << "[FAIL] GroupSyncWrite::txPacket(sync group move)\n";
    return false;
  }

  for (size_t i = 0; i < devices.size(); ++i) {
    bool reached = false;
    const uint32_t target = move_values[i][0] |
                            (goal_field->size > 1 ? (static_cast<uint32_t>(move_values[i][1]) << 8) : 0) |
                            (goal_field->size > 2 ? (static_cast<uint32_t>(move_values[i][2]) << 16) : 0) |
                            (goal_field->size > 3 ? (static_cast<uint32_t>(move_values[i][3]) << 24) : 0);
    reached = WaitForPosition(packet_handler, port_handler, devices[i].id, *present_field, target,
                              options.move_timeout_ms, &error_message);
    if (!reached && error_message != "timeout") {
      std::cerr << "[FAIL] read present position during sync group move id="
                << static_cast<int>(devices[i].id) << ": " << error_message << "\n";
      return false;
    }
    if (!reached) {
      std::cerr << "[FAIL] sync group move timeout id=" << static_cast<int>(devices[i].id) << "\n";
      return false;
    }
  }

  for (size_t i = 0; i < devices.size(); ++i) {
    if (!WriteValue(packet_handler, port_handler, devices[i].id, *goal_field, original_goals[i], &error_message)) {
      std::cerr << "[FAIL] restore goal after sync group move id="
                << static_cast<int>(devices[i].id) << ": " << error_message << "\n";
      return false;
    }
    if (!WaitForPosition(packet_handler, port_handler, devices[i].id, *present_field, original_goals[i],
                         options.move_timeout_ms, &error_message) &&
        error_message != "timeout") {
      std::cerr << "[FAIL] read present position while restoring sync group move id="
                << static_cast<int>(devices[i].id) << ": " << error_message << "\n";
      return false;
    }
    if (!torque_states[i] &&
        !SetTorqueEnabled(packet_handler, port_handler, devices[i].id, *devices[i].model, false, &error_message)) {
      std::cerr << "[FAIL] restore torque after sync group move id="
                << static_cast<int>(devices[i].id) << ": " << error_message << "\n";
      return false;
    }
  }

  std::cout << "[PASS] GroupSyncWrite group move verified and restored goal positions\n";
  return true;
}

bool VerifyBulkWrite(const Options& options, dynamixel::PacketHandler* packet_handler,
                     dynamixel::PortHandler* port_handler, const std::vector<DeviceInfo>& devices) {
  if (!ShouldRun(options, "bulk-write")) {
    return true;
  }
  PrintTestHeader("bulk-write");
  PrintTestDescription("Use GroupBulkWrite to toggle per-device LED fields, then move motors per-device and restore.");
  if (options.skip_write) {
    std::cout << "[SKIP] write tests disabled by --skip-write\n";
    return true;
  }

  dynamixel::GroupBulkWrite group(port_handler, packet_handler);
  std::vector<const FieldInfo*> fields(devices.size(), nullptr);
  std::vector<uint32_t> originals(devices.size(), 0);
  std::vector<std::vector<uint8_t>> values(devices.size());
  std::string error_message;

  for (size_t i = 0; i < devices.size(); ++i) {
    fields[i] = FindField(*devices[i].model, "LED");
    if (fields[i] == nullptr || fields[i]->size != 1) {
      std::cout << "[SKIP] bulk-write needs LED on every selected id\n";
      return true;
    }
    PrintAccessSummary("bulk-write", {fields[i]}, {fields[i]});
    PrintFieldInfo(devices[i].id, "bulk-write", *fields[i]);
    if (!ReadValue(packet_handler, port_handler, devices[i].id, *fields[i], &originals[i], &error_message)) {
      std::cerr << "[FAIL] read LED before bulk-write id=" << static_cast<int>(devices[i].id)
                << ": " << error_message << "\n";
      return false;
    }
    const uint32_t toggled = (i % 2 == 0) ? 1 : 0;
    values[i] = EncodeValue(toggled, fields[i]->size);
    if (!group.addParam(devices[i].id, fields[i]->address, fields[i]->size, values[i].data())) {
      std::cerr << "[FAIL] GroupBulkWrite::addParam(" << static_cast<int>(devices[i].id) << ")\n";
      return false;
    }
  }

  const auto bulk_write_started_at = std::chrono::steady_clock::now();
  const int bulk_write_result = group.txPacket();
  const auto bulk_write_finished_at = std::chrono::steady_clock::now();
  PrintCommTiming("GroupBulkWrite::txPacket", 253, nullptr, bulk_write_started_at, bulk_write_finished_at);
  if (bulk_write_result != COMM_SUCCESS) {
    std::cerr << "[FAIL] GroupBulkWrite::txPacket()\n";
    return false;
  }

  for (size_t i = 0; i < devices.size(); ++i) {
    uint32_t observed = 0;
    if (!ReadValue(packet_handler, port_handler, devices[i].id, *fields[i], &observed, &error_message)) {
      std::cerr << "[FAIL] read LED after bulk-write id=" << static_cast<int>(devices[i].id)
                << ": " << error_message << "\n";
      return false;
    }
    const uint32_t expected = values[i][0];
    if (observed != expected) {
      std::cerr << "[FAIL] bulk-write mismatch id=" << static_cast<int>(devices[i].id)
                << " expected=" << expected << " actual=" << observed << "\n";
      return false;
    }
  }

  for (size_t i = 0; i < devices.size(); ++i) {
    if (!WriteValue(packet_handler, port_handler, devices[i].id, *fields[i], originals[i], &error_message)) {
      std::cerr << "[FAIL] restore LED after bulk-write id=" << static_cast<int>(devices[i].id)
                << ": " << error_message << "\n";
      return false;
    }
  }

  std::cout << "[PASS] GroupBulkWrite verified and restored LED values\n";

  dynamixel::GroupBulkWrite move_group(port_handler, packet_handler);
  std::vector<const FieldInfo*> goal_fields(devices.size(), nullptr);
  std::vector<const FieldInfo*> present_fields(devices.size(), nullptr);
  std::vector<uint32_t> original_goals(devices.size(), 0);
  std::vector<std::vector<uint8_t>> move_values(devices.size());
  std::vector<uint8_t> torque_states(devices.size(), 0);
  for (size_t i = 0; i < devices.size(); ++i) {
    goal_fields[i] = FindField(*devices[i].model, "Goal Position");
    present_fields[i] = FindField(*devices[i].model, "Present Position");
    if (goal_fields[i] == nullptr || present_fields[i] == nullptr ||
        goal_fields[i]->size != present_fields[i]->size ||
        (goal_fields[i]->size != 2 && goal_fields[i]->size != 4)) {
      std::cout << "[SKIP] bulk-write group move needs Goal/Present Position on every selected id\n";
      return true;
    }
    PrintAccessSummary("bulk-write group move",
                       {present_fields[i], goal_fields[i]},
                       {goal_fields[i]});
    std::cout << "[INFO] id=" << static_cast<int>(devices[i].id)
              << " bulk-write move fields goal=" << DescribeField(*goal_fields[i])
              << ", present=" << DescribeField(*present_fields[i]) << "\n";
    bool torque_enabled = false;
    if (!ReadTorqueEnabled(packet_handler, port_handler, devices[i].id, *devices[i].model,
                           &torque_enabled, &error_message)) {
      std::cerr << "[FAIL] read torque state before bulk group move id="
                << static_cast<int>(devices[i].id) << ": " << error_message << "\n";
      return false;
    }
    torque_states[i] = torque_enabled ? 1u : 0u;
    if (!torque_states[i] &&
        !SetTorqueEnabled(packet_handler, port_handler, devices[i].id, *devices[i].model, true, &error_message)) {
      std::cerr << "[FAIL] enable torque before bulk group move id="
                << static_cast<int>(devices[i].id) << ": " << error_message << "\n";
      return false;
    }
    uint32_t present = 0;
    if (!ReadValue(packet_handler, port_handler, devices[i].id, *present_fields[i], &present, &error_message) ||
        !ReadValue(packet_handler, port_handler, devices[i].id, *goal_fields[i], &original_goals[i], &error_message)) {
      std::cerr << "[FAIL] read position before bulk group move id="
                << static_cast<int>(devices[i].id) << ": " << error_message << "\n";
      return false;
    }
    uint32_t target = 0;
    if (!SelectTargetPosition(packet_handler, port_handler, devices[i].id, *devices[i].model,
                              *goal_fields[i], present, options.move_delta, &target, &error_message)) {
      std::cout << "[SKIP] id=" << static_cast<int>(devices[i].id)
                << " cannot choose a safe alternate target position for bulk group move\n";
      return true;
    }
    std::cout << "[INFO] id=" << static_cast<int>(devices[i].id)
              << " bulk-write move target=" << target
              << " restore_goal=" << original_goals[i] << "\n";
    move_values[i] = EncodeValue(target, goal_fields[i]->size);
    if (!move_group.addParam(devices[i].id, goal_fields[i]->address, goal_fields[i]->size,
                             move_values[i].data())) {
      std::cerr << "[FAIL] GroupBulkWrite::addParam(bulk group move, id="
                << static_cast<int>(devices[i].id) << ")\n";
      return false;
    }
  }

  const auto bulk_move_started_at = std::chrono::steady_clock::now();
  const int bulk_move_result = move_group.txPacket();
  const auto bulk_move_finished_at = std::chrono::steady_clock::now();
  PrintCommTiming("GroupBulkWrite::txPacket(bulk group move)", 253, nullptr,
                  bulk_move_started_at, bulk_move_finished_at);
  if (bulk_move_result != COMM_SUCCESS) {
    std::cerr << "[FAIL] GroupBulkWrite::txPacket(bulk group move)\n";
    return false;
  }

  for (size_t i = 0; i < devices.size(); ++i) {
    bool reached = false;
    const uint32_t target = move_values[i][0] |
                            (goal_fields[i]->size > 1 ? (static_cast<uint32_t>(move_values[i][1]) << 8) : 0) |
                            (goal_fields[i]->size > 2 ? (static_cast<uint32_t>(move_values[i][2]) << 16) : 0) |
                            (goal_fields[i]->size > 3 ? (static_cast<uint32_t>(move_values[i][3]) << 24) : 0);
    reached = WaitForPosition(packet_handler, port_handler, devices[i].id, *present_fields[i], target,
                              options.move_timeout_ms, &error_message);
    if (!reached && error_message != "timeout") {
      std::cerr << "[FAIL] read present position during bulk group move id="
                << static_cast<int>(devices[i].id) << ": " << error_message << "\n";
      return false;
    }
    if (!reached) {
      std::cerr << "[FAIL] bulk group move timeout id=" << static_cast<int>(devices[i].id) << "\n";
      return false;
    }
  }

  for (size_t i = 0; i < devices.size(); ++i) {
    if (!WriteValue(packet_handler, port_handler, devices[i].id, *goal_fields[i], original_goals[i], &error_message)) {
      std::cerr << "[FAIL] restore goal after bulk group move id="
                << static_cast<int>(devices[i].id) << ": " << error_message << "\n";
      return false;
    }
    if (!WaitForPosition(packet_handler, port_handler, devices[i].id, *present_fields[i], original_goals[i],
                         options.move_timeout_ms, &error_message) &&
        error_message != "timeout") {
      std::cerr << "[FAIL] read present position while restoring bulk group move id="
                << static_cast<int>(devices[i].id) << ": " << error_message << "\n";
      return false;
    }
    if (!torque_states[i] &&
        !SetTorqueEnabled(packet_handler, port_handler, devices[i].id, *devices[i].model, false, &error_message)) {
      std::cerr << "[FAIL] restore torque after bulk group move id="
                << static_cast<int>(devices[i].id) << ": " << error_message << "\n";
      return false;
    }
  }

  std::cout << "[PASS] GroupBulkWrite group move verified and restored goal positions\n";
  return true;
}

bool VerifyMove(const Options& options, dynamixel::PacketHandler* packet_handler,
                dynamixel::PortHandler* port_handler, const std::vector<DeviceInfo>& devices) {
  if (!ShouldRun(options, "move")) {
    return true;
  }
  PrintTestHeader("move");
  PrintTestDescription("Move each motor by Goal Position, wait until Present Position follows, then restore.");
  if (options.skip_write) {
    std::cout << "[SKIP] move test disabled by --skip-write\n";
    return true;
  }

  bool executed = false;
  for (const DeviceInfo& device : devices) {
    const FieldInfo* goal_position = FindField(*device.model, "Goal Position");
    const FieldInfo* present_position = FindField(*device.model, "Present Position");
    const FieldInfo* torque_enable = FindField(*device.model, "Torque Enable");
    if (goal_position == nullptr || present_position == nullptr || torque_enable == nullptr ||
        goal_position->size != present_position->size || (goal_position->size != 2 && goal_position->size != 4)) {
      std::cout << "[SKIP] id=" << static_cast<int>(device.id)
                << " does not expose compatible Goal/Present Position fields\n";
      continue;
    }

    PrintAccessSummary("move",
                       {present_position, goal_position, torque_enable},
                       {goal_position, torque_enable});
    std::cout << "[INFO] id=" << static_cast<int>(device.id)
              << " move fields goal=" << DescribeField(*goal_position)
              << ", present=" << DescribeField(*present_position)
              << ", torque=" << DescribeField(*torque_enable) << "\n";

    executed = true;
    std::string error_message;
    bool original_torque_enabled = false;
    if (!ReadTorqueEnabled(packet_handler, port_handler, device.id, *device.model,
                           &original_torque_enabled, &error_message)) {
      std::cerr << "[FAIL] read torque state before move id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }
    if (!original_torque_enabled &&
        !SetTorqueEnabled(packet_handler, port_handler, device.id, *device.model, true, &error_message)) {
      std::cerr << "[FAIL] enable torque before move id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }

    uint32_t original_goal = 0;
    if (!ReadValue(packet_handler, port_handler, device.id, *goal_position, &original_goal, &error_message)) {
      std::cerr << "[FAIL] read goal position before move id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }

    uint32_t present = 0;
    if (!ReadValue(packet_handler, port_handler, device.id, *present_position, &present, &error_message)) {
      std::cerr << "[FAIL] read present position before move id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }

    uint32_t target = 0;
    SelectTargetPosition(packet_handler, port_handler, device.id, *device.model, *goal_position,
                         present, options.move_delta, &target, &error_message);
    if (target == present) {
      std::cout << "[SKIP] id=" << static_cast<int>(device.id)
                << " cannot choose a safe alternate target position\n";
      if (!original_torque_enabled) {
        SetTorqueEnabled(packet_handler, port_handler, device.id, *device.model, false, &error_message);
      }
      continue;
    }

    std::cout << "[INFO] id=" << static_cast<int>(device.id)
              << " move target=" << target
              << " restore_goal=" << original_goal << "\n";

    if (!WriteValue(packet_handler, port_handler, device.id, *goal_position, target, &error_message)) {
      std::cerr << "[FAIL] write goal position for move id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }

    bool reached = false;
    reached = WaitForPosition(packet_handler, port_handler, device.id, *present_position, target,
                              options.move_timeout_ms, &error_message);
    if (!reached && error_message != "timeout") {
      std::cerr << "[FAIL] read present position during move id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }

    if (!WriteValue(packet_handler, port_handler, device.id, *goal_position, original_goal, &error_message)) {
      std::cerr << "[FAIL] restore goal position after move id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }

    if (!WaitForPosition(packet_handler, port_handler, device.id, *present_position, original_goal,
                         options.move_timeout_ms, &error_message) &&
        error_message != "timeout") {
      std::cerr << "[FAIL] read present position while restoring move id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }

    if (!original_torque_enabled &&
        !SetTorqueEnabled(packet_handler, port_handler, device.id, *device.model, false, &error_message)) {
      std::cerr << "[FAIL] restore torque after move id=" << static_cast<int>(device.id)
                << ": " << error_message << "\n";
      return false;
    }

    if (!reached) {
      std::cerr << "[FAIL] move timeout id=" << static_cast<int>(device.id)
                << " target=" << target << "\n";
      return false;
    }

    std::cout << "[PASS] move id=" << static_cast<int>(device.id)
              << " present=" << present << " target=" << target
              << " restored_goal=" << original_goal << "\n";
  }

  if (!executed) {
    std::cout << "[SKIP] no device provided a usable move field set\n";
  }
  return true;
}

bool VerifyLoop(const Options& options, dynamixel::PacketHandler* packet_handler,
                dynamixel::PortHandler* port_handler, const std::vector<DeviceInfo>& devices) {
  if (!ShouldRun(options, "loop")) {
    return true;
  }
  PrintTestHeader("loop");
  PrintTestDescription("Repeat a safe read many times to check communication stability and timing.");

  const FieldInfo* field = FindFirstField(*devices.front().model,
                                          {"Present Position", "Hardware Error Status", "Firmware Version", "Moving"});
  if (field == nullptr) {
    std::cout << "[SKIP] no candidate field for loop test\n";
    return true;
  }

  PrintAccessSummary("loop", {field}, {});
  std::cout << "[INFO] loop read field " << DescribeField(*field) << "\n";

  for (int i = 0; i < options.loop_count; ++i) {
    for (const DeviceInfo& device : devices) {
      uint32_t value = 0;
      std::string error_message;
      if (!ReadValue(packet_handler, port_handler, device.id, *field, &value, &error_message)) {
        std::cerr << "[FAIL] loop read iteration=" << i
                  << " id=" << static_cast<int>(device.id) << ": " << error_message << "\n";
        return false;
      }
    }
  }

  std::cout << "[PASS] loop test completed " << options.loop_count << " iterations on field "
            << field->name << "\n";
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!ParseArgs(argc, argv, &options)) {
    return 1;
  }

  std::cout << "Dynamixel SDK verifier\n"
            << "  device            : " << options.device << "\n"
            << "  baudrate          : " << options.baudrate << "\n"
            << "  protocol          : " << std::fixed << std::setprecision(1) << options.protocol << "\n"
            << "  ids               : ";
  for (size_t i = 0; i < options.ids.size(); ++i) {
    if (i != 0) std::cout << ",";
    std::cout << options.ids[i];
  }
  std::cout << "\n"
            << "  control table dir : " << options.control_table_dir << "\n";

  std::unique_ptr<dynamixel::PortHandler> port_handler(
      dynamixel::PortHandler::getPortHandler(options.device.c_str()));
  if (!port_handler) {
    std::cerr << "Failed to create PortHandler\n";
    return 1;
  }

  dynamixel::PacketHandler* packet_handler =
      dynamixel::PacketHandler::getPacketHandler(options.protocol);
  if (packet_handler == nullptr) {
    std::cerr << "Failed to create PacketHandler\n";
    return 1;
  }

  if (!port_handler->openPort()) {
    std::cerr << "[FAIL] openPort()\n";
    return 2;
  }
  std::cout << "[PASS] openPort()\n";

  if (!port_handler->setBaudRate(options.baudrate)) {
    std::cerr << "[FAIL] setBaudRate(" << options.baudrate << ")\n";
    port_handler->closePort();
    return 3;
  }
  std::cout << "[PASS] setBaudRate(" << options.baudrate << ")\n";

  const auto verification_started_at = std::chrono::steady_clock::now();
  std::vector<ModelInfo> models;
  std::vector<DeviceInfo> devices;
  std::vector<StepTiming> timings;
  g_comm_timings.clear();
  bool ok = true;
  auto run_or_skip = [&](const std::string& name, auto&& func) {
    const bool step_ok = RunTimedStep(name, &timings, std::forward<decltype(func)>(func));
    ok = ok && step_ok;
  };

  run_or_skip("ping", [&] {
    return VerifyPingAndLoadModels(options, packet_handler, port_handler.get(), &models, &devices);
  });
  run_or_skip("broadcast", [&] {
    return VerifyBroadcastPing(options, packet_handler, port_handler.get(), devices);
  });
  run_or_skip("read", [&] {
    return VerifySingleRead(options, packet_handler, port_handler.get(), devices);
  });
  run_or_skip("sync-read", [&] {
    return VerifySyncRead(options, packet_handler, port_handler.get(), devices);
  });
  run_or_skip("bulk-read", [&] {
    return VerifyBulkRead(options, packet_handler, port_handler.get(), devices);
  });
  run_or_skip("fast-sync-read", [&] {
    return VerifyFastSyncRead(options, packet_handler, port_handler.get(), devices);
  });
  run_or_skip("fast-bulk-read", [&] {
    return VerifyFastBulkRead(options, packet_handler, port_handler.get(), devices);
  });
  run_or_skip("write", [&] {
    return VerifySingleWrite(options, packet_handler, port_handler.get(), devices);
  });
  run_or_skip("write-2", [&] {
    return Verify2ByteWrite(options, packet_handler, port_handler.get(), devices);
  });
  run_or_skip("write-4", [&] {
    return Verify4ByteWrite(options, packet_handler, port_handler.get(), devices);
  });
  run_or_skip("sync-write", [&] {
    return VerifySyncWrite(options, packet_handler, port_handler.get(), devices);
  });
  run_or_skip("bulk-write", [&] {
    return VerifyBulkWrite(options, packet_handler, port_handler.get(), devices);
  });
  run_or_skip("move", [&] {
    return VerifyMove(options, packet_handler, port_handler.get(), devices);
  });
  run_or_skip("loop", [&] {
    return VerifyLoop(options, packet_handler, port_handler.get(), devices);
  });

  port_handler->closePort();
  std::cout << "[PASS] closePort()\n";

  const auto verification_finished_at = std::chrono::steady_clock::now();
  const double total_seconds =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          verification_finished_at - verification_started_at)
          .count();

  std::cout << "\nTiming summary:\n";
  for (const StepTiming& timing : timings) {
    std::cout << "  " << std::left << std::setw(16) << timing.name
              << " : ";
    if (timing.skipped) {
      std::cout << "skipped\n";
      continue;
    }
    std::cout << std::right << std::fixed << std::setprecision(3)
              << timing.seconds << " sec"
              << (timing.success ? "" : " (failed)") << "\n";
  }

  if (!g_comm_timings.empty()) {
    std::cout << "\nCommunication summary:\n";
    std::map<std::tuple<std::string, int, std::string, uint16_t, uint16_t>, CommTimingAggregate> aggregates;
    for (const CommTiming& timing : g_comm_timings) {
      auto& aggregate = aggregates[std::make_tuple(
          timing.op, timing.id, timing.field_name, timing.address, timing.size)];
      aggregate.count += 1;
      aggregate.total_milliseconds += timing.milliseconds;
    }
    for (const auto& entry : aggregates) {
      const auto& key = entry.first;
      const CommTimingAggregate& aggregate = entry.second;
      const std::string& op = std::get<0>(key);
      const int id = std::get<1>(key);
      const std::string& field_name = std::get<2>(key);
      const uint16_t address = std::get<3>(key);
      const uint16_t size = std::get<4>(key);
      std::cout << "  " << op;
      if (id >= 0) {
        std::cout << " id=" << id;
      }
      if (!field_name.empty()) {
        std::cout << " field=" << field_name
                  << " addr=" << address
                  << " size=" << size;
      }
      std::cout << " : count=" << aggregate.count
                << " total=" << std::fixed << std::setprecision(3)
                << aggregate.total_milliseconds << " ms"
                << " avg=" << (aggregate.total_milliseconds / aggregate.count) << " ms\n";
    }
  }
  std::cout << "  " << std::left << std::setw(16) << "total"
            << " : " << std::right << std::fixed << std::setprecision(3)
            << total_seconds << " sec\n";

  if (!ok) {
    return 4;
  }

  std::cout << "\nAll requested verification steps completed successfully.\n";
  return 0;
}
