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

#include <cstdlib>
#include <stdexcept>

#include "dynamixel_easy_sdk/control_table.hpp"

namespace dynamixel
{
namespace
{
std::string configuredControlTableRootDir()
{
  if (const char * env = std::getenv("CONTROL_TABLE_PATH")) {
    if (env[0] != '\0') {
      return std::string(env);
    }
  }
  return "";
}

bool hasDynamixelModel(const std::string & dir)
{
  std::ifstream in(dir + "/dynamixel.model");
  return in.is_open();
}

std::string controlTableRootDir()
{
  const std::string candidate = configuredControlTableRootDir();
  if (!candidate.empty() && hasDynamixelModel(candidate)) {
    return candidate;
  }
  return std::string(CONTROL_TABLE_PATH);
}

std::string missingControlTableMessage(const std::string & file_name)
{
  std::string message =
    "Error: Could not open file " + file_name +
    ". Install DynamixelSDK (control_table data) or set CONTROL_TABLE_PATH to that directory.";

  const std::string configured_dir = configuredControlTableRootDir();
  if (!configured_dir.empty() && !hasDynamixelModel(configured_dir)) {
    message += " CONTROL_TABLE_PATH is set but invalid: " + configured_dir;
  }

  return message;
}

std::runtime_error malformedControlTableError(
  const std::string & file_name,
  size_t line_number,
  const std::string & line)
{
  return std::runtime_error(
    "Malformed control table entry in " + file_name +
    " at line " + std::to_string(line_number) + ": " + line);
}
}  // namespace

const std::map<uint16_t, std::string> ControlTable::ParsingModelList()
{
  std::map<uint16_t, std::string> tmp_model_list;
  const std::string root_dir = controlTableRootDir();
  std::string file_name = root_dir + "/dynamixel.model";
  std::ifstream infile(file_name);
  if (!infile.is_open()) {
    throw std::runtime_error(missingControlTableMessage(file_name));
  }

  std::string line;
  std::getline(infile, line);
  while (std::getline(infile, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream ss(line);
    std::string name;
    std::string number_str;
    if (std::getline(ss, number_str, '\t')) {
      if (std::getline(ss, name)) {
        try {
          uint16_t number = std::stoi(number_str);
          tmp_model_list[number] = name;
        } catch (const std::invalid_argument & e) {
          throw std::runtime_error("Invalid model number in " + file_name + ": " + number_str);
        } catch (const std::out_of_range & e) {
          throw std::runtime_error("Model number out of range in " + file_name + ": " + number_str);
        } catch (...) {
          throw std::runtime_error("Unknown error while parsing model list in " + file_name);
        }
      }
    }
  }
  return tmp_model_list;
}

const std::string ControlTable::getModelName(uint16_t model_number)
{
  static const std::map<uint16_t, std::string> model_name_list_ = ParsingModelList();
  auto it = model_name_list_.find(model_number);
  if (it == model_name_list_.end()) {
    throw DxlRuntimeError(
            "Model number is not found in dynamixel.model: " + std::to_string(model_number));
  }
  return it->second;
}

const std::map<std::string, ControlTableItem> & ControlTable::getControlTable(uint16_t model_number)
{
  static std::map<uint16_t, std::map<std::string, ControlTableItem>> control_tables_cache_;
  if (control_tables_cache_.count(model_number)) {
    return control_tables_cache_.at(model_number);
  }

  const std::string & model_filename = getModelName(model_number);
  const std::string root_dir = controlTableRootDir();
  std::string full_path = root_dir + "/" + model_filename;

  std::map<std::string, ControlTableItem> control_table;
  std::ifstream infile(full_path);

  if (!infile.is_open()) {
    throw std::runtime_error(missingControlTableMessage(full_path));
  }

  std::string line;
  size_t line_number = 0;
  bool in_section = false;
  bool found_control_table_section = false;

  while (std::getline(infile, line)) {
    ++line_number;
    if (line.empty()) {
      continue;
    }
    if (line == "[control table]") {
      in_section = true;
      found_control_table_section = true;
      if (!std::getline(infile, line)) {
        break;
      }
      ++line_number;
      continue;
    }

    if (in_section) {
      std::stringstream ss(line);
      std::string address_str, size_str, name;

      if (!(std::getline(ss, address_str, '\t') &&
        std::getline(ss, size_str, '\t') &&
        std::getline(ss, name)))
      {
        throw malformedControlTableError(full_path, line_number, line);
      }

      try {
        ControlTableItem item;
        item.address = std::stoi(address_str);
        item.size = std::stoi(size_str);
        control_table[name] = item;
      } catch (const std::exception & e) {
        throw std::runtime_error(
                "Error parsing control table item in " + full_path +
                " at line " + std::to_string(line_number) + ": " + line + " - " + e.what());
      }
    }
  }

  if (!found_control_table_section) {
    throw std::runtime_error("Missing [control table] section in " + full_path);
  }
  if (control_table.empty()) {
    throw std::runtime_error("Control table is empty in " + full_path);
  }

  auto result = control_tables_cache_.emplace(model_number, std::move(control_table));
  return result.first->second;
}
}  // namespace dynamixel
