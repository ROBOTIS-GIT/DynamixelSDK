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

#include "dynamixel_easy_sdk/control_table.hpp"
#include "dynamixel_easy_sdk/dynamixel_error.hpp"

#ifdef ROS_BUILD
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <vector>
#include <cstdlib>

namespace dynamixel
{

std::string ControlTable::model_dir_;
std::map<int, std::map<std::string, ControlTableItem>> ControlTable::model_cache_;
std::map<int, std::string> ControlTable::model_name_cache_;
std::mutex ControlTable::cache_mutex_;

void ControlTable::setModelDirectory(const std::string& path)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  model_dir_ = path;
}

std::string ControlTable::findModelFile(int model_number)
{
  std::vector<std::filesystem::path> search_paths;

  // 1. Explicitly set path
  if (!model_dir_.empty()) {
    search_paths.push_back(std::filesystem::path(model_dir_));
  }

  // 2. Environment Variable
  const char* env_path = std::getenv("DXL_MODEL_PATH");
  if (env_path) {
    search_paths.push_back(std::filesystem::path(env_path));
  }

  // 3. Compile-time Macro
#ifdef CONTROL_TABLE_PATH
  search_paths.push_back(std::filesystem::path(CONTROL_TABLE_PATH));
#endif

  // 3.1 ROS 2 Ament Index
#ifdef ROS_BUILD
  try {
    std::string share_dir = ament_index_cpp::get_package_share_directory("dynamixel_sdk");
    search_paths.push_back(std::filesystem::path(share_dir) / "control_table");
  } catch (...) {
      // Ignore if package not found (fallback to others)
  }
#endif

  // 4. Current Directory
  search_paths.push_back(std::filesystem::current_path());

  // 5. Common Install Locations (Fallback)
  search_paths.push_back(std::filesystem::path("/usr/local/share/dynamixel_sdk/control_table"));

  for (const auto& base_path : search_paths) {
    if (std::filesystem::exists(base_path / "dynamixel.model")) {
      return (base_path / "dynamixel.model").string();
    }
  }

  return "";
}

bool ControlTable::loadModel(int model_number)
{
  // 1. Find dynamixel.model to get the model name
  std::string index_file_path = findModelFile(model_number);
  if (index_file_path.empty()) {
    return false;
  }

  std::filesystem::path base_dir = std::filesystem::path(index_file_path).parent_path();
  std::string model_name;
  
  {
    std::ifstream infile(index_file_path);
    if (!infile.is_open()) return false;

    std::string line;
    while (std::getline(infile, line)) {
      if (line.empty()) continue;
      std::stringstream ss(line);
      int header_model_num;
      std::string name;
      
      if (ss >> header_model_num) {
        std::getline(ss, name);
        // Trim whitespace and potential CR
        const auto strBegin = name.find_first_not_of(" \t");
        if (strBegin != std::string::npos) {
          const auto strEnd = name.find_last_not_of(" \t\r");
          name = name.substr(strBegin, strEnd - strBegin + 1);
        } else {
          name = "";
        }

        if (header_model_num == model_number && !name.empty()) {
            model_name = name;
            break;
        }
      }
    }
  }

  if (model_name.empty()) return false;
  model_name_cache_[model_number] = model_name;

  // 2. Load the specific model file (e.g., mx-28.model)
  std::filesystem::path model_file_path = base_dir / model_name;
  std::ifstream model_file(model_file_path);
  if (!model_file.is_open()) return false;

  std::map<std::string, ControlTableItem> table;
  std::string line;
  bool in_control_table = false;

  while (std::getline(model_file, line)) {
    if (line.empty()) continue;
    if (line.find("[control table]") != std::string::npos) {
      in_control_table = true;
      // Skip header line
      std::getline(model_file, line); 
      continue;
    }

    if (in_control_table) {
      if (line.front() == '[') break; // Next section

      std::stringstream ss(line);
      int item_addr, item_size;
      std::string name;
      // Format: Address Size Data Name (Whitespace separated)
      if (ss >> item_addr >> item_size) {
        std::getline(ss, name);

        // Trim whitespace and potential CR
        const auto strBegin = name.find_first_not_of(" \t");
        if (strBegin != std::string::npos) {
          const auto strEnd = name.find_last_not_of(" \t\r");
          name = name.substr(strBegin, strEnd - strBegin + 1);
        } else {
          continue; 
        }

        try {
          ControlTableItem item;
          item.address = static_cast<uint16_t>(item_addr);
          item.size = static_cast<uint8_t>(item_size);
          table[name] = item;
        } catch (...) {}
      }
    }
  }

  model_cache_[model_number] = table;
  return true;
}

std::optional<ControlTableItem> ControlTable::getItemInfo(int model_number, std::string_view item_name)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  
  if (model_cache_.find(model_number) == model_cache_.end()) {
    if (!loadModel(model_number)) {
      return std::nullopt;
    }
  }

  const auto& table = model_cache_[model_number];
  
  // 1. Direct match
  auto it = table.find(std::string(item_name));
  if (it != table.end()) {
    return it->second;
  }

  // 2. Try replacing underscores with spaces (common convention mismatch)
  std::string alt_name(item_name);
  std::replace(alt_name.begin(), alt_name.end(), '_', ' ');
  auto it_alt = table.find(alt_name);
  if (it_alt != table.end()) {
    return it_alt->second;
  }

  // 3. Case-Insensitive Search (Final Fallback)
  // Convert input to lower for comparison (optimization: do this once if map keys were normalized, but here we traverse)
  // Actually, standard iteration
  auto it_ci = std::find_if(table.begin(), table.end(), [&](const auto& pair) {
    const std::string& key = pair.first;
    if (key.size() != item_name.size()) return false;
    return std::equal(key.begin(), key.end(), item_name.begin(),
                      [](unsigned char a, unsigned char b) {
                          return std::tolower(a) == std::tolower(b);
                      });
  });

  if (it_ci != table.end()) {
    return it_ci->second;
  }
  
  // 4. Underscore + Case-Insensitive (e.g. "torque_enable" input vs "Torque Enable" key)
  // The input "torque_enable" with step 2 became "torque enable" (alt_name).
  // Now check if "torque enable" matches any key case-insensitively.
  auto it_alt_ci = std::find_if(table.begin(), table.end(), [&](const auto& pair) {
    const std::string& key = pair.first;
    if (key.size() != alt_name.size()) return false;
    return std::equal(key.begin(), key.end(), alt_name.begin(),
                      [](unsigned char a, unsigned char b) {
                          return std::tolower(a) == std::tolower(b);
                      });
  });

  if (it_alt_ci != table.end()) {
    return it_alt_ci->second;
  }

  return std::nullopt;
}

const std::map<std::string, ControlTableItem>* ControlTable::getModelTable(int model_number)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (model_cache_.find(model_number) == model_cache_.end()) {
    if (!loadModel(model_number)) {
      return nullptr;
    }
  }
  return &model_cache_[model_number];
}

std::string ControlTable::getModelName(int model_number)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (model_name_cache_.find(model_number) == model_name_cache_.end()) {
    if (!loadModel(model_number)) {
      return "Unknown";
    }
  }
  return model_name_cache_[model_number]; 
}

const std::map<std::string, ControlTableItem>& ControlTable::getControlTable(int model_number)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (model_cache_.find(model_number) == model_cache_.end()) {
    if (!loadModel(model_number)) {
      throw DxlRuntimeError("Model number " + std::to_string(model_number) + " not found in control table models.");
    }
  }
  return model_cache_[model_number];
}

} // namespace dynamixel
