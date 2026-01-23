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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_CONTROL_TABLE_HPP_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_CONTROL_TABLE_HPP_

#include <map>
#include <string>
#include <string_view>
#include <mutex>
#include <optional>
#include <filesystem>
#include <vector>

#include "dynamixel_easy_sdk/data_types.hpp"

namespace dynamixel
{

class ControlTable
{
public:
  // Thread-safe initialization of model path
  static void setModelDirectory(const std::string& path);
  
  // Get item info (Thread-safe, Cached)
  static std::optional<ControlTableItem> getItemInfo(int model_number, std::string_view item_name);

  // Get full table Pointer (Legacy/internal use)
  static const std::map<std::string, ControlTableItem>* getModelTable(int model_number);

  /**
   * @brief Get the Model Name object
   * 
   * @param model_number 
   * @return std::string (e.g., "XM430-W210")
   */
  static std::string getModelName(int model_number);

  /**
   * @brief Get the Control Table object (Reference)
   * 
   * @param model_number 
   * @return const std::map<std::string, ControlTableItem>& 
   * @throws std::runtime_error if model not found
   */
  static const std::map<std::string, ControlTableItem>& getControlTable(int model_number);

private:
  static std::string model_dir_;
  
  static std::map<int, std::map<std::string, ControlTableItem>> model_cache_;
  static std::map<int, std::string> model_name_cache_;
  
  static std::mutex cache_mutex_;

  static bool loadModel(int model_number);
  static std::string findModelFile(int model_number);
};

} // namespace dynamixel

#endif // DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_EASY_SDK_CONTROL_TABLE_HPP_
