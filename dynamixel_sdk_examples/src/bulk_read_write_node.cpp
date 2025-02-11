// Copyright 2021 ROBOTIS CO., LTD.
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

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples bulk_read_write_node
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /bulk_set_item dynamixel_sdk_examples/BulkSetItem "{id1: 1, id2: 2, item1: 'position', item2: 'LED', value1: 1000, value2: 1}"
 * $ rostopic pub -1 /bulk_set_item dynamixel_sdk_examples/BulkSetItem "{id1: 1, id2: 2, item1: 'LED', item2: 'position', value1: 1, value2: 1000}"
 * $ rosservice call /bulk_get_item "{id1: 1, id2: 2, item1: 'position', item2: 'LED'}"
 *
 * Author: Jaehyun Shim
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/BulkGetItem.h"
#include "dynamixel_sdk_examples/BulkSetItem.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_PRESENT_LED      65
#define ADDR_PRESENT_POSITION 132
#define ADDR_GOAL_POSITION    116

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define DXL2_ID               2               // DXL2 ID
#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler * packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

GroupBulkRead groupBulkRead(portHandler, packetHandler);
GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

bool bulkGetItemCallback(
  dynamixel_sdk_examples::BulkGetItem::Request & req,
  dynamixel_sdk_examples::BulkGetItem::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int32_t position1 = 0;
  int32_t position2 = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  if (req.item1 == "position") {
    dxl_addparam_result = groupBulkRead.addParam((uint8_t)req.id1, ADDR_PRESENT_POSITION, 4);
  } else if (req.item1 == "LED") {
    dxl_addparam_result = groupBulkRead.addParam((uint8_t)req.id1, ADDR_PRESENT_LED, 1);
  }
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupBulkRead for Dynamixel ID: %d", req.id1);
    return 0;
  }

  if (req.item2 == "position") {
    dxl_addparam_result = groupBulkRead.addParam((uint8_t)req.id2, ADDR_PRESENT_POSITION, 4);
  } else if (req.item2 == "LED") {
    dxl_addparam_result = groupBulkRead.addParam((uint8_t)req.id2, ADDR_PRESENT_LED, 1);
  }
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupBulkRead for Dynamixel ID %d", req.id2);
    return 0;
  }

  uint32_t value1 = 0;
  uint32_t value2 = 0;
  dxl_comm_result = groupBulkRead.txRxPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    if (req.item1 == "position") {
      value1 = groupBulkRead.getData((uint8_t)req.id1, ADDR_PRESENT_POSITION, 4);
    } else if (req.item2 == "LED") {
      value1 = groupBulkRead.getData((uint8_t)req.id1, ADDR_PRESENT_POSITION, 4);
    }

    if (req.item1 == "position") {
      value2 = groupBulkRead.getData((uint8_t)req.id2, ADDR_PRESENT_POSITION, 4);
    } else if (req.item2 == "LED") {
      value2 = groupBulkRead.getData((uint8_t)req.id2, ADDR_PRESENT_POSITION, 4);
    }

    ROS_INFO("getItem : [ID:%d] [%s: %d]", req.id1, req.item1.c_str(), value1);
    ROS_INFO("getItem : [ID:%d] [%s: %d]", req.id2, req.item2.c_str(), value2);
    res.value1 = value1;
    res.value2 = value2;
    groupBulkRead.clearParam();
    return true;
  } else {
    ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
    groupBulkRead.clearParam();
    return false;
  }
}

void bulkSetItemCallback(const dynamixel_sdk_examples::BulkSetItem::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  uint8_t param_goal_position1[4];
  uint8_t param_goal_position2[4];
  uint8_t param_goal_led1[1];
  uint8_t param_goal_led2[1];
  uint8_t addr_goal_item[2];
  uint8_t len_goal_item[2];

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  if (msg->item1 == "position") {
    uint32_t position1 = (unsigned int)msg->value1; // Convert int32 -> uint32
    param_goal_position1[0] = DXL_LOBYTE(DXL_LOWORD(position1));
    param_goal_position1[1] = DXL_HIBYTE(DXL_LOWORD(position1));
    param_goal_position1[2] = DXL_LOBYTE(DXL_HIWORD(position1));
    param_goal_position1[3] = DXL_HIBYTE(DXL_HIWORD(position1));
    addr_goal_item[0] = ADDR_GOAL_POSITION;
    len_goal_item[0] = 4;
  } else if (msg->item1 == "LED") {
    uint32_t led1 = (unsigned int)msg->value1; // Convert int32 -> uint32
    param_goal_led1[0] = led1;
    addr_goal_item[0] = ADDR_PRESENT_LED;
    len_goal_item[0] = 1;
  }

  if (msg->item2 == "position") {
    uint32_t position2 = (unsigned int)msg->value2; // Convert int32 -> uint32
    param_goal_position2[0] = DXL_LOBYTE(DXL_LOWORD(position2));
    param_goal_position2[1] = DXL_HIBYTE(DXL_LOWORD(position2));
    param_goal_position2[2] = DXL_LOBYTE(DXL_HIWORD(position2));
    param_goal_position2[3] = DXL_HIBYTE(DXL_HIWORD(position2));
    addr_goal_item[1] = ADDR_GOAL_POSITION;
    len_goal_item[1] = 4;
  } else if (msg->item2 == "LED") {
    uint32_t led2 = (unsigned int)msg->value2; // Convert int32 -> uint32
    param_goal_led2[0] = led2;
    addr_goal_item[1] = ADDR_PRESENT_LED;
    len_goal_item[1] = 1;
  }

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  if (msg->item1 == "position") {
    dxl_addparam_result = groupBulkWrite.addParam((uint8_t)msg->id1, addr_goal_item[0], len_goal_item[0], param_goal_position1);
  } else if (msg->item1 == "LED") {
    dxl_addparam_result = groupBulkWrite.addParam((uint8_t)msg->id1, addr_goal_item[0], len_goal_item[0], param_goal_led1);
  }
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupBulkWrite for Dynamixel ID: %d", msg->id1);
  }

  if (msg->item2 == "position") {
    dxl_addparam_result = groupBulkWrite.addParam((uint8_t)msg->id2, addr_goal_item[1], len_goal_item[1], param_goal_position2);
  } else if (msg->item2 == "LED") {
    dxl_addparam_result = groupBulkWrite.addParam((uint8_t)msg->id2, addr_goal_item[1], len_goal_item[1], param_goal_led2);
  }
  if (dxl_addparam_result != true) {
    ROS_ERROR("Failed to addparam to groupBulkWrite for Dynamixel ID: %d", msg->id2);
  }

  dxl_comm_result = groupBulkWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setItem : [ID:%d] [%s:%d]", msg->id1, msg->item1.c_str(), msg->value1);
    ROS_INFO("setItem : [ID:%d] [%s:%d]", msg->id2, msg->item2.c_str(), msg->value2);
  } else {
    ROS_INFO("Failed to set position! Result: %d", dxl_comm_result);
  }

  groupBulkWrite.clearParam();
}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID: %d", DXL1_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID: %d", DXL2_ID);
    return -1;
  }

  ros::init(argc, argv, "bulk_read_write_node");
  ros::NodeHandle nh;
  ros::ServiceServer bulk_get_item_srv = nh.advertiseService("/bulk_get_item", bulkGetItemCallback);
  ros::Subscriber bulk_set_item_sub = nh.subscribe("/bulk_set_item", 10, bulkSetItemCallback);
  ros::spin();

  portHandler->closePort();
  return 0;
}
