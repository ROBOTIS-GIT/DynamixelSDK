/*******************************************************************************
* Copyright 2020 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 * 
 * Open terminal #1
 * $ roscore
 * 
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples read_write_node
 * 
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
 * $ rosservice call /get_position "id: 1"
 * 
 * Author: Zerom
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/GetPosition.h"
#include "dynamixel_sdk_examples/SetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define DXL2_ID               2               // DXL2 ID
#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler *portHandler;
PacketHandler *packetHandler;

bool getPresentPosition(dynamixel_sdk_examples::GetPosition::Request &req,
                        dynamixel_sdk_examples::GetPosition::Response &res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  
  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int32_t pos = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, (uint8_t)req.id, ADDR_PRESENT_POSITION, (uint32_t*)&pos, &dxl_error);
  ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", req.id, pos);

  res.position = pos;
  return true;
}

void setPositionCallback(const dynamixel_sdk_examples::SetPosition::ConstPtr &msg)
{
  uint8_t dxl_error = 0;
  
  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t pos = (unsigned int)msg->position; // Convert int32 -> uint32

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  packetHandler->write4ByteTxRx(portHandler, (uint8_t)msg->id, ADDR_GOAL_POSITION, pos, &dxl_error);
  ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id, msg->position);
}

int main(int argc, char **argv)
{
  uint8_t dxl_error = 0;
  ros::init(argc, argv, "read_write_node");
  ros::NodeHandle nh;

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if(portHandler->openPort() == false)
  {
    ROS_ERROR("Failed to open the port!");
  }

  if(portHandler->setBaudRate(BAUDRATE) == false)
  {
    ROS_ERROR("Failed to set the baudrate!");
  }

  ros::Subscriber set_position_sub = nh.subscribe("/set_position", 10, setPositionCallback);
  ros::ServiceServer get_position_srv = nh.advertiseService("/get_position", getPresentPosition);

  packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);

  while (ros::ok())
  {
    usleep(8 * 1000);

    ros::spin();
  }

  return 0;
}
