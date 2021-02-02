/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
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
************************     Read and Write Example      ***********************
* Required Environment to run this example :
*   - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
*   - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
* How to use the example :
*   - Use proper DYNAMIXEL Model definition from line #44
*   - Build and Run from proper architecture subdirectory.
*   - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
*   - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
* Author: Ryu Woon Jung (Leon)
* Maintainer : Zerom, Will Son
*******************************************************************************/

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"  // Uses DYNAMIXEL SDK library

/********* DYNAMIXEL Model definition *********
***** (Use only one definition at a time) *****/
#define X_SERIES // X330, X430, X540, 2X430
// #define PRO_SERIES // H54, H42, M54, M42, L54, L42
// #define PRO_A_SERIES // PRO series with (A) firmware update.
// #define P_SERIES  // PH54, PH42, PM54
// #define XL320  // [WARNING] Operating Voltage : 7.4V
// #define MX_SERIES // MX series with 2.0 firmware update.

// Control table address
#if defined(X_SERIES) || defined(MX_SERIES)
  #define ADDR_TORQUE_ENABLE          64
  #define ADDR_GOAL_POSITION          116
  #define ADDR_PRESENT_POSITION       132
  #define MINIMUM_POSITION_LIMIT      0  // Refer to the Minimum Position Limit of product eManual
  #define MAXIMUM_POSITION_LIMIT      4095  // Refer to the Maximum Position Limit of product eManual
  #define BAUDRATE                    57600
#elif defined(PRO_SERIES)
  #define ADDR_TORQUE_ENABLE          562  // Control table address is different in DYNAMIXEL model
  #define ADDR_GOAL_POSITION          596
  #define ADDR_PRESENT_POSITION       611
  #define MINIMUM_POSITION_LIMIT      -150000  // Refer to the Minimum Position Limit of product eManual
  #define MAXIMUM_POSITION_LIMIT      150000  // Refer to the Maximum Position Limit of product eManual
  #define BAUDRATE                    57600
#elif defined(P_SERIES) ||defined(PRO_A_SERIES)
  #define ADDR_TORQUE_ENABLE          512  // Control table address is different in DYNAMIXEL model
  #define ADDR_GOAL_POSITION          564
  #define ADDR_PRESENT_POSITION       580
  #define MINIMUM_POSITION_LIMIT      -150000  // Refer to the Minimum Position Limit of product eManual
  #define MAXIMUM_POSITION_LIMIT      150000  // Refer to the Maximum Position Limit of product eManual
  #define BAUDRATE                    57600
#elif defined(XL320)
  #define ADDR_TORQUE_ENABLE          24
  #define ADDR_GOAL_POSITION          30
  #define ADDR_PRESENT_POSITION       37
  #define MINIMUM_POSITION_LIMIT      0  // Refer to the CW Angle Limit of product eManual
  #define MAXIMUM_POSITION_LIMIT      1023  // Refer to the CCW Angle Limit of product eManual
  #define BAUDRATE                    1000000  // Default Baudrate of XL-320 is 1Mbps
#endif

// DYNAMIXEL Protocol Version (1.0 / 2.0)
// https://emanual.robotis.com/docs/en/dxl/protocol2/
#define PROTOCOL_VERSION  2.0

// Factory default ID of all DYNAMIXEL is 1
#define DXL_ID  1

// Use the actual port assigned to the U2D2.
// ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
#define DEVICENAME  "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MOVING_STATUS_THRESHOLD     20  // DYNAMIXEL moving status threshold
#define ESC_ASCII_VALUE                 0x1b

int getch() {
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void) {
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main() {
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = {MINIMUM_POSITION_LIMIT, MAXIMUM_POSITION_LIMIT};         // Goal position

  uint8_t dxl_error = 0;                          // DYNAMIXEL error
  #if defined(XL320)
  int16_t dxl_present_position = 0;  // XL-320 uses 2 byte Position data
  #else
  int32_t dxl_present_position = 0;  // Read 4 byte Position data
  #endif

  // Open port
  if (portHandler->openPort()) {
    printf("Succeeded to open the port!\n");
  }
  else {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n");
  }
  else {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable DYNAMIXEL Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else {
    printf("Succeeded enabling DYNAMIXEL Torque.\n");
  }

  while(1) {
    printf("Press any key to continue. (Press [ESC] to exit)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Write goal position
    #if defined(XL320)  // XL-320 uses 2 byte Position data
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
    #else
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
    #endif
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    do {
      // Read the Present Position
      #if defined(XL320)  // XL-320 uses 2 byte Position data
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);
      #else
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      #endif
      if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      }

      printf("[ID:%03d] Goal Position:%03d  Present Position:%03d\n", DXL_ID, dxl_goal_position[index], dxl_present_position);

    } while((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    // Switch the Goal Position
    if (index == 0) {
      index = 1;
    }
    else {
      index = 0;
    }
  }

  // Disable DYNAMIXEL Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else {
    printf("Succeeded disabling DYNAMIXEL Torque.\n");
  }

  // Close port
  portHandler->closePort();
  return 0;
}
