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

////////////////////////////////////////////////////////////////////////////////
/// @file Port handler for dxl-uart kernel driver (/dev/dxlN)
/// @description Low-latency port handler that uses poll()-based blocking reads
///              instead of the stock SDK's usleep(0) spin loop. Designed for
///              the dxl-uart kernel driver on Raspberry Pi 5.
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLERDXL_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLERDXL_H_

#include "port_handler.h"

namespace dynamixel
{

////////////////////////////////////////////////////////////////////////////////
/// @brief Port handler for the dxl-uart low-latency kernel driver
////////////////////////////////////////////////////////////////////////////////
class PortHandlerDXL : public PortHandler
{
 private:
  int     socket_fd_;
  int     baudrate_;
  char    port_name_[100];

  double  packet_start_time_;
  double  packet_timeout_;
  double  tx_time_per_byte_;

  double  getCurrentTime();
  double  getTimeSinceStart();

 public:
  ////////////////////////////////////////////////////////////////////////////////
  /// @brief Constructor - initializes with device path (e.g. "/dev/dxl0")
  ////////////////////////////////////////////////////////////////////////////////
  PortHandlerDXL(const char *port_name);

  virtual ~PortHandlerDXL() { closePort(); }

  bool    openPort();
  void    closePort();
  void    clearPort();

  void    setPortName(const char *port_name);
  char   *getPortName();

  bool    setBaudRate(const int baudrate);
  int     getBaudRate();

  int     getBytesAvailable();

  int     readPort(uint8_t *packet, int length);
  int     writePort(uint8_t *packet, int length);

  void    setPacketTimeout(uint16_t packet_length);
  void    setPacketTimeout(double msec);
  bool    isPacketTimeout();
};

}

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLERDXL_H_ */
