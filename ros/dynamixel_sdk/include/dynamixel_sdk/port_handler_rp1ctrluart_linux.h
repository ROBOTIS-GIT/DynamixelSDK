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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLER_RP1CTRLUART_LINUX_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLER_RP1CTRLUART_LINUX_H_

#include "port_handler.h"

namespace dynamixel
{

class PortHandlerRp1CtrlUartLinux : public PortHandler
{
 private:
  int socket_fd_;
  int baudrate_;
  int fixed_baudrate_;
  char port_name_[100];

  double packet_start_time_;
  double packet_timeout_;
  double tx_time_per_byte_;

  bool openFileDescriptor();
  int inferFixedBaudrate(const char *port_name) const;
  double getCurrentTime();
  double getTimeSinceStart();

 public:
  explicit PortHandlerRp1CtrlUartLinux(const char *port_name);
  virtual ~PortHandlerRp1CtrlUartLinux() { closePort(); }

  bool openPort();
  void closePort();
  void clearPort();
  void setPortName(const char *port_name);
  char *getPortName();
  bool setBaudRate(const int baudrate);
  int getBaudRate();
  int getBytesAvailable();
  int readPort(uint8_t *packet, int length);
  int writePort(uint8_t *packet, int length);
  void setPacketTimeout(uint16_t packet_length);
  void setPacketTimeout(double msec);
  bool isPacketTimeout();
};

}

#endif
