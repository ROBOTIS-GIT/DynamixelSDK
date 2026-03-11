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

#if defined(__linux__)

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "port_handler_rp1ctrluart_linux.h"

using namespace dynamixel;

namespace
{
constexpr int kLatencyTimerMsec = 16;
constexpr char kRp1CtrlUartPrefix[] = "/dev/rp1ctrluart";
}

PortHandlerRp1CtrlUartLinux::PortHandlerRp1CtrlUartLinux(const char *port_name)
  : socket_fd_(-1),
    baudrate_(DEFAULT_BAUDRATE_),
    fixed_baudrate_(-1),
    packet_start_time_(0.0),
    packet_timeout_(0.0),
    tx_time_per_byte_(0.0)
{
  is_using_ = false;
  setPortName(port_name);
}

int PortHandlerRp1CtrlUartLinux::inferFixedBaudrate(const char *port_name) const
{
  const size_t prefix_len = sizeof(kRp1CtrlUartPrefix) - 1;

  if (strncmp(port_name, kRp1CtrlUartPrefix, prefix_len) != 0)
    return -1;

  int alias_id = atoi(port_name + prefix_len);
  switch (alias_id)
  {
    case 2:
      return 6250000;
    case 4:
      return 4000000;
    default:
      return -1;
  }
}

bool PortHandlerRp1CtrlUartLinux::openFileDescriptor()
{
  if (socket_fd_ >= 0)
    return true;

  socket_fd_ = open(port_name_, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (socket_fd_ < 0)
  {
    printf("[PortHandlerRp1CtrlUartLinux::openFileDescriptor] Error opening %s: %s\n",
           port_name_, strerror(errno));
    return false;
  }
  return true;
}

bool PortHandlerRp1CtrlUartLinux::openPort()
{
  if (!openFileDescriptor())
    return false;

  if (fixed_baudrate_ > 0)
  {
    baudrate_ = fixed_baudrate_;
    tx_time_per_byte_ = (1000.0 / static_cast<double>(baudrate_)) * 10.0;
  }

  return true;
}

void PortHandlerRp1CtrlUartLinux::closePort()
{
  if (socket_fd_ != -1)
    close(socket_fd_);

  socket_fd_ = -1;
  is_using_ = false;
}

void PortHandlerRp1CtrlUartLinux::clearPort()
{
  uint8_t drain[256];

  if (socket_fd_ < 0)
    return;

  while (read(socket_fd_, drain, sizeof(drain)) > 0)
  {
  }
}

void PortHandlerRp1CtrlUartLinux::setPortName(const char *port_name)
{
  strcpy(port_name_, port_name);
  fixed_baudrate_ = inferFixedBaudrate(port_name_);
}

char *PortHandlerRp1CtrlUartLinux::getPortName()
{
  return port_name_;
}

bool PortHandlerRp1CtrlUartLinux::setBaudRate(const int baudrate)
{
  if (!openFileDescriptor())
    return false;

  if (fixed_baudrate_ > 0 && baudrate != fixed_baudrate_)
  {
    printf("[PortHandlerRp1CtrlUartLinux::setBaudRate] %s is fixed at %d, requested %d\n",
           port_name_, fixed_baudrate_, baudrate);
    return false;
  }

  baudrate_ = fixed_baudrate_ > 0 ? fixed_baudrate_ : baudrate;
  tx_time_per_byte_ = (1000.0 / static_cast<double>(baudrate_)) * 10.0;
  return true;
}

int PortHandlerRp1CtrlUartLinux::getBaudRate()
{
  return baudrate_;
}

int PortHandlerRp1CtrlUartLinux::getBytesAvailable()
{
  struct pollfd pfd;

  if (socket_fd_ < 0)
    return 0;

  memset(&pfd, 0, sizeof(pfd));
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;

  return poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN) ? 1 : 0;
}

int PortHandlerRp1CtrlUartLinux::readPort(uint8_t *packet, int length)
{
  return read(socket_fd_, packet, length);
}

int PortHandlerRp1CtrlUartLinux::writePort(uint8_t *packet, int length)
{
  return write(socket_fd_, packet, length);
}

void PortHandlerRp1CtrlUartLinux::setPacketTimeout(uint16_t packet_length)
{
  packet_start_time_ = getCurrentTime();
  packet_timeout_ = (tx_time_per_byte_ * static_cast<double>(packet_length)) +
                    (kLatencyTimerMsec * 2.0) + 2.0;
}

void PortHandlerRp1CtrlUartLinux::setPacketTimeout(double msec)
{
  packet_start_time_ = getCurrentTime();
  packet_timeout_ = msec;
}

bool PortHandlerRp1CtrlUartLinux::isPacketTimeout()
{
  if (getTimeSinceStart() > packet_timeout_)
  {
    packet_timeout_ = 0;
    return true;
  }

  return false;
}

double PortHandlerRp1CtrlUartLinux::getCurrentTime()
{
  struct timespec tv;

  clock_gettime(CLOCK_REALTIME, &tv);
  return ((double)tv.tv_sec * 1000.0 + (double)tv.tv_nsec * 0.001 * 0.001);
}

double PortHandlerRp1CtrlUartLinux::getTimeSinceStart()
{
  double time = getCurrentTime() - packet_start_time_;

  if (time < 0.0)
    packet_start_time_ = getCurrentTime();

  return time;
}

#endif
