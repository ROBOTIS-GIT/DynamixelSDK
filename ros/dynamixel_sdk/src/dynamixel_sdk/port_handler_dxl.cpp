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

/* Low-latency port handler for dxl-uart kernel driver */

#if defined(__linux__)

#define _GNU_SOURCE /* for ppoll */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>

#include "port_handler_dxl.h"

/* dxl-uart ioctl definitions (must match dxl-uart.h) */
#define DXL_IOC_MAGIC    'D'
#define DXL_SET_BAUDRATE _IOW(DXL_IOC_MAGIC, 1, int)
#define DXL_GET_BAUDRATE _IOR(DXL_IOC_MAGIC, 2, int)

/* No USB latency padding - direct UART, sub-microsecond driver latency */
#define LATENCY_TIMER    0.0

using namespace dynamixel;

PortHandlerDXL::PortHandlerDXL(const char *port_name)
  : socket_fd_(-1),
    baudrate_(57600),
    packet_start_time_(0.0),
    packet_timeout_(0.0),
    tx_time_per_byte_(0.0)
{
  is_using_ = false;
  setPortName(port_name);
}

bool PortHandlerDXL::openPort()
{
  return setBaudRate(baudrate_);
}

void PortHandlerDXL::closePort()
{
  if (socket_fd_ != -1) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

void PortHandlerDXL::clearPort()
{
  if (socket_fd_ != -1) {
    /* TCIFLUSH = 0: flush RX buffer */
    ioctl(socket_fd_, 0x540B /* TCFLSH */, 0);
  }
}

void PortHandlerDXL::setPortName(const char *port_name)
{
  strncpy(port_name_, port_name, sizeof(port_name_) - 1);
  port_name_[sizeof(port_name_) - 1] = '\0';
}

char *PortHandlerDXL::getPortName()
{
  return port_name_;
}

bool PortHandlerDXL::setBaudRate(const int baudrate)
{
  baudrate_ = baudrate;
  /* tx_time_per_byte in milliseconds: (1000 / baud) * 10 bits per byte */
  tx_time_per_byte_ = (1000.0 / (double)baudrate_) * 10.0;

  /* Open device if not already open */
  if (socket_fd_ == -1) {
    socket_fd_ = open(port_name_, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (socket_fd_ < 0) {
      printf("[PortHandlerDXL] Failed to open %s\n", port_name_);
      socket_fd_ = -1;
      return false;
    }
  }

  /* Set baud rate via dxl-uart ioctl */
  int baud = baudrate;
  if (ioctl(socket_fd_, DXL_SET_BAUDRATE, &baud) < 0) {
    printf("[PortHandlerDXL] Failed to set baud rate %d on %s\n",
           baudrate, port_name_);
    return false;
  }

  /* Flush any stale data */
  clearPort();

  return true;
}

int PortHandlerDXL::getBaudRate()
{
  return baudrate_;
}

int PortHandlerDXL::getBytesAvailable()
{
  int bytes_available = 0;

  if (socket_fd_ != -1)
    ioctl(socket_fd_, 0x541B /* FIONREAD */, &bytes_available);

  return bytes_available;
}

int PortHandlerDXL::readPort(uint8_t *packet, int length)
{
  if (socket_fd_ == -1)
    return -1;

  /*
   * Key optimization: instead of the stock SDK's usleep(0) spin loop,
   * we use ppoll() to efficiently wait for data with sub-millisecond
   * precision. The dxl-uart driver's IRQ handler calls
   * wake_up_interruptible() when data arrives, which wakes ppoll()
   * immediately. This replaces 100+ usleep(0) iterations with a
   * single ppoll() call.
   *
   * We use ppoll() instead of poll() because at 6.25Mbps, typical
   * packet timeouts are < 1ms, and poll()'s millisecond granularity
   * would round down to 0 (never waiting).
   */
  double remaining_ms = packet_timeout_ - getTimeSinceStart();
  if (remaining_ms <= 0.0)
    return 0; /* timeout expired, return 0 (SDK calls isPacketTimeout) */

  /* Convert remaining ms to timespec for ppoll (sub-ms precision) */
  long remaining_ns = (long)(remaining_ms * 1000000.0);
  if (remaining_ns < 100000)
    remaining_ns = 100000; /* minimum 100us to avoid busy-loop */

  struct timespec ts;
  ts.tv_sec  = remaining_ns / 1000000000L;
  ts.tv_nsec = remaining_ns % 1000000000L;

  struct pollfd pfd;
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;
  pfd.revents = 0;

  int poll_ret = ppoll(&pfd, 1, &ts, NULL);
  if (poll_ret <= 0)
    return 0; /* timeout or error: return 0 bytes (SDK checks isPacketTimeout) */

  int rx = read(socket_fd_, packet, length);
  if (rx < 0)
    return 0;

  return rx;
}

int PortHandlerDXL::writePort(uint8_t *packet, int length)
{
  if (socket_fd_ == -1)
    return -1;

  return write(socket_fd_, packet, length);
}

void PortHandlerDXL::setPacketTimeout(uint16_t packet_length)
{
  packet_start_time_ = getCurrentTime();
  /*
   * Timeout = wire time + small overhead.
   * No LATENCY_TIMER padding (direct UART, not USB).
   * +0.5ms for IRQ + scheduling overhead on PREEMPT_RT.
   */
  packet_timeout_ = (tx_time_per_byte_ * (double)packet_length) + 0.5;
}

void PortHandlerDXL::setPacketTimeout(double msec)
{
  packet_start_time_ = getCurrentTime();
  packet_timeout_    = msec;
}

bool PortHandlerDXL::isPacketTimeout()
{
  if (getTimeSinceStart() > packet_timeout_) {
    packet_timeout_ = 0;
    return true;
  }
  return false;
}

double PortHandlerDXL::getCurrentTime()
{
  struct timespec tv;
  clock_gettime(CLOCK_MONOTONIC, &tv);
  return ((double)tv.tv_sec * 1000.0 + (double)tv.tv_nsec * 0.000001);
}

double PortHandlerDXL::getTimeSinceStart()
{
  double elapsed = getCurrentTime() - packet_start_time_;
  if (elapsed < 0.0)
    return 0.0;
  return elapsed;
}

#endif /* __linux__ */
