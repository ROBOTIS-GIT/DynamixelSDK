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

/* Author: Ryu Woon Jung (Leon) */

#if defined(__APPLE__)

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#include "port_handler_mac.h"

#define LATENCY_TIMER   16  // msec (USB latency timer)
                            // You should adjust the latency timer value.
                            // When you are going to use sync / bulk read, the latency timer should be loosen.
                            // the lower latency timer value, the faster communication speed.

                            // Note:
                            // You can either change its value by following:
                            // http://www.ftdichip.com/Support/Documents/TechnicalNotes/TN_105%20Adding%20Support%20for%20New%20FTDI%20Devices%20to%20Mac%20Driver.pdf

typedef struct
{
  int     socket_fd;
  int     baudrate;
  char    port_name[100];

  double  packet_start_time;
  double  packet_timeout;
  double  tx_time_per_byte;
}PortData;

static PortData *portData;

int portHandlerMac(const char *port_name)
{
  int port_num;

  if (portData == NULL)
  {
    port_num = 0;
    g_used_port_num = 1;
    portData = (PortData *)calloc(1, sizeof(PortData));
    g_is_using = (uint8_t*)calloc(1, sizeof(uint8_t));
  }
  else
  {
    for (port_num = 0; port_num < g_used_port_num; port_num++)
    {
      if (!strcmp(portData[port_num].port_name, port_name))
        break;
    }

    if (port_num == g_used_port_num)
    {
      for (port_num = 0; port_num < g_used_port_num; port_num++)
      {
        if (portData[port_num].socket_fd != -1)
          break;
      }

      if (port_num == g_used_port_num)
      {
        g_used_port_num++;
        portData = (PortData*)realloc(portData, g_used_port_num * sizeof(PortData));
        g_is_using = (uint8_t*)realloc(g_is_using, g_used_port_num * sizeof(uint8_t));
      }
    }
    else
    {
      printf("[PortHandler setup] The port number %d has same device name... reinitialize port number %d!!\n", port_num, port_num);
    }
  }

  portData[port_num].socket_fd = -1;
  portData[port_num].baudrate = DEFAULT_BAUDRATE;
  portData[port_num].packet_start_time = 0.0;
  portData[port_num].packet_timeout = 0.0;
  portData[port_num].tx_time_per_byte = 0.0;

  g_is_using[port_num] = False;

  setPortNameMac(port_num, port_name);

  return port_num;
}

uint8_t openPortMac(int port_num)
{
  return setBaudRateMac(port_num, portData[port_num].baudrate);
}

void closePortMac(int port_num)
{
  if (portData[port_num].socket_fd != -1)
  {
    close(portData[port_num].socket_fd);
    portData[port_num].socket_fd = -1;
  }
}

void clearPortMac(int port_num)
{
  tcflush(portData[port_num].socket_fd, TCIFLUSH);
}

void setPortNameMac(int port_num, const char *port_name)
{
  strcpy(portData[port_num].port_name, port_name);
}

char *getPortNameMac(int port_num)
{
  return portData[port_num].port_name;
}

uint8_t setBaudRateMac(int port_num, const int baudrate)
{
  int baud = getCFlagBaud(baudrate);

  closePortMac(port_num);

  if (baud <= 0)   // custom baudrate
  {
    setupPortMac(port_num, B38400);
    portData[port_num].baudrate = baudrate;
    return setCustomBaudrateMac(port_num, baudrate);
  }
  else
  {
    portData[port_num].baudrate = baudrate;
    return setupPortMac(port_num, baud);
  }
}

int getBaudRateMac(int port_num)
{
  return portData[port_num].baudrate;
}

int getBytesAvailableMac(int port_num)
{
  int bytes_available;
  ioctl(portData[port_num].socket_fd, FIONREAD, &bytes_available);
  return bytes_available;
}

int readPortMac(int port_num, uint8_t *packet, int length)
{
  return read(portData[port_num].socket_fd, packet, length);
}

int writePortMac(int port_num, uint8_t *packet, int length)
{
  return write(portData[port_num].socket_fd, packet, length);
}

void setPacketTimeoutMac(int port_num, uint16_t packet_length)
{
  portData[port_num].packet_start_time = getCurrentTimeMac();
  portData[port_num].packet_timeout = (portData[port_num].tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void setPacketTimeoutMSecMac(int port_num, double msec)
{
  portData[port_num].packet_start_time = getCurrentTimeMac();
  portData[port_num].packet_timeout = msec;
}

uint8_t isPacketTimeoutMac(int port_num)
{
  if (getTimeSinceStartMac(port_num) > portData[port_num].packet_timeout)
  {
    portData[port_num].packet_timeout = 0;
    return True;
  }
  return False;
}

double getCurrentTimeMac()
{
  struct timespec tv;
#ifdef __MACH__ // OS X does not have clock_gettime, so here uses clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  tv.tv_sec = mts.tv_sec;
  tv.tv_nsec = mts.tv_nsec;
#else
  clock_gettime(CLOCK_REALTIME, &tv);
#endif
  return ((double)tv.tv_sec * 1000.0 + (double)tv.tv_nsec * 0.001 * 0.001);
}

double getTimeSinceStartMac(int port_num)
{
  double time_since_start;

  time_since_start = getCurrentTimeMac() - portData[port_num].packet_start_time;
  if (time_since_start < 0.0)
    portData[port_num].packet_start_time = getCurrentTimeMac();

  return time_since_start;
}

uint8_t setupPortMac(int port_num, int cflag_baud)
{
  struct termios newtio;

  portData[port_num].socket_fd = open(portData[port_num].port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (portData[port_num].socket_fd < 0)
  {
    printf("[PortHandlerMac::SetupPort] Error opening serial port!\n");
    return False;
  }

  bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

  newtio.c_cflag = CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;
  cfsetispeed(&newtio, cflag_baud);
  cfsetospeed(&newtio, cflag_baud);

  // clean the buffer and activate the settings for the port
  tcflush(portData[port_num].socket_fd, TCIFLUSH);
  tcsetattr(portData[port_num].socket_fd, TCSANOW, &newtio);

  portData[port_num].tx_time_per_byte = (1000.0 / (double)portData[port_num].baudrate) * 10.0;
  return True;
}

uint8_t setCustomBaudrateMac(int port_num, int speed)
{
  printf("[PortHandlerMac::SetCustomBaudrate] Not supported on Mac!\n");
  return False;
}

int getCFlagBaud(int baudrate)
{
  switch (baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    // Mac OS doesn't support over B230400
    // case 460800:
    //   return B460800;
    // case 500000:
    //   return B500000;
    // case 576000:
    //   return B576000;
    // case 921600:
    //   return B921600;
    // case 1000000:
    //   return B1000000;
    // case 1152000:
    //   return B1152000;
    // case 1500000:
    //   return B1500000;
    // case 2000000:
    //   return B2000000;
    // case 2500000:
    //   return B2500000;
    // case 3000000:
    //   return B3000000;
    // case 3500000:
    //   return B3500000;
    // case 4000000:
    //   return B4000000;
    default:
      return -1;
  }
}

#endif
