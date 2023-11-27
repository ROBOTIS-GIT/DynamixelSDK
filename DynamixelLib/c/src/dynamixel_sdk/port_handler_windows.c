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

#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "port_handler_windows.h"

#define LATENCY_TIMER  16 // msec (USB latency timer)
                          // You should adjust the latency timer value. In Windows, the default latency timer of the usb serial is '16 msec'.
                          // When you are going to use sync / bulk read, the latency timer should be loosen.
                          // the lower latency timer value, the faster communication speed.

                          // Note:
                          // You can either checking or changing its value by:
                          // [Device Manager] -> [Port (COM & LPT)] -> the port you use but starts with COMx-> mouse right click -> properties
                          // -> [port settings] -> [details] -> change response time from 16 to the value you need

typedef struct
{
  HANDLE  serial_handle;
  LARGE_INTEGER freq, counter;

  int     baudrate;
  char    port_name[100];

  double  packet_start_time;
  double  packet_timeout;
  double  tx_time_per_byte;
}PortData;

static PortData *portData;

int portHandlerWindows(const char *port_name)
{
  int port_num;
  char buffer[15];

  sprintf_s(buffer, sizeof(buffer), "\\\\.\\%s", port_name);

  if (portData == NULL)
  {
    port_num = 0;
    g_used_port_num = 1;
    portData = (PortData*)calloc(1, sizeof(PortData));
    g_is_using = (uint8_t*)calloc(1, sizeof(uint8_t));
  }
  else
  {
    for (port_num = 0; port_num < g_used_port_num; port_num++)
    {
      if (!strcmp(portData[port_num].port_name, buffer))
        break;
    }

    if (port_num == g_used_port_num)
    {
      for (port_num = 0; port_num < g_used_port_num; port_num++)
      {
        if (portData[port_num].serial_handle != INVALID_HANDLE_VALUE)
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

  portData[port_num].serial_handle = INVALID_HANDLE_VALUE;
  portData[port_num].baudrate = DEFAULT_BAUDRATE;
  portData[port_num].packet_start_time = 0.0;
  portData[port_num].packet_timeout = 0.0;
  portData[port_num].tx_time_per_byte = 0.0;

  g_is_using[port_num] = False;

  setPortNameWindows(port_num, buffer);

  return port_num;
}

uint8_t openPortWindows(int port_num)
{
  return setBaudRateWindows(port_num, portData[port_num].baudrate);
}

void closePortWindows(int port_num)
{
  if (portData[port_num].serial_handle != INVALID_HANDLE_VALUE)
  {
    CloseHandle(portData[port_num].serial_handle);
    portData[port_num].serial_handle = INVALID_HANDLE_VALUE;
  }
}

void clearPortWindows(int port_num)
{
  PurgeComm(portData[port_num].serial_handle, PURGE_RXABORT | PURGE_RXCLEAR);
}

void setPortNameWindows(int port_num, const char *port_name)
{
  strcpy_s(portData[port_num].port_name, sizeof(portData[port_num].port_name), port_name);
}

char *getPortNameWindows(int port_num)
{
  return portData[port_num].port_name;
}

uint8_t setBaudRateWindows(int port_num, const int baudrate)
{
  closePortWindows(port_num);

  portData[port_num].baudrate = baudrate;
  return setupPortWindows(port_num, baudrate);
}

int getBaudRateWindows(int port_num)
{
  return portData[port_num].baudrate;
}

int readPortWindows(int port_num, uint8_t *packet, int length)
{
  DWORD dwRead = 0;

  if (ReadFile(portData[port_num].serial_handle, packet, (DWORD)length, &dwRead, NULL) == FALSE)
    return -1;

  return (int)dwRead;
}

int writePortWindows(int port_num, uint8_t *packet, int length)
{
  DWORD dwWrite = 0;

  if (WriteFile(portData[port_num].serial_handle, packet, (DWORD)length, &dwWrite, NULL) == FALSE)
    return -1;

  return (int)dwWrite;
}

void setPacketTimeoutWindows(int port_num, uint16_t packet_length)
{
  portData[port_num].packet_start_time = getCurrentTimeWindows(port_num);
  portData[port_num].packet_timeout = (portData[port_num].tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void setPacketTimeoutMSecWindows(int port_num, double msec)
{
  portData[port_num].packet_start_time = getCurrentTimeWindows(port_num);
  portData[port_num].packet_timeout = msec;
}

uint8_t isPacketTimeoutWindows(int port_num)
{
  if (getTimeSinceStartWindows(port_num) > portData[port_num].packet_timeout)
  {
    portData[port_num].packet_timeout = 0;
    return True;
  }
  return False;
}

double getCurrentTimeWindows(int port_num)
{
  QueryPerformanceCounter(&portData[port_num].counter);
  QueryPerformanceFrequency(&portData[port_num].freq);
  return (double)portData[port_num].counter.QuadPart / (double)portData[port_num].freq.QuadPart * 1000.0;
}

double getTimeSinceStartWindows(int port_num)
{
  double time_since_start;

  time_since_start = getCurrentTimeWindows(port_num) - portData[port_num].packet_start_time;
  if (time_since_start < 0.0)
    portData[port_num].packet_start_time = getCurrentTimeWindows(port_num);

  return time_since_start;
}

uint8_t setupPortWindows(int port_num, const int baudrate)
{
  DCB dcb;
  COMMTIMEOUTS timeouts;
  DWORD dwError;

  closePortWindows(port_num);

  portData[port_num].serial_handle = CreateFileA(portData[port_num].port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
  if (portData[port_num].serial_handle == INVALID_HANDLE_VALUE)
  {
    printf("[PortHandlerWindows::SetupPort] Error opening serial port!\n");
    return False;
  }

  dcb.DCBlength = sizeof(DCB);
  if (GetCommState(portData[port_num].serial_handle, &dcb) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  // Set baudrate
  dcb.BaudRate = (DWORD)baudrate;
  dcb.ByteSize = 8;                    // Data bit = 8bit
  dcb.Parity = NOPARITY;             // No parity
  dcb.StopBits = ONESTOPBIT;           // Stop bit = 1
  dcb.fParity = NOPARITY;             // No Parity check
  dcb.fBinary = 1;                    // Binary mode
  dcb.fNull = 0;                    // Get Null byte
  dcb.fAbortOnError = 0;
  dcb.fErrorChar = 0;
  // Not using XOn/XOff
  dcb.fOutX = 0;
  dcb.fInX = 0;
  // Not using H/W flow control
  dcb.fDtrControl = DTR_CONTROL_DISABLE;
  dcb.fRtsControl = RTS_CONTROL_DISABLE;
  dcb.fDsrSensitivity = 0;
  dcb.fOutxDsrFlow = 0;
  dcb.fOutxCtsFlow = 0;

  if (SetCommState(portData[port_num].serial_handle, &dcb) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  if (SetCommMask(portData[port_num].serial_handle, 0) == FALSE) // Not using Comm event
    goto DXL_HAL_OPEN_ERROR;
  if (SetupComm(portData[port_num].serial_handle, 4096, 4096) == FALSE) // Buffer size (Rx,Tx)
    goto DXL_HAL_OPEN_ERROR;
  if (PurgeComm(portData[port_num].serial_handle, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR) == FALSE) // Clear buffer
    goto DXL_HAL_OPEN_ERROR;
  if (ClearCommError(portData[port_num].serial_handle, &dwError, NULL) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  if (GetCommTimeouts(portData[port_num].serial_handle, &timeouts) == FALSE)
    goto DXL_HAL_OPEN_ERROR;
  // Timeout (Not using timeout)
  // Immediatly return
  timeouts.ReadIntervalTimeout = 0;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.ReadTotalTimeoutConstant = 1; // must not be zero.
  timeouts.WriteTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 0;
  if (SetCommTimeouts(portData[port_num].serial_handle, &timeouts) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  portData[port_num].tx_time_per_byte = (1000.0 / (double)portData[port_num].baudrate) * 10.0;
  return True;

  DXL_HAL_OPEN_ERROR:
    closePortWindows(port_num);

  return False;
}

#endif
