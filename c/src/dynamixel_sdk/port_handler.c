/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#endif

#include "dynamixel_sdk/port_handler.h"

#if defined(__linux__) || defined(__APPLE__)
#include "dynamixel_sdk_linux/port_handler_linux.h"
#endif

#if defined(_WIN32) || defined(_WIN64)
#include "dynamixel_sdk_windows/port_handler_windows.h"
#endif

#if defined(__linux__) || defined(__APPLE__)
int     portHandler         (const char *port_name) { return portHandlerLinux(port_name); };

uint8_t openPort            (int port_num) { return openPortLinux(port_num); };
void    closePort           (int port_num) { closePortLinux(port_num); };
void    clearPort           (int port_num) { clearPortLinux(port_num); };

void    setPortName         (int port_num, const char *port_name) { setPortNameLinux(port_num, port_name); };
char   *getPortName         (int port_num) { return getPortNameLinux(port_num); };

uint8_t setBaudRate         (int port_num, const int baudrate) { return setBaudRateLinux(port_num, baudrate); };
int     getBaudRate         (int port_num) { return getBaudRateLinux(port_num); }

int     getBytesAvailable   (int port_num) { return getBytesAvailableLinux(port_num); }

int     readPort            (int port_num, uint8_t *packet, int length) { return readPortLinux(port_num, packet, length); };
int     writePort           (int port_num, uint8_t *packet, int length) { return writePortLinux(port_num, packet, length); };

void    setPacketTimeout    (int port_num, uint16_t packet_length) { setPacketTimeoutLinux(port_num, packet_length); };
void    setPacketTimeoutMSec(int port_num, double msec) { setPacketTimeoutMSecLinux(port_num, msec); };
uint8_t isPacketTimeout     (int port_num) { return isPacketTimeoutLinux(port_num); };
#endif

#if defined(_WIN32) || defined(_WIN64)
int     portHandler         (const char *port_name) { return portHandlerWindows(port_name); };

uint8_t openPort            (int port_num) { return openPortWindows(port_num); };
void    closePort           (int port_num) { closePortWindows(port_num); };
void    clearPort           (int port_num) { clearPortWindows(port_num); };

void    setPortName         (int port_num, const char *port_name) { setPortNameWindows(port_num, port_name); };
char   *getPortName         (int port_num) { return getPortNameWindows(port_num); };

uint8_t setBaudRate         (int port_num, const int baudrate) { return setBaudRateWindows(port_num, baudrate); };
int     getBaudRate         (int port_num) { return getBaudRateWindows(port_num); }

int     readPort            (int port_num, uint8_t *packet, int length) { return readPortWindows(port_num, packet, length); };
int     writePort           (int port_num, uint8_t *packet, int length) { return writePortWindows(port_num, packet, length); };

void    setPacketTimeout    (int port_num, uint16_t packet_length) { setPacketTimeoutWindows(port_num, packet_length); };
void    setPacketTimeoutMSec(int port_num, double msec) { setPacketTimeoutMSecWindows(port_num, msec); };
uint8_t isPacketTimeout     (int port_num) { return isPacketTimeoutWindows(port_num); };
#endif
