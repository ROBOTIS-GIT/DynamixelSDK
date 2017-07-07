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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_WINDOWS_PORTHANDLERWINDOWS_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_WINDOWS_PORTHANDLERWINDOWS_C_H_

#include <Windows.h>
#include "../dynamixel_sdk/port_handler.h"

WINDECLSPEC uint8_t setupPortWindows            (int port_num, const int baudrate);

WINDECLSPEC double  getCurrentTimeWindows       (int port_num);
WINDECLSPEC double  getTimeSinceStartWindows    (int port_num);

WINDECLSPEC int     portHandlerWindows          (const char *port_name);

WINDECLSPEC uint8_t openPortWindows             (int port_num);
WINDECLSPEC void    closePortWindows            (int port_num);
WINDECLSPEC void    clearPortWindows            (int port_num);

WINDECLSPEC void    setPortNameWindows          (int port_num, const char* port_name);
WINDECLSPEC char   *getPortNameWindows          (int port_num);

WINDECLSPEC uint8_t setBaudRateWindows          (int port_num, const int baudrate);
WINDECLSPEC int     getBaudRateWindows          (int port_num);

WINDECLSPEC int     readPortWindows             (int port_num, uint8_t *packet, int length);
WINDECLSPEC int     writePortWindows            (int port_num, uint8_t *packet, int length);

WINDECLSPEC void    setPacketTimeoutWindows     (int port_num, uint16_t packet_length);
WINDECLSPEC void    setPacketTimeoutMSecWindows (int port_num, double msec);
WINDECLSPEC uint8_t isPacketTimeoutWindows      (int port_num);

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_LINUX_PORTHANDLERWINDOWS_C_H_ */
