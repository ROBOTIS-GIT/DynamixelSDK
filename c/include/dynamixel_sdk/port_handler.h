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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLER_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLER_C_H_

#if defined(__linux__) || defined(__APPLE__)
#define WINDECLSPEC
#elif defined(_WIN32) || defined(_WIN64)
#ifdef WINDLLEXPORT
#define WINDECLSPEC __declspec(dllexport)
#else
#define WINDECLSPEC __declspec(dllimport)
#endif
#endif

#include "robotis_def.h"

static const int DEFAULT_BAUDRATE = 1000000;

int     g_used_port_num;
uint8_t    *g_is_using;

WINDECLSPEC int     portHandler             (const char *port_name);

WINDECLSPEC uint8_t openPort                (int port_num);
WINDECLSPEC void    closePort               (int port_num);
WINDECLSPEC void    clearPort               (int port_num);

WINDECLSPEC void    setPortName             (int port_num, const char* port_name);
WINDECLSPEC char   *getPortName             (int port_num);

WINDECLSPEC uint8_t setBaudRate             (int port_num, const int baudrate);
WINDECLSPEC int     getBaudRate             (int port_num);

#if defined(__linux__) || defined(__APPLE__)
WINDECLSPEC int     getBytesAvailable       (int port_num);
#endif

WINDECLSPEC int     readPort                (int port_num, uint8_t *packet, int length);
WINDECLSPEC int     writePort               (int port_num, uint8_t *packet, int length);

WINDECLSPEC void    setPacketTimeout        (int port_num, uint16_t packet_length);
WINDECLSPEC void    setPacketTimeoutMSec    (int port_num, double msec);
WINDECLSPEC uint8_t isPacketTimeout         (int port_num);


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLER_C_H_ */
