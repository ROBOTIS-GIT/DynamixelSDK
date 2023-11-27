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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_WINDOWS_PORTHANDLERWINDOWS_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_WINDOWS_PORTHANDLERWINDOWS_C_H_

#include <Windows.h>
#include "port_handler.h"

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
