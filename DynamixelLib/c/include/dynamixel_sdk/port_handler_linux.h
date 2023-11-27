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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_LINUX_PORTHANDLERLINUX_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_LINUX_PORTHANDLERLINUX_C_H_


#include "port_handler.h"

int portHandlerLinux            (const char *port_name);

uint8_t setupPortLinux          (int port_num, const int cflag_baud);
uint8_t setCustomBaudrateLinux  (int port_num, int speed);
int     getCFlagBaud            (const int baudrate);

double  getCurrentTimeLinux     ();
double  getTimeSinceStartLinux  (int port_num);

uint8_t openPortLinux           (int port_num);
void    closePortLinux          (int port_num);
void    clearPortLinux          (int port_num);

void    setPortNameLinux        (int port_num, const char *port_name);
char   *getPortNameLinux        (int port_num);

uint8_t setBaudRateLinux        (int port_num, const int baudrate);
int     getBaudRateLinux        (int port_num);

int     getBytesAvailableLinux  (int port_num);

int     readPortLinux           (int port_num, uint8_t *packet, int length);
int     writePortLinux          (int port_num, uint8_t *packet, int length);

void    setPacketTimeoutLinux     (int port_num, uint16_t packet_length);
void    setPacketTimeoutMSecLinux (int port_num, double msec);
uint8_t isPacketTimeoutLinux      (int port_num);

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_LINUX_PORTHANDLERLINUX_C_H_ */
