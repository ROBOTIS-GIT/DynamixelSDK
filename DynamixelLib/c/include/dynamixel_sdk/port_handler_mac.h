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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_MAC_PORTHANDLERMAC_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_MAC_PORTHANDLERMAC_C_H_


#include "port_handler.h"

int portHandlerMac            (const char *port_name);

uint8_t setupPortMac          (int port_num, const int cflag_baud);
uint8_t setCustomBaudrateMac  (int port_num, int speed);
int     getCFlagBaud            (const int baudrate);

double  getCurrentTimeMac     ();
double  getTimeSinceStartMac  (int port_num);

uint8_t openPortMac           (int port_num);
void    closePortMac          (int port_num);
void    clearPortMac          (int port_num);

void    setPortNameMac        (int port_num, const char *port_name);
char   *getPortNameMac        (int port_num);

uint8_t setBaudRateMac        (int port_num, const int baudrate);
int     getBaudRateMac        (int port_num);

int     getBytesAvailableMac  (int port_num);

int     readPortMac           (int port_num, uint8_t *packet, int length);
int     writePortMac          (int port_num, uint8_t *packet, int length);

void    setPacketTimeoutMac     (int port_num, uint16_t packet_length);
void    setPacketTimeoutMSecMac (int port_num, double msec);
uint8_t isPacketTimeoutMac      (int port_num);

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_MAC_PORTHANDLERMAC_C_H_ */
