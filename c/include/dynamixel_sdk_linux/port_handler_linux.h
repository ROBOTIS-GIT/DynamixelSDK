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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_LINUX_PORTHANDLERLINUX_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_LINUX_PORTHANDLERLINUX_C_H_


#include "../dynamixel_sdk/port_handler.h"

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
