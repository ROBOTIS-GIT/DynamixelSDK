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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_C_H_


#include "packet_handler.h"

WINDECLSPEC const char *getTxRxResult1      (int result);
WINDECLSPEC const char *getRxPacketError1   (uint8_t error);

WINDECLSPEC int         getLastTxRxResult1  (int port_num);
WINDECLSPEC uint8_t     getLastRxPacketError1   (int port_num);

WINDECLSPEC void        setDataWrite1       (int port_num, uint16_t data_length, uint16_t data_pos, uint32_t data);
WINDECLSPEC uint32_t    getDataRead1        (int port_num, uint16_t data_length, uint16_t data_pos);

WINDECLSPEC void        txPacket1           (int port_num);
WINDECLSPEC void        rxPacket1           (int port_num);
WINDECLSPEC void        txRxPacket1         (int port_num);

WINDECLSPEC void        ping1               (int port_num, uint8_t id);
WINDECLSPEC uint16_t    pingGetModelNum1    (int port_num, uint8_t id);

// broadcastPing
WINDECLSPEC void        broadcastPing1      (int port_num);
WINDECLSPEC uint8_t     getBroadcastPingResult1 (int port_num, int id);

WINDECLSPEC void        action1             (int port_num, uint8_t id);
WINDECLSPEC void        reboot1             (int port_num, uint8_t id);
WINDECLSPEC void        clearMultiTurn1     (int port_num, uint8_t id);
WINDECLSPEC void        factoryReset1       (int port_num, uint8_t id, uint8_t option);

WINDECLSPEC void        readTx1             (int port_num, uint8_t id, uint16_t address, uint16_t length);
WINDECLSPEC void        readRx1             (int port_num, uint16_t length);
WINDECLSPEC void        readTxRx1           (int port_num, uint8_t id, uint16_t address, uint16_t length);

WINDECLSPEC void        read1ByteTx1        (int port_num, uint8_t id, uint16_t address);
WINDECLSPEC uint8_t     read1ByteRx1        (int port_num);
WINDECLSPEC uint8_t     read1ByteTxRx1      (int port_num, uint8_t id, uint16_t address);

WINDECLSPEC void        read2ByteTx1        (int port_num, uint8_t id, uint16_t address);
WINDECLSPEC uint16_t    read2ByteRx1        (int port_num);
WINDECLSPEC uint16_t    read2ByteTxRx1      (int port_num, uint8_t id, uint16_t address);

WINDECLSPEC void        read4ByteTx1        (int port_num, uint8_t id, uint16_t address);
WINDECLSPEC uint32_t    read4ByteRx1        (int port_num);
WINDECLSPEC uint32_t    read4ByteTxRx1      (int port_num, uint8_t id, uint16_t address);

WINDECLSPEC void        writeTxOnly1        (int port_num, uint8_t id, uint16_t address, uint16_t length);
WINDECLSPEC void        writeTxRx1          (int port_num, uint8_t id, uint16_t address, uint16_t length);

WINDECLSPEC void        write1ByteTxOnly1   (int port_num, uint8_t id, uint16_t address, uint8_t data);
WINDECLSPEC void        write1ByteTxRx1     (int port_num, uint8_t id, uint16_t address, uint8_t data);

WINDECLSPEC void        write2ByteTxOnly1   (int port_num, uint8_t id, uint16_t address, uint16_t data);
WINDECLSPEC void        write2ByteTxRx1     (int port_num, uint8_t id, uint16_t address, uint16_t data);

WINDECLSPEC void        write4ByteTxOnly1   (int port_num, uint8_t id, uint16_t address, uint32_t data);
WINDECLSPEC void        write4ByteTxRx1     (int port_num, uint8_t id, uint16_t address, uint32_t data);

WINDECLSPEC void        regWriteTxOnly1     (int port_num, uint8_t id, uint16_t address, uint16_t length);
WINDECLSPEC void        regWriteTxRx1       (int port_num, uint8_t id, uint16_t address, uint16_t length);

WINDECLSPEC void        syncReadTx1         (int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length);
// syncReadRx   -> GroupSyncRead
// syncReadTxRx -> GroupSyncRead

// param : ID1 DATA0 DATA1 ... DATAn ID2 DATA0 DATA1 ... DATAn ID3 DATA0 DATA1 ... DATAn
WINDECLSPEC void        syncWriteTxOnly1    (int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length);

// param : LEN1 ID1 ADDR1 LEN2 ID2 ADDR2 ...
WINDECLSPEC void        bulkReadTx1         (int port_num, uint16_t param_length);
// bulkReadRx   -> GroupBulkRead
// bulkReadTxRx -> GroupBulkRead

// param : ID1 DATA0 DATA1 ... DATAn ID2 DATA0 DATA1 ... DATAn ID3 DATA0 DATA1 ... DATAn
WINDECLSPEC void        bulkWriteTxOnly1    (int port_num, uint16_t param_length);

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_C_H_ */
