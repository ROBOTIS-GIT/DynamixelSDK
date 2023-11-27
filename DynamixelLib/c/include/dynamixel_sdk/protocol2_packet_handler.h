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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL2PACKETHANDLER_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL2PACKETHANDLER_C_H_


#include "packet_handler.h"

WINDECLSPEC uint16_t    updateCRC           (uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
WINDECLSPEC void        addStuffing         (uint8_t *packet);
WINDECLSPEC void        removeStuffing      (uint8_t *packet);

WINDECLSPEC const char *getTxRxResult2      (int result);
WINDECLSPEC const char *getRxPacketError2       (uint8_t error);

WINDECLSPEC int         getLastTxRxResult2  (int port_num);
WINDECLSPEC uint8_t     getLastRxPacketError2   (int port_num);

WINDECLSPEC void        setDataWrite2       (int port_num, uint16_t data_length, uint16_t data_pos, uint32_t data);
WINDECLSPEC uint32_t    getDataRead2        (int port_num, uint16_t data_length, uint16_t data_pos);

WINDECLSPEC void        txPacket2           (int port_num);
WINDECLSPEC void        rxPacket2           (int port_num);
WINDECLSPEC void        txRxPacket2         (int port_num);

WINDECLSPEC void        ping2               (int port_num, uint8_t id);
WINDECLSPEC uint16_t    pingGetModelNum2    (int port_num, uint8_t id);

// BroadcastPing
WINDECLSPEC void        broadcastPing2      (int port_num);
WINDECLSPEC uint8_t     getBroadcastPingResult2 (int port_num, int id);

WINDECLSPEC void        action2             (int port_num, uint8_t id);
WINDECLSPEC void        reboot2             (int port_num, uint8_t id);
WINDECLSPEC void        clearMultiTurn2     (int port_num, uint8_t id);
WINDECLSPEC void        factoryReset2       (int port_num, uint8_t id, uint8_t option);

WINDECLSPEC void        readTx2             (int port_num, uint8_t id, uint16_t address, uint16_t length);
WINDECLSPEC void        readRx2             (int port_num, uint16_t length);
WINDECLSPEC void        readTxRx2           (int port_num, uint8_t id, uint16_t address, uint16_t length);

WINDECLSPEC void        read1ByteTx2        (int port_num, uint8_t id, uint16_t address);
WINDECLSPEC uint8_t     read1ByteRx2        (int port_num);
WINDECLSPEC uint8_t     read1ByteTxRx2      (int port_num, uint8_t id, uint16_t address);

WINDECLSPEC void        read2ByteTx2        (int port_num, uint8_t id, uint16_t address);
WINDECLSPEC uint16_t    read2ByteRx2        (int port_num);
WINDECLSPEC uint16_t    read2ByteTxRx2      (int port_num, uint8_t id, uint16_t address);

WINDECLSPEC void        read4ByteTx2        (int port_num, uint8_t id, uint16_t address);
WINDECLSPEC uint32_t    read4ByteRx2        (int port_num);
WINDECLSPEC uint32_t    read4ByteTxRx2      (int port_num, uint8_t id, uint16_t address);

WINDECLSPEC void        writeTxOnly2        (int port_num, uint8_t id, uint16_t address, uint16_t length);
WINDECLSPEC void        writeTxRx2          (int port_num, uint8_t id, uint16_t address, uint16_t length);

WINDECLSPEC void        write1ByteTxOnly2   (int port_num, uint8_t id, uint16_t address, uint8_t data);
WINDECLSPEC void        write1ByteTxRx2     (int port_num, uint8_t id, uint16_t address, uint8_t data);

WINDECLSPEC void        write2ByteTxOnly2   (int port_num, uint8_t id, uint16_t address, uint16_t data);
WINDECLSPEC void        write2ByteTxRx2     (int port_num, uint8_t id, uint16_t address, uint16_t data);

WINDECLSPEC void        write4ByteTxOnly2   (int port_num, uint8_t id, uint16_t address, uint32_t data);
WINDECLSPEC void        write4ByteTxRx2     (int port_num, uint8_t id, uint16_t address, uint32_t data);

WINDECLSPEC void        regWriteTxOnly2     (int port_num, uint8_t id, uint16_t address, uint16_t length);
WINDECLSPEC void        regWriteTxRx2       (int port_num, uint8_t id, uint16_t address, uint16_t length);

WINDECLSPEC void        syncReadTx2         (int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length);
// syncReadRx   -> GroupSyncRead
// syncReadTxRx -> GroupSyncRead

// param : ID1 DATA0 DATA1 ... DATAn ID2 DATA0 DATA1 ... DATAn ID3 DATA0 DATA1 ... DATAn
WINDECLSPEC void        syncWriteTxOnly2   (int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length);

// param : ID1 ADDR_L1 ADDR_H1 LEN_L1 LEN_H1 ID2 ADDR_L2 ADDR_H2 LEN_L2 LEN_H2 ...
WINDECLSPEC void        bulkReadTx2        (int port_num, uint16_t param_length);
// bulkReadRx   -> GroupBulkRead
// bulkReadTxRx -> GroupBulkRead

// param : ID1 START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H DATA0 DATA1 ... DATAn ID2 START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H DATA0 DATA1 ... DATAn
WINDECLSPEC void        bulkWriteTxOnly2   (int port_num, uint16_t param_length);

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL2PACKETHANDLER_C_H_ */
