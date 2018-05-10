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

using System;
using System.Runtime.InteropServices;

namespace dynamixel_sdk
{
  unsafe class dynamixel
  {
    const string dll_path = "dxl_x64_c.dll";

    #region PortHandler
    [DllImport(dll_path)]
    public static extern int    portHandler         (string port_name);

    [DllImport(dll_path)]
    public static extern bool   openPort            (int port_num);
    [DllImport(dll_path)]
    public static extern void   closePort           (int port_num);
    [DllImport(dll_path)]
    public static extern void   clearPort           (int port_num);

    [DllImport(dll_path)]
    public static extern void   setPortName         (int port_num, string port_name);
    [DllImport(dll_path)]
    public static extern string getPortName         (int port_num);

    [DllImport(dll_path)]
    public static extern bool   setBaudRate         (int port_num, int baudrate);
    [DllImport(dll_path)]
    public static extern int    getBaudRate         (int port_num);

    #if __linux__
    [DllImport(dll_path)]
    public static extern int    getBytesAvailable   (int port_num);
    #endif

    [DllImport(dll_path)]
    public static extern int    readPort            (int port_num, Byte *packet, int length);
    [DllImport(dll_path)]
    public static extern int    writePort           (int port_num, Byte* packet, int length);

    [DllImport(dll_path)]
    public static extern void   setPacketTimeout    (int port_num, UInt16 packet_length);
    [DllImport(dll_path)]
    public static extern void   setPacketTimeoutMSec(int port_num, double msec);
    [DllImport(dll_path)]
    public static extern bool   isPacketTimeout     (int port_num);
    #endregion

    #region PacketHandler
    [DllImport(dll_path)]
    public static extern void   packetHandler       ();

    [DllImport(dll_path)]
    public static extern void   printTxRxResult     (int protocol_version, int result);
    [DllImport(dll_path)]
    public static extern IntPtr getTxRxResult       (int protocol_version, int result);
    [DllImport(dll_path)]
    public static extern void   printRxPacketError  (int protocol_version, byte error);
    [DllImport(dll_path)]
    public static extern IntPtr getRxPacketError    (int protocol_version, byte error);
    
    [DllImport(dll_path)]
    public static extern int    getLastTxRxResult   (int port_num, int protocol_version);
    [DllImport(dll_path)]
    public static extern byte   getLastRxPacketError(int port_num, int protocol_version);

    [DllImport(dll_path)]
    public static extern void   setDataWrite        (int port_num, int protocol_version, UInt16 data_length, UInt16 data_pos, UInt32 data);
    [DllImport(dll_path)]
    public static extern UInt32 getDataRead         (int port_num, int protocol_version, UInt16 data_length, UInt16 data_pos);

    [DllImport(dll_path)]
    public static extern void   txPacket            (int port_num, int protocol_version);

    [DllImport(dll_path)]
    public static extern void   rxPacket            (int port_num, int protocol_version);

    [DllImport(dll_path)]
    public static extern void   txRxPacket          (int port_num, int protocol_version);

    [DllImport(dll_path)]
    public static extern void   ping                (int port_num, int protocol_version, byte id);

    [DllImport(dll_path)]
    public static extern UInt16 pingGetModelNum     (int port_num, int protocol_version, byte id);

    [DllImport(dll_path)]
    public static extern void   broadcastPing       (int port_num, int protocol_version);
    [DllImport(dll_path)]
    public static extern bool   getBroadcastPingResult(int port_num, int protocol_version, int id);

    [DllImport(dll_path)]
    public static extern void   reboot              (int port_num, int protocol_version, byte id);

    [DllImport(dll_path)]
    public static extern void   factoryReset        (int port_num, int protocol_version, byte id, byte option);

    [DllImport(dll_path)]
    public static extern void   readTx              (int port_num, int protocol_version, byte id, UInt16 address, UInt16 length);
    [DllImport(dll_path)]
    public static extern void   readRx              (int port_num, int protocol_version, UInt16 length);
    [DllImport(dll_path)]
    public static extern void   readTxRx            (int port_num, int protocol_version, byte id, UInt16 address, UInt16 length);

    [DllImport(dll_path)]
    public static extern void   read1ByteTx         (int port_num, int protocol_version, byte id, UInt16 address);
    [DllImport(dll_path)]
    public static extern byte   read1ByteRx         (int port_num, int protocol_version);
    [DllImport(dll_path)]
    public static extern byte   read1ByteTxRx       (int port_num, int protocol_version, byte id, UInt16 address);

    [DllImport(dll_path)]
    public static extern void   read2ByteTx         (int port_num, int protocol_version, byte id, UInt16 address);
    [DllImport(dll_path)]
    public static extern UInt16 read2ByteRx         (int port_num, int protocol_version);
    [DllImport(dll_path)]
    public static extern UInt16 read2ByteTxRx       (int port_num, int protocol_version, byte id, UInt16 address);

    [DllImport(dll_path)]
    public static extern void   read4ByteTx         (int port_num, int protocol_version, byte id, UInt16 address);
    [DllImport(dll_path)]
    public static extern UInt32 read4ByteRx         (int port_num, int protocol_version);
    [DllImport(dll_path)]
    public static extern UInt32 read4ByteTxRx       (int port_num, int protocol_version, byte id, UInt16 address);

    [DllImport(dll_path)]
    public static extern void   writeTxOnly         (int port_num, int protocol_version, byte id, UInt16 address, UInt16 length);
    [DllImport(dll_path)]
    public static extern void   writeTxRx           (int port_num, int protocol_version, byte id, UInt16 address, UInt16 length);

    [DllImport(dll_path)]
    public static extern void   write1ByteTxOnly    (int port_num, int protocol_version, byte id, UInt16 address, byte data);
    [DllImport(dll_path)]
    public static extern void   write1ByteTxRx      (int port_num, int protocol_version, byte id, UInt16 address, byte data);

    [DllImport(dll_path)]
    public static extern void   write2ByteTxOnly    (int port_num, int protocol_version, byte id, UInt16 address, UInt16 data);
    [DllImport(dll_path)]
    public static extern void   write2ByteTxRx      (int port_num, int protocol_version, byte id, UInt16 address, UInt16 data);

    [DllImport(dll_path)]
    public static extern void   write4ByteTxOnly    (int port_num, int protocol_version, byte id, UInt16 address, UInt32 data);
    [DllImport(dll_path)]
    public static extern void   write4ByteTxRx      (int port_num, int protocol_version, byte id, UInt16 address, UInt32 data);

    [DllImport(dll_path)]
    public static extern void   regWriteTxOnly      (int port_num, int protocol_version, byte id, UInt16 address, UInt16 length);
    [DllImport(dll_path)]
    public static extern void   regWriteTxRx        (int port_num, int protocol_version, byte id, UInt16 address, UInt16 length);

    [DllImport(dll_path)]
    public static extern void   syncReadTx          (int port_num, int protocol_version, UInt16 start_address, UInt16 data_length, UInt16 param_length);
    // syncReadRx   -> GroupSyncRead
    // syncReadTxRx -> GroupSyncRead

    [DllImport(dll_path)]
    public static extern void   syncWriteTxOnly     (int port_num, int protocol_version, UInt16 start_address, UInt16 data_length, UInt16 param_length);

    [DllImport(dll_path)]
    public static extern void   bulkReadTx          (int port_num, int protocol_version, UInt16 param_length);
    // bulkReadRx   -> GroupBulkRead
    // bulkReadTxRx -> GroupBulkRead

    [DllImport(dll_path)]
    public static extern void   bulkWriteTxOnly     (int port_num, int protocol_version, UInt16 param_length);
    #endregion

    #region GroupBulkRead
    [DllImport(dll_path)]
    public static extern int    groupBulkRead       (int port_num, int protocol_version);

    [DllImport(dll_path)]
    public static extern bool   groupBulkReadAddParam   (int group_num, byte id, UInt16 start_address, UInt16 data_length);
    [DllImport(dll_path)]
    public static extern void   groupBulkReadRemoveParam(int group_num, byte id);
    [DllImport(dll_path)]
    public static extern void   groupBulkReadClearParam (int group_num);

    [DllImport(dll_path)]
    public static extern void   groupBulkReadTxPacket   (int group_num);
    [DllImport(dll_path)]
    public static extern void   groupBulkReadRxPacket   (int group_num);
    [DllImport(dll_path)]
    public static extern void   groupBulkReadTxRxPacket (int group_num);

    [DllImport(dll_path)]
    public static extern bool   groupBulkReadIsAvailable(int group_num, byte id, UInt16 address, UInt16 data_length);
    [DllImport(dll_path)]
    public static extern UInt32 groupBulkReadGetData    (int group_num, byte id, UInt16 address, UInt16 data_length);
    #endregion

    #region GroupBulkWrite
    [DllImport(dll_path)]
    public static extern int    groupBulkWrite            (int port_num, int protocol_version);

    [DllImport(dll_path)]
    public static extern bool   groupBulkWriteAddParam    (int group_num, byte id, UInt16 start_address, UInt16 data_length, UInt32 data, UInt16 input_length);
    [DllImport(dll_path)]
    public static extern void   groupBulkWriteRemoveParam (int group_num, byte id);
    [DllImport(dll_path)]
    public static extern bool   groupBulkWriteChangeParam (int group_num, byte id, UInt16 start_address, UInt16 data_length, UInt32 data, UInt16 input_length, UInt16 data_pos);
    [DllImport(dll_path)]
    public static extern void   groupBulkWriteClearParam  (int group_num);

    [DllImport(dll_path)]
    public static extern void   groupBulkWriteTxPacket    (int group_num);
    #endregion

    #region GroupSyncRead
    [DllImport(dll_path)]
    public static extern int    groupSyncRead             (int port_num, int protocol_version, UInt16 start_address, UInt16 data_length);

    [DllImport(dll_path)]
    public static extern bool   groupSyncReadAddParam     (int group_num, byte id);
    [DllImport(dll_path)]
    public static extern void   groupSyncReadRemoveParam  (int group_num, byte id);
    [DllImport(dll_path)]
    public static extern void   groupSyncReadClearParam   (int group_num);

    [DllImport(dll_path)]
    public static extern void   groupSyncReadTxPacket     (int group_num);
    [DllImport(dll_path)]
    public static extern void   groupSyncReadRxPacket     (int group_num);
    [DllImport(dll_path)]
    public static extern void   groupSyncReadTxRxPacket   (int group_num);

    [DllImport(dll_path)]
    public static extern bool   groupSyncReadIsAvailable  (int group_num, byte id, UInt16 address, UInt16 data_length);
    [DllImport(dll_path)]
    public static extern UInt32 groupSyncReadGetData      (int group_num, byte id, UInt16 address, UInt16 data_length);
    #endregion

    #region GroupSyncWrite
    [DllImport(dll_path)]
    public static extern int    groupSyncWrite            (int port_num, int protocol_version, UInt16 start_address, UInt16 data_length);

    [DllImport(dll_path)]
    public static extern bool   groupSyncWriteAddParam    (int group_num, byte id, UInt32 data, UInt16 data_length);
    [DllImport(dll_path)]
    public static extern void   groupSyncWriteRemoveParam (int group_num, byte id);
    [DllImport(dll_path)]
    public static extern bool   groupSyncWriteChangeParam (int group_num, byte id, UInt32 data, UInt16 data_length, UInt16 data_pos);
    [DllImport(dll_path)]
    public static extern void   groupSyncWriteClearParam  (int group_num);

    [DllImport(dll_path)]
    public static extern void   groupSyncWriteTxPacket    (int group_num);
    #endregion
  }
}
