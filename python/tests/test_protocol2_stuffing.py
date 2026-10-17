#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Regression tests for Protocol 2.0 byte stuffing on the transmit path
# (https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/700).
# These tests are hardware-free: pyserial is stubbed and the port is mocked,
# so they can run anywhere with a plain Python interpreter:
#
#   python3 -m unittest discover -s python/tests -p "test_*.py"

import os
import sys
import types
import unittest

sys.modules.setdefault("serial", types.ModuleType("serial"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "src"))

from dynamixel_sdk.robotis_def import DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD
from dynamixel_sdk.protocol2_packet_handler import Protocol2PacketHandler


def crc16(data):
    """Reference CRC-16 (poly 0x8005, init 0x0000, MSB-first), independent of
    the SDK's table-driven updateCRC()."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8005) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class FakePort(object):
    is_using = False

    def __init__(self):
        self.written = None

    def clearPort(self):
        pass

    def writePort(self, packet):
        self.written = list(packet)
        return len(packet)


def make_tx_packet(packet_handler, dxl_id, address, data):
    """Build the unstuffed instruction packet exactly like writeTxOnly() does,
    then let txPacket() stuff it and capture what reaches the port."""
    length = len(data)
    txpacket = [0] * (length + 12)

    txpacket[4] = dxl_id                              # PKT_ID
    txpacket[5] = DXL_LOBYTE(length + 5)              # PKT_LENGTH_L
    txpacket[6] = DXL_HIBYTE(length + 5)              # PKT_LENGTH_H
    txpacket[7] = 0x03                                # PKT_INSTRUCTION (WRITE)
    txpacket[8] = DXL_LOBYTE(address)                 # PKT_PARAMETER0
    txpacket[9] = DXL_HIBYTE(address)
    txpacket[10:10 + length] = data

    port = FakePort()
    result = packet_handler.txPacket(port, txpacket)
    return result, port.written


class TestProtocol2TxStuffing(unittest.TestCase):

    def setUp(self):
        self.handler = Protocol2PacketHandler()

    def test_add_stuffing_updates_caller_packet_in_place(self):
        # Goal Position value whose little-endian bytes contain FF FF FD.
        packet = [0] * 16
        packet[4] = 0x01                              # PKT_ID
        packet[5] = 0x09                              # PKT_LENGTH_L (unstuffed)
        packet[6] = 0x00                              # PKT_LENGTH_H
        packet[7] = 0x03                              # PKT_INSTRUCTION (WRITE)
        packet[8:14] = [0x74, 0x00, 0xFF, 0xFF, 0xFD, 0x00]

        stuffed = self.handler.addStuffing(packet)

        # The caller's list must be the stuffed packet (same object, updated
        # in place), matching the C++ SDK where the buffer is modified where
        # it lies.
        self.assertTrue(stuffed is packet)
        self.assertEqual(len(packet), 17)
        self.assertEqual(packet[5], 0x0A)             # stuffed length
        self.assertEqual(packet[6], 0x00)
        self.assertEqual(packet[10:15], [0xFF, 0xFF, 0xFD, 0xFD, 0x00])

    def test_add_stuffing_without_trigger_leaves_length(self):
        packet = [0] * 16
        packet[4] = 0x01
        packet[5] = 0x09
        packet[6] = 0x00
        packet[7] = 0x03
        packet[8:14] = [0x74, 0x00, 0x78, 0x56, 0x34, 0x12]

        stuffed = self.handler.addStuffing(packet)

        self.assertTrue(stuffed is packet)
        self.assertEqual(len(packet), 16)
        self.assertEqual(packet[5], 0x09)             # length unchanged
        self.assertEqual(packet[6], 0x00)

    def test_write_with_header_sequence_reaches_port_stuffed(self):
        # 0x00FDFFFF little-endian is FF FF FD 00: needs one stuffed FD.
        # Expected on-wire bytes verified against an independent CRC-16
        # implementation (poly 0x8005) and issue #700.
        result, wire = make_tx_packet(self.handler, 1, 116,
                                      [0xFF, 0xFF, 0xFD, 0x00])

        self.assertEqual(result, 0)                   # COMM_SUCCESS
        self.assertEqual(wire,
                         [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x0A, 0x00, 0x03,
                          0x74, 0x00, 0xFF, 0xFF, 0xFD, 0xFD, 0x00, 0x21,
                          0xE7])
        self.assertEqual(crc16(wire[:-2]),
                         wire[-2] | (wire[-1] << 8))  # CRC consistency

    def test_write_with_header_sequence_at_data_start_reaches_port_stuffed(self):
        # 0x02FDFFFF little-endian is FF FF FD 02: the stuffed FD must be
        # inserted before the trailing data byte.
        result, wire = make_tx_packet(self.handler, 1, 116,
                                      [0xFF, 0xFF, 0xFD, 0x02])

        self.assertEqual(result, 0)
        self.assertEqual(wire,
                         [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x0A, 0x00, 0x03,
                          0x74, 0x00, 0xFF, 0xFF, 0xFD, 0xFD, 0x02, 0x2E,
                          0x67])
        self.assertEqual(crc16(wire[:-2]),
                         wire[-2] | (wire[-1] << 8))

    def test_write_without_header_sequence_reaches_port_unstuffed(self):
        # No FF FF FD in the payload: length stays 0x09 and the wire bytes
        # carry the plain data with a matching CRC.
        result, wire = make_tx_packet(self.handler, 1, 116,
                                      [0x78, 0x56, 0x34, 0x12])

        self.assertEqual(result, 0)
        self.assertEqual(wire,
                         [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x03,
                          0x74, 0x00, 0x78, 0x56, 0x34, 0x12, 0xA2, 0x55])
        self.assertEqual(crc16(wire[:-2]),
                         wire[-2] | (wire[-1] << 8))


if __name__ == "__main__":
    unittest.main()
