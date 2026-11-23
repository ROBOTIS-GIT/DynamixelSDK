#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Host-side test for GroupBulkRead.fastBulkRead() status packet parsing.
# A fake port replays one Fast Bulk Read status packet, so no device is needed.
# Run: python -m unittest discover -s python/tests/unit

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from dynamixel_sdk import GroupBulkRead, Protocol2PacketHandler  # noqa: E402
from dynamixel_sdk.robotis_def import (  # noqa: E402
    BROADCAST_ID, COMM_SUCCESS, DXL_HIBYTE, DXL_LOBYTE, INST_STATUS)

ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4


class FakePort:
    def __init__(self, rx_bytes):
        self.rx = list(rx_bytes)
        self.is_using = False
        self.written = []

    def clearPort(self):
        pass

    def writePort(self, packet):
        self.written.append(list(packet))
        return len(packet)

    def readPort(self, length):
        out, self.rx = self.rx[:length], self.rx[length:]
        return out

    def setPacketTimeout(self, packet_length):
        pass

    def isPacketTimeout(self):
        return not self.rx


def fast_bulk_read_status(ph, devices):
    """Build a Fast Bulk Read status packet.

    devices is a list of (dxl_id, data_bytes). Each device block is
    ERR ID DATA CRC_L CRC_H, and the CRC of the last block is the packet CRC.
    """
    params = []
    for dxl_id, data in devices:
        params += [0, dxl_id] + list(data) + [0, 0]
    length = 1 + len(params)  # INST + params (the last 2 bytes are the CRC)
    packet = [0xFF, 0xFF, 0xFD, 0x00, BROADCAST_ID,
              DXL_LOBYTE(length), DXL_HIBYTE(length), INST_STATUS] + params
    crc = ph.updateCRC(0, packet, len(packet) - 2)
    packet[-2] = DXL_LOBYTE(crc)
    packet[-1] = DXL_HIBYTE(crc)
    return packet


def position_bytes(value):
    return [value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF, (value >> 24) & 0xFF]


class TestFastBulkRead(unittest.TestCase):
    def run_fast_bulk_read(self, ids):
        ph = Protocol2PacketHandler()
        positions = {dxl_id: 1000 + 111 * dxl_id for dxl_id in ids}
        status = fast_bulk_read_status(
            ph, [(dxl_id, position_bytes(positions[dxl_id])) for dxl_id in ids])
        port = FakePort(status)
        group = GroupBulkRead(port, ph)
        for dxl_id in ids:
            self.assertTrue(group.addParam(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
        self.assertEqual(group.fastBulkRead(), COMM_SUCCESS)
        return group, positions

    def test_ids_1_to_2(self):
        group, positions = self.run_fast_bulk_read([1, 2])
        for dxl_id, position in positions.items():
            self.assertTrue(group.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
            self.assertEqual(group.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION), position)

    def test_id_equal_to_a_length_byte(self):
        # ID 4 also appears as the data length byte of ID 1 in the parameter
        # list [1, 132, 0, 4, 0, 2, ...]. The length of ID 4 must still be 4.
        group, positions = self.run_fast_bulk_read([1, 2, 3, 4, 5, 6])
        for dxl_id, position in positions.items():
            self.assertEqual(group.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION), position,
                             'ID %d' % dxl_id)

    def test_id_equal_to_an_address_byte(self):
        # Reading address 2 (Model Information) of ID 5 puts a 2 in the
        # parameter list before the entry for ID 2.
        ph = Protocol2PacketHandler()
        status = fast_bulk_read_status(ph, [(5, [0x11, 0x22, 0x33, 0x44]), (2, [0x55, 0x66]), (7, [0x77])])
        port = FakePort(status)
        group = GroupBulkRead(port, ph)
        self.assertTrue(group.addParam(5, 2, 4))
        self.assertTrue(group.addParam(2, ADDR_PRESENT_POSITION, 2))
        self.assertTrue(group.addParam(7, 146, 1))
        self.assertEqual(group.fastBulkRead(), COMM_SUCCESS)
        self.assertEqual(group.getData(5, 2, 4), 0x44332211)
        self.assertEqual(group.getData(2, ADDR_PRESENT_POSITION, 2), 0x6655)
        self.assertEqual(group.getData(7, 146, 1), 0x77)


if __name__ == '__main__':
    unittest.main()
