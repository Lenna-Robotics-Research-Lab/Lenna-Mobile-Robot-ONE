#!/usr/bin/python3
# -*- coding: utf-8 -*-

from serial_handler import *
from packet_handler import *

DEVICENAME = '/dev/ttyTHS1'
BAUDRATE = 115200

serial = SerialHandler(DEVICENAME, BAUDRATE)
serial.openPort()

packet = PacketHandler()

while True:
    res = packet.txPacket(serial, 0xA0, [0xABCD])

# txpacket = [0xFF, 0xFF, 0x00, 0x02, 0x0F, 0x0F, 0xAB, 0xCD]

# while True:
#     serial.writePort(txpacket)
