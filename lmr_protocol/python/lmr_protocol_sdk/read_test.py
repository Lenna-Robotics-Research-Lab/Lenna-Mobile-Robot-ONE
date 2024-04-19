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
    if serial.getBytesAvailable():
        a = serial.readPort(8)
        print(a)
