#!/usr/bin/python3
# -*- coding: utf-8 -*-

from serial_handler import *
from packet_handler import *
from field_ops import *
from lenna_mobile_robot import *

DEVICENAME = '/dev/ttyTHS1'
BAUDRATE = 115200

serial = SerialHandler(DEVICENAME, BAUDRATE)
serial.openPort()

packet = PacketHandler(serial)
lenna = LennaMobileRobot(packet)

while True:
    odom, _, _ = lenna.getOdometry(100)
    print(getSigned(odom))