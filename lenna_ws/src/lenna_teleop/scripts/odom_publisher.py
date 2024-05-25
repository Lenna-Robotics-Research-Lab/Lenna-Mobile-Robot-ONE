#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np

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
    pass
    # odom, _, _ = lenna.getOdometry(20)
    # print(getSigned(odom))