#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy 
import serial
import time

from serial_handler import *
from packet_handler import *
from lenna_mobile_robot import *

from std_msgs.msg import Bool


DEVICENAME = '/dev/ttyTHS1'
BAUDRATE = 115200

serial = SerialHandler(DEVICENAME, BAUDRATE)
serial.openPort()

packet = PacketHandler(serial)

if __name__ == "__main__":
    rospy.init_node('node_handshake', anonymous=False)
    hs_pub = rospy.Publisher('/handshake', Bool, queue_size=10)
    rate = rospy.Rate(10)

    
    while(True):
        data = serial.readPort(10)
        hs_pub.publish(0)
        if data:
            rospy.loginfo("handshake data received!")
            if data == b'LENNA':
                rospy.loginfo("handshake data is correct!") 
                serial.writePort([0x45])
                flag = 1
                break
                # why 45? shinedown song or a holy decimal number???
            else:
                rospy.loginfo(f"incorrect handshake data : {data}")

    while not rospy.is_shutdown():
        hs_pub.publish(1)
        rate.sleep()
        
