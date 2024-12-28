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

# hs_feedback_flag = False
# isfirt_flag = True

# def feedbackSubscriberCallback(data):
#     global hs_feedback_flag
#     global isfirt_flag
#     if data:
#         hs_feedback_flag = True
#         isfirt_flag = False
#     elif not data:
#         hs_feedback_flag = False
#     else :
#         hs_feedback_flag = True

if __name__ == "__main__":
    rospy.init_node('node_handshake', anonymous=False)
    hs_pub = rospy.Publisher('/handshake', Bool, queue_size=10)

    # rospy.Subscriber('/handshake_feedback', Bool, feedbackSubscriberCallback)

    rate = rospy.Rate(10)

    flag = 0
    
    while not rospy.is_shutdown():
        if not flag:
            data = serial.readPort(5)

        if (data) and (not flag):
            hs_pub.publish(0)
            rospy.loginfo("handshake data received!")
            if data == b'LENNA':
                rospy.loginfo("handshake data is correct!") 
                serial.writePort([0x45])
                flag = 1
                # break
                # why 45? shinedown song or a holy decimal number???
            else:
                rospy.loginfo(f"incorrect handshake data : {data}")

        elif flag:
            hs_pub.publish(1)

        rate.sleep()

    # while not rospy.is_shutdown():
    #     if not hs_feedback_flag and (not isfirt_flag): 
    #         data = serial.readPort(10)
    #         hs_pub.publish(0)
    #         if data:
    #             rospy.loginfo("handshake data received!")
    #             if data == b'LENNA':
    #                 rospy.loginfo("handshake data is correct!") 
    #                 serial.writePort([0x45])
    #                 flag = 1
    #                 # why 45? shinedown song or a holy decimal number???
    #             else:
    #                 rospy.loginfo(f"incorrect handshake data : {data}")
    #         else:
    #             if flag == 1:
    #                 rospy.loginfo("connection to the host board is lost")
    #                 flag = 0
    #             else: 
    #                 rospy.loginfo("no data is available")
    #                 flag = 0
    #     else:
    #         flag = 1
    #         hs_pub.publish(bool(flag))    
        
       
    
        
