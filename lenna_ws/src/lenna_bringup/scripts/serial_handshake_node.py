#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import serial
import time
from std_msgs.msg import Bool

from serial_handler import SerialHandler
from packet_handler import PacketHandler

class HandshakeNode:
    def __init__(self):
        # ROS parameters for serial port
        self.DEVICENAME = rospy.get_param("~port", "/dev/ttyTHS1")
        self.BAUDRATE = rospy.get_param("~baudrate", 115200)

        # Initialize serial and packet handler
        self.serial = SerialHandler(self.DEVICENAME, self.BAUDRATE)
        self.packet = PacketHandler(self.serial)

        # Open serial port
        self.serial.openPort()

        # Initialize ROS publisher
        self.hs_pub = rospy.Publisher('/handshake', Bool, queue_size=10)

        # Flag to check if handshake is done
        self.flag = False

    def handle_handshake(self):
        """Main handshake logic."""
        # Loop rate: 10 Hz
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # If handshake not yet done, try to read incoming data (timeout 5s)
            if not self.flag:
                data = self.serial.readPort(5)

            # Check if data was received
            if data and not self.flag:
                self.hs_pub.publish(False)  # Handshake not done yet
                rospy.loginfo("Handshake data received!")

                # Expecting the low-level embedded board to send "LENNA" as handshake keyword
                if data == b'LENNA':
                    rospy.loginfo("Handshake data is correct!")
                    self.serial.writePort([0x45])  # Send confirmation byte
                    self.flag = True
                else:
                    rospy.logwarn(f"Incorrect handshake data: {data}")
            elif self.flag:
                # Publish "connected" state
                self.hs_pub.publish(True)
            
            rate.sleep()

    def run(self):
        """Run the ROS node and handle handshake."""
        rospy.init_node('node_handshake', anonymous=False)
        self.handle_handshake()


if __name__ == "__main__":
    # Instantiate and run the HandshakeNode
    node = HandshakeNode()
    node.run()
