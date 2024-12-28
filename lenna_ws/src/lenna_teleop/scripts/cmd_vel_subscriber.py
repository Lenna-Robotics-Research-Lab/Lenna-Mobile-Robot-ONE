#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import numpy as np

from serial_handler import *
from packet_handler import *
from lenna_mobile_robot import *

DEVICENAME = '/dev/ttyTHS1'
BAUDRATE = 115200

serial = SerialHandler(DEVICENAME, BAUDRATE)
serial.openPort()

packet = PacketHandler(serial)
lenna = LennaMobileRobot(packet)

# Global variable to store handshake status
handshake_established = False

# handshake_flag = False

# Topic callback function for Twist messages.
def twistSubscriberCallback(data):
    global handshake_established
    # global handshake_flag
    if handshake_established:
        rospy.loginfo('linear x: %f, angular z: %f', data.linear.x, data.angular.z)
        vel_right = (data.linear.x) + (data.angular.z * lenna.wheel_distance / 4)
        vel_left = (data.linear.x) - (data.angular.z * lenna.wheel_distance / 4)

        vel_left = int(vel_left * 60 / (2*np.pi*lenna.wheel_radius))
        vel_right = int(vel_right * 60 / (2*np.pi*lenna.wheel_radius))

        if (abs(vel_left) >= lenna.max_motor_speed):
            vel_left = (vel_left/abs(vel_left)) * lenna.max_motor_speed
        
        if (abs(vel_right) >= lenna.max_motor_speed):
            vel_right = (vel_right/abs(vel_right)) * lenna.max_motor_speed

        rospy.loginfo('motor speeds L: %f, R: %f', vel_left, vel_right)

        lenna.setMotorSpeed(vel_left, vel_right)
        # handshake_flag = True      

# Topic callback function for Handshake messages.
def handshakeSubscriberCallback(data):
    global handshake_established
    if data.data:
        packet.handshake = 1
        handshake_established = True
        # rospy.loginfo('Handshake Established!')
    else:
        packet.handshake = 0
        handshake_established = False
        rospy.loginfo('Waiting for Serial Handshake!')

def main():
    rospy.init_node('node_serial_vel', anonymous=False)

    # Subscribe to Twist messages
    rospy.Subscriber('/cmd_vel', Twist, twistSubscriberCallback)

    # Subscribe to Handshake messages
    rospy.Subscriber('/handshake', Bool, handshakeSubscriberCallback)

    # hs_feedback = rospy.Publisher('/handshake_feedback', Bool, queue_size=10)

    rospy.loginfo("Node initialized and waiting for messages.")
    # while not rospy.is_shutdown():    
    #     if handshake_established:
    #         hs_feedback.publish(1)
    #     elif not handshake_established and handshake_flag:
    #         rospy.loginfo("Connection to the Board Terminated")
    #         hs_feedback.publish(0)
    #     elif not handshake_established and (not handshake_flag):
    #         rospy.loginfo("Handshake Data is Not Received Ignoring Twist Data")
    #         hs_feedback.publish(0)
    rospy.spin()

if __name__ == '__main__':
    main()


