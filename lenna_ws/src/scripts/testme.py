#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist

import numpy as np

from serial_handler import *
from packet_handler import *
from lenna_mobile_robot import *

l = 0.2 # in meters SI unit
r = 0.0375

DEVICENAME = '/dev/ttyTHS1'
BAUDRATE = 115200

serial = SerialHandler(DEVICENAME, BAUDRATE)
serial.openPort()

packet = PacketHandler(serial)
lenna = LennaMobileRobot(packet)

# Topic callback function.
def twistSubscriberCallback(data):
    rospy.loginfo('linear x: %f, angular z: %f', data.linear.x, data.angular.z)
    vel_right = (data.linear.x) + (data.angular.z * l / 4)
    vel_left = (data.linear.x) - (data.angular.z * l / 4)

    vel_left = int(vel_left * 60 / (2*np.pi*r))
    vel_right = int(vel_right * 60 / (2*np.pi*r))

    rospy.loginfo('motor speeds L: %f, R: %f', vel_left, vel_right)

    lenna.setMotorSpeed(vel_left, vel_right)

def twistSubscriber():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'stringListener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('node_serial', anonymous=False)
    rospy.Subscriber('/cmd_vel', Twist, twistSubscriberCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    twistSubscriber()