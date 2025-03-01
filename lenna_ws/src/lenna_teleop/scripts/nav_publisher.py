#!/usr/bin/python3

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

import numpy as np
import math

from serial_handler import *
from packet_handler import *
from field_ops import *
from lenna_mobile_robot import *

import time

DEVICENAME = '/dev/ttyTHS1'
BAUDRATE = 115200

serial = SerialHandler(DEVICENAME, BAUDRATE)
serial.openPort()

packet = PacketHandler(serial)
lenna = LennaMobileRobot(packet)

odom = Odometry()

def update_odom(left_vel, right_vel, left_dist, right_dist, old_l, old_r):

    # mm to meter
    left_dist /= 1000
    right_dist /= 1000

    old_l /= 1000
    old_r /= 1000

    left_vel = (left_vel / 60) * (2*np.pi*lenna.wheel_radius)   # m/s
    right_vel = (right_vel / 60) * (2*np.pi*lenna.wheel_radius) # m/s

    d_l = left_dist - old_l
    d_r = right_dist - old_r

    d_avg_dist = (d_l + d_r)/2
    d_theta = (d_r - d_l) / (lenna.wheel_distance)

    x_old = odom.pose.pose.position.x
    y_old = odom.pose.pose.position.y
    theta_old = (old_r - old_l) / (lenna.wheel_distance)

    theta_new = theta_old + d_theta
    x_new = x_old + np.cos(theta_new) * d_avg_dist
    y_new = y_old + np.sin(theta_new) * d_avg_dist

    return x_new, y_new, theta_new

if __name__ == "__main__":
    rospy.init_node('node_odom', anonymous=False)
    my_nav = rospy.Publisher('/odom', Odometry, queue_size=50)

    rate = rospy.Rate(100) # change from 100 to 50
    
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = "base_link"

    odom.pose.pose.position.x = 0
    odom.pose.pose.position.y = 0
    odom.pose.pose.position.z = 0

    odom.pose.pose.orientation.x = 0
    odom.pose.pose.orientation.y = 0
    odom.pose.pose.orientation.z = 0
    odom.pose.pose.orientation.w = 1

    old_l = 0
    old_r = 0

    x_new = 0
    y_new = 0
    theta_new = 0

    while not rospy.is_shutdown():
        enc, _, _ = lenna.getOdometry()
        enc = getSigned(enc)

        if any(enc):
            [x_new, y_new, theta_new] = update_odom(enc[0], enc[1], enc[2], enc[3], old_l, old_r)
            old_l, old_r = enc[2] ,enc[3]
        

        odom.header.stamp = rospy.Time.now()

        odom.pose.pose.position.x = x_new 
        odom.pose.pose.position.y = y_new
        odom.pose.pose.position.z = 0

        odom.pose.pose.orientation.x = 0
        odom.pose.pose.orientation.y = 0
        odom.pose.pose.orientation.z = np.sin(theta_new/2)
        odom.pose.pose.orientation.w = np.cos(theta_new/2)

        my_nav.publish(odom)
        rate.sleep()