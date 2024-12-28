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

    # if left_vel >= 300:
    #     left_vel = 300
    # elif left_vel <= -300:
    #     left_vel = -300

    # if right_vel >= 300:
    #     right_vel = 300
    # elif right_vel <= -300:
    #     right_vel = -300

    vel_r_raw = right_vel
    vel_l_raw = left_vel

    left_vel = (left_vel / 60) * (2*np.pi*lenna.wheel_radius)   # m/s
    right_vel = (right_vel / 60) * (2*np.pi*lenna.wheel_radius) # m/s

    d_l = left_dist - old_l
    d_r = right_dist - old_r

    # if d_l >= 0.02:
    #     d_l = 0.02
    # elif d_l <= -0.02:
    #     d_l = -0.02

    # if d_r >= 0.02:
    #     d_r = 0.02
    # elif d_r <= -0.02:
    #     d_r = -0.02 

    d_avg_dist = (d_l + d_r)/2
    d_theta = (d_r - d_l) / (lenna.wheel_distance / 2)

    # if (d_theta > np.pi) :
    #     d_theta -= 2*np.pi

    # elif (d_theta < -np.pi) :
    #     d_theta += 2*np.pi


    x_old = odom.pose.pose.position.x
    y_old = odom.pose.pose.position.y
    theta_old = (old_r - old_l) / (lenna.wheel_distance / 2)

    # if (theta_old > np.pi) :
    #     theta_old -= 2*np.pi

    # elif (theta_old < -np.pi) :
    #     theta_old += 2*np.pi

    theta_new = theta_old + d_theta
    x_new = x_old + np.cos(theta_new) * d_avg_dist
    y_new = y_old + np.sin(theta_new) * d_avg_dist

    odom.header.stamp = rospy.Time.now()

    odom.pose.pose.position.x = x_new
    odom.pose.pose.position.y = y_new
    odom.pose.pose.position.z = 0

    odom.pose.pose.orientation.x = 0
    odom.pose.pose.orientation.y = 0
    odom.pose.pose.orientation.z = np.sin(theta_new/2)
    odom.pose.pose.orientation.w = np.cos(theta_new/2)
    
    odom.twist.twist.linear.x = (right_vel + left_vel) / 2
    odom.twist.twist.angular.z = 2 * (right_vel - left_vel) / lenna.wheel_distance


if __name__ == "__main__":
    rospy.init_node('node_odom', anonymous=False)
    my_nav = rospy.Publisher('/odom', Odometry, queue_size=50)

    rate = rospy.Rate(100)
    
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

    while not rospy.is_shutdown():
        enc, _, _ = lenna.getOdometry()
        enc = getSigned(enc)

        update_odom(enc[0], enc[1], enc[2], enc[3], old_l, old_r)
        old_l, old_r = enc[2] ,enc[3]
        
        my_nav.publish(odom)
        rate.sleep()