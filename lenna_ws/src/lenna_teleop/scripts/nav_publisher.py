#!/usr/bin/python3

import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
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

class SerialOdomNode():
    def __init__(self):
        # ROS parameters
        self.DEVICENAME = rospy.get_param("~port", "/dev/ttyTHS1")
        self.BAUDRATE = rospy.get_param("~baudrate", 115200)
        self.parent_frame = rospy.get_param("~odom_tf_parent_frame", "odom")
        self.child_frame = rospy.get_param("~odom_tf_child_frame", "base_link")

        # Initialize serial communication and robot interfaces
        self.serial = SerialHandler(self.DEVICENAME, self.BAUDRATE)
        self.packet = PacketHandler(self.serial)
        self.lenna = LennaMobileRobot(self.packet)

        # Open serial port
        self.serial.openPort()

        # Handshake flag
        self.handshake_established = False

        # Odometry instance
        self.odom = Odometry()

        self.odom.header.frame_id = self.parent_frame 
        self.odom.child_frame_id = self.child_frame
        self.odom.pose.pose.position.x = 0
        self.odom.pose.pose.position.y = 0
        self.odom.pose.pose.position.z = 0
        self.odom.pose.pose.orientation.x = 0
        self.odom.pose.pose.orientation.y = 0
        self.odom.pose.pose.orientation.z = 0
        self.odom.pose.pose.orientation.w = 1
        self.odom.twist.twist.angular.z = 0
        self.odom.twist.twist.linear.x = 0

        # Inputs for odometry calculations
        self.left_wheel_dist = 0
        self.right_wheel_dist = 0
        self.left_wheel_vel = 0
        self.right_wheel_vel = 0

        self.left_dist_prev = 0
        self.right_dist_prev = 0

        # Robot states
        self.x = 0
        self.y = 0
        self.theta = 0

        rospy.init_node('node_serial_odom', anonymous=False)

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        self.handshake_sub = rospy.Subscriber('/handshake', Bool, self.handshake_callback)

        rospy.loginfo("SerialOdomlNode initialized! Waiting for serial handshake and sensor readings...")

    def update_odom(self):
        """Dead reckoning; update odometry message"""
        self.left_wheel_dist /= 1000
        self.right_wheel_dist /= 1000

        self.left_dist_prev /= 1000
        self.right_dist_prev /= 1000

        d_l = self.left_wheel_dist - self.left_dist_prev
        d_r = self.right_wheel_dist - self.right_dist_prev

        d_avg_dist = (d_l + d_r) / 2
        d_theta = (d_r - d_l) / (self.lenna.wheel_distance)

        x_old = self.odom.pose.pose.position.x
        y_old = self.odom.pose.pose.position.y
        theta_old = (self.right_dist_prev - self.left_dist_prev) / (self.lenna.wheel_distance)

        self.theta = theta_old + d_theta
        self.x = x_old + np.cos(self.theta) * d_avg_dist
        self.y = y_old + np.sin(self.theta) * d_avg_dist

    def handle_odometry(self):
        rate = rospy.Rate(100)
        self.odom.header.stamp = rospy.Time.now()

        while not rospy.is_shutdown():
            if not self.handshake_established:
                rospy.logwarn_throttle(10, "Handshake not established!")
                continue

            enc, _, _ = self.lenna.getOdometry()
            enc = getSigned(enc)

            if any(enc):
                self.odom.header.stamp = rospy.Time.now()

                self.left_wheel_dist, self.right_wheel_dist = enc[2] ,enc[3]  
                self.update_odom()
                self.left_dist_prev, self.right_dist_prev = enc[2] ,enc[3]    

                self.odom.pose.pose.position.x = self.x 
                self.odom.pose.pose.position.y = self.y
                self.odom.pose.pose.position.z = 0

                self.odom.pose.pose.orientation.x = 0
                self.odom.pose.pose.orientation.y = 0
                self.odom.pose.pose.orientation.z = np.sin(self.theta / 2)
                self.odom.pose.pose.orientation.w = np.cos(self.theta / 2)

                self.left_wheel_vel  = (enc[0] / 60) * (2*np.pi*self.lenna.wheel_radius)   # m/s
                self.right_wheel_vel  = (enc[1] / 60) * (2*np.pi*self.lenna.wheel_radius) # m/s

                self.odom.twist.twist.angular.z = (self.left_wheel_vel - self.right_wheel_vel) / (self.lenna.wheel_distance)
                self.odom.twist.twist.linear.x = (self.left_wheel_vel + self.right_wheel_vel) / 2

                self.odom_pub.publish(self.odom)
            rate.sleep()

    def handshake_callback(self, msg: Bool):
        """Callback for handshake status updates."""
        if msg.data:
            if not self.handshake_established:
                rospy.loginfo("Handshake Established!")
            self.handshake_established = True
        else:
            if self.handshake_established:
                rospy.logwarn("Handshake Lost! Waiting for Serial Handshake...")
            self.handshake_established = False


if __name__ == "__main__":
    node = SerialOdomNode()
    node.handle_odometry()