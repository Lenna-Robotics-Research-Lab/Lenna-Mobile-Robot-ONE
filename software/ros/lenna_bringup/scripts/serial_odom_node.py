#!/usr/bin/env python3
"""
ROS Node: SerialOdomNode
Publishes odometry data from a serial-connected Lenna mobile robot.
"""

import rospy
import numpy as np

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Quaternion
from serial_handler import SerialHandler
from packet_handler import PacketHandler
from field_ops import getSigned
from lenna_mobile_robot import LennaMobileRobot


class SerialOdomNode:
    """ROS Node to read odometry data from serial and publish as nav_msgs/Odometry."""

    def __init__(self):
        # --- ROS Parameters ---
        self.DEVICENAME = rospy.get_param("~port", "/dev/ttyTHS1")
        self.BAUDRATE = rospy.get_param("~baudrate", 115200)
        self.parent_frame = rospy.get_param("~odom_tf_parent_frame", "odom")
        self.child_frame = rospy.get_param("~odom_tf_child_frame", "base_link")

        # --- Serial & Robot Setup ---
        self.serial = SerialHandler(self.DEVICENAME, self.BAUDRATE)
        self.packet = PacketHandler(self.serial)
        self.lenna = LennaMobileRobot(self.packet)

        # Open serial port
        self.serial.openPort()

        # --- State Variables ---
        self.handshake_established = False
        self.odom = self._init_odometry_msg()

        # Wheel distances & velocities
        self.left_wheel_dist = 0
        self.right_wheel_dist = 0
        self.left_wheel_vel = 0
        self.right_wheel_vel = 0
        self.left_dist_prev = 0
        self.right_dist_prev = 0

        # Robot pose estimate
        self.x = 0
        self.y = 0
        self.theta = 0

        # --- ROS Setup ---
        rospy.init_node("node_serial_odom", anonymous=False)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
        self.handshake_sub = rospy.Subscriber("/handshake", Bool, self.handshake_callback)

        rospy.loginfo("SerialOdomNode initialized! Waiting for handshake and sensor readings...")

    def _init_odometry_msg(self) -> Odometry:
        """Initialize and return an empty Odometry message with default values."""
        odom = Odometry()
        odom.header.frame_id = self.parent_frame
        odom.child_frame_id = self.child_frame

        # Default pose
        odom.pose.pose.position.x = 0
        odom.pose.pose.position.y = 0
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation.x = 0
        odom.pose.pose.orientation.y = 0
        odom.pose.pose.orientation.z = 0
        odom.pose.pose.orientation.w = 1

        # Default twist
        odom.twist.twist.linear.x = 0
        odom.twist.twist.angular.z = 0

        return odom

    def update_odom(self):
        """
        Perform dead reckoning to update robot pose.
        Uses wheel encoder distances to compute new x, y, and theta.
        """
        # Convert from mm to meters
        self.left_wheel_dist /= 1000
        self.right_wheel_dist /= 1000
        self.left_dist_prev /= 1000
        self.right_dist_prev /= 1000

        # Wheel increments
        d_l = self.left_wheel_dist - self.left_dist_prev
        d_r = self.right_wheel_dist - self.right_dist_prev

        # Robot motion
        d_avg = (d_l + d_r) / 2.0
        d_theta = (d_r - d_l) / self.lenna.wheel_distance

        # Previous states
        x_old = self.odom.pose.pose.position.x
        y_old = self.odom.pose.pose.position.y
        theta_old = (self.right_dist_prev - self.left_dist_prev) / self.lenna.wheel_distance

        # Update states
        self.theta = theta_old + d_theta
        self.x = x_old + np.cos(self.theta) * d_avg
        self.y = y_old + np.sin(self.theta) * d_avg

    def handle_odometry(self):
        """
        Main loop: read encoder values, compute odometry, and publish results.
        """
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            if not self.handshake_established:
                rospy.logwarn_throttle(10, "Handshake not established!")
                rate.sleep()
                continue

            # Get encoder data
            enc, _, _ = self.lenna.getOdometry()
            enc = getSigned(enc)

            if any(enc):
                # Update timestamp
                self.odom.header.stamp = rospy.Time.now()

                # Extract wheel distances
                self.left_wheel_dist, self.right_wheel_dist = enc[2], enc[3]
                self.update_odom()
                self.left_dist_prev, self.right_dist_prev = enc[2], enc[3]

                # Update pose
                self.odom.pose.pose.position.x = self.x
                self.odom.pose.pose.position.y = self.y
                self.odom.pose.pose.orientation.z = np.sin(self.theta / 2.0)
                self.odom.pose.pose.orientation.w = np.cos(self.theta / 2.0)

                # Compute wheel velocities
                self.left_wheel_vel = (enc[0] / 60.0) * (2 * np.pi * self.lenna.wheel_radius)
                self.right_wheel_vel = (enc[1] / 60.0) * (2 * np.pi * self.lenna.wheel_radius)

                # Update twist
                self.odom.twist.twist.angular.z = (self.left_wheel_vel - self.right_wheel_vel) / self.lenna.wheel_distance
                self.odom.twist.twist.linear.x = (self.left_wheel_vel + self.right_wheel_vel) / 2.0

                # Publish odometry
                self.odom_pub.publish(self.odom)

            rate.sleep()

    def handshake_callback(self, msg: Bool):
        """Callback to monitor handshake status from another node."""
        if msg.data:
            if not self.handshake_established:
                rospy.loginfo("Handshake Established!")
            self.handshake_established = True
        else:
            if self.handshake_established:
                rospy.logwarn("Handshake Lost! Waiting for handshake...")
            self.handshake_established = False


if __name__ == "__main__":
    node = SerialOdomNode()
    node.handle_odometry()
