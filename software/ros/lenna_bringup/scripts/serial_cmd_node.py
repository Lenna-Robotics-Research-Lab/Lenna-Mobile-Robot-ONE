#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import time

from serial_handler import SerialHandler
from packet_handler import PacketHandler
from lenna_mobile_robot import LennaMobileRobot


class SerialCmdVelNode:
    def __init__(self):
        # ROS parameters for serial port
        self.DEVICENAME = rospy.get_param("~port", "/dev/ttyTHS1")
        self.BAUDRATE = rospy.get_param("~baudrate", 115200)
        self.WAIT_TIME = rospy.get_param("~ser_wait_time", 0.01)

        # Initialize serial communication and robot interfaces
        self.serial = SerialHandler(self.DEVICENAME, self.BAUDRATE)
        self.packet = PacketHandler(self.serial)
        self.lenna = LennaMobileRobot(self.packet)

        # Open serial port
        self.serial.openPort()

        # Handshake flag
        self.handshake_established = False

        # Robot velocities history
        self.linear = 0.0
        self.angular = 0.0

        # Initialize ROS node components
        rospy.init_node('node_serial_cmd_vel', anonymous=False)
        
        # Subscribers
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.twist_callback)
        self.handshake_sub = rospy.Subscriber('/handshake', Bool, self.handshake_callback)

        rospy.loginfo("SerialCmdVelNode initialized! Waiting for serial handshake and velocity commands...")

    def twist_callback(self, msg: Twist):
        """Callback for velocity commands, sends motor speeds if handshake established."""
        if not self.handshake_established:
            rospy.logwarn_throttle(10, "Handshake not established. Ignoring velocity commands.")
            return

        # Calculate individual motor speeds
        vel_right = (msg.linear.x) + (msg.angular.z * self.lenna.wheel_distance / 4)
        vel_left = (msg.linear.x) - (msg.angular.z * self.lenna.wheel_distance / 4)

        # Convert from m/s to RPM
        vel_right = int(vel_right * 60 / (2 * np.pi * self.lenna.wheel_radius))
        vel_left = int(vel_left * 60 / (2 * np.pi * self.lenna.wheel_radius))

        # Clamp motor speeds
        vel_left = max(min(vel_left, self.lenna.max_motor_speed), -self.lenna.max_motor_speed)
        vel_right = max(min(vel_right, self.lenna.max_motor_speed), -self.lenna.max_motor_speed)

        # Store changes in velocity commands
        if (self.linear != msg.linear.x) or (self.angular != msg.angular.z):
            self.linear, self.angular = msg.linear.x, msg.angular.z
            
            # Log robot kinematic and individual motor velocities
            rospy.loginfo(  f'Diff. Robot Velocities:\t linear: {round(msg.linear.x, 2)},\t angular: {round(msg.angular.z, 2)} \n'
                        f'\t\t\t\t  Motor Velocities:\t left: {vel_left},\t right: {vel_right}')


        # Send motor velocity commands over serial
        self.lenna.setMotorSpeed(vel_left, vel_right, self.WAIT_TIME)

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

    def spin(self):
        """Keep the node running."""
        rospy.spin()


if __name__ == '__main__':
    node = SerialCmdVelNode()
    node.spin()