#!/usr/bin/python3
# -*- coding: utf-8 -*-

from field_ops import *
from packet_handler import *
from field_ops import *

import numpy as np
import time

class LennaMobileRobot():
    def __init__(self, protocol):
        self.protocol = protocol
        self.odometry_length = 32
        self.min_motor_speed = 0
        self.max_motor_speed = 250 # rpm
        self.wheel_radius = 0.0325 # meter
        self.wheel_distance = 0.19 # meter

    def rpy2quat(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

    def setMotorSpeed(self, motor_speed_left, motor_speed_right, wait_time=0.0):
        COMM_STATUS = self.protocol.    txPacket(INST_MOTION_CONTROL, [motor_speed_left, motor_speed_right])
        time.sleep(wait_time)
        return COMM_STATUS

    def getOdometry(self, timeout=100):
        data, length, result = self.protocol.rxPacket(timeout)
        
        right_wheel_speed = 0
        left_wheel_speed = 0

        right_wheel_distance = 0
        left_wheel_distance = 0

        roll = 0
        pitch = 0
        yaw = 0        
        
        gyro_x = 0
        gyro_y = 0
        gyro_z = 0

        acc_x = 0
        acc_y = 0
        acc_z = 0

        if result == COMM_SUCCESS:
            if length == self.odometry_length:
                left_wheel_speed = combine2Byte(data[5], data[4])
                right_wheel_speed = combine2Byte(data[7], data[6])

                left_wheel_distance = combine2Byte(data[9], data[8])
                right_wheel_distance = combine2Byte(data[11], data[10])

                acc_x = combine2Byte(data[13], data[12])
                acc_y = combine2Byte(data[15], data[14])
                acc_z = combine2Byte(data[17], data[16])

                gyro_x = combine2Byte(data[19], data[18])
                gyro_y = combine2Byte(data[21], data[20])
                gyro_z = combine2Byte(data[23], data[22])

                roll = combine2Byte(data[25], data[24])
                pitch = combine2Byte(data[27], data[26])
                yaw = combine2Byte(data[29], data[28])

            else:
                result = COMM_RX_FAIL

        return [left_wheel_speed, right_wheel_speed, left_wheel_distance, right_wheel_distance, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw], length, result