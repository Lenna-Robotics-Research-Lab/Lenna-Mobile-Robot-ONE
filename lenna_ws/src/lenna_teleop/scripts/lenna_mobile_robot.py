#!/usr/bin/python3
# -*- coding: utf-8 -*-

from field_ops import *
from packet_handler import *
from field_ops import *

class LennaMobileRobot():
    def __init__(self, protocol):
        self.protocol = protocol
        self.odometry_length = 32
        self.min_motor_speed = 0
        self.max_motor_speed = 250 # rpm
        self.wheel_radius = 0.0375 # meter
        self.wheel_distance = 0.18 # meter

    def setMotorSpeed(self, motor_speed_left, motor_speed_right):
        return self.protocol.txPacket(INST_MOTION_CONTROL, [motor_speed_left, motor_speed_right])

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