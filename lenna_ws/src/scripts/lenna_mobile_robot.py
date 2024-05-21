#!/usr/bin/python3
# -*- coding: utf-8 -*-

from field_ops import *
from packet_handler import *
from field_ops import *

class LennaMobileRobot():
    def __init__(self, protocol):
        self.protocol = protocol
        self.odometry_length = 16

    def setMotorSpeed(self, motor_speed_left, motor_speed_right):
        return self.protocol.txPacket(INST_MOTION_CONTROL, [motor_speed_left, motor_speed_right])

    def getOdometry(self, timeout=100):
        data, length, result = self.protocol.rxPacket(timeout)

        heading = 0
        motor_speed_right = 0
        motor_speed_left = 0
        acc_x = 0
        acc_y = 0
        gyro_x = 0

        if result == COMM_SUCCESS:
            if length == self.odometry_length:
                heading = combine2Byte(data[3], data[2])
                motor_speed_left = combine2Byte(data[5], data[4])
                motor_speed_right = combine2Byte(data[7], data[6])
                acc_x = combine2Byte(data[9], data[8])
                acc_y = combine2Byte(data[11], data[10])
                gyro_x = combine2Byte(data[13], data[12])
            else:
                result = COMM_RX_FAIL

        return [heading, motor_speed_left, motor_speed_right, acc_x, acc_y, gyro_x], length, result