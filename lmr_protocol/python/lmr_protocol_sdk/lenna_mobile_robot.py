#!/usr/bin/python3
# -*- coding: utf-8 -*-

from field_ops import *
from packet_handler import *

class LennaMobileRobot():
    def __init__(self, protocol):
        self.protocol = protocol

    def setMotorSpeed(self, motor_speed_left, motor_speed_right):
        return self.protocol.txPacket(INST_MOTION_CONTROL, [motor_speed_left, motor_speed_right])