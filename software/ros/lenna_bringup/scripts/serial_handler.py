#!/usr/bin/python3
# -*- coding: utf-8 -*-

import time
import serial

class SerialHandler():
    def __init__(self, port_name='/dev/ttyTHS1', baudrate=115200):
        self.is_open = False
        self.is_using = False

        self.port_name = port_name
        self.baudrate = baudrate

        # Default Serial Parameters
        self.parity = serial.PARITY_NONE
        self.stopbits = serial.STOPBITS_ONE
        self.bytesize = serial.EIGHTBITS
        self.timeout = 0

        self.ser = None

        self.packet_start_time = 0.0
        self.packet_timeout = 0.0
        self.tx_time_per_byte = 0.0

    def openPort(self):
        self.setBaudrate(self.baudrate)

    def closePort(self):
        self.ser.close()
        self.is_open = False

    def clearPort(self):
        self.ser.flush()

    def setupPort(self):
        if self.is_open:
            self.closePort()

        self.ser = serial.Serial(
            port = self.port_name,
            baudrate = self.baudrate,
            parity = self.parity,
            stopbits = self.stopbits,
            bytesize = self.bytesize,
            timeout = self.timeout
        )

        self.is_open = True
        self.ser.reset_input_buffer()
        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0

        return True

    def setPortName(self, port_name):
        self.port_name = port_name

    def getPortName(self):
        return self.port_name

    def setBaudrate(self, baudrate):
        # standard baudrates supported on most platforms
        # read about them on: 
        # https://pyserial.readthedocs.io/en/latest/pyserial_api.html

        baudrates = [   9600, 19200, 38400, 57600, 115200, 
                        230400, 460800, 500000, 576000, 921600, 1000000, 
                        1152000, 2000000, 2500000, 3000000, 3500000, 4000000]
        
        if baudrate in baudrates:
            self.baudrate = baudrate
            self.setupPort()

            return self.baudrate
        else:
            return False

    def getBaudRate(self):
        return self.baudrate

    def getBytesAvailable(self):
        return self.ser.inWaiting()

    def readPort(self, length):
        # NOTE: only works for python 3
        return self.ser.read(length)

    def writePort(self, packet):
        return self.ser.write(packet)

    def setPacketTimeout(self, packet_length, latency_timer=16):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = (self.tx_time_per_byte * packet_length) + (latency_timer * 2.0) + 2.0

    def setPacketTimeoutMillis(self, msec):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = msec

    def isPacketTimeout(self):
        if self.getTimeSinceStart() > self.packet_timeout:
            self.packet_timeout = 0
            return True

        return False

    def getCurrentTime(self):
        return round(time.time() * 1000000000) / 1000000.0

    def getTimeSinceStart(self):
        time_since = self.getCurrentTime() - self.packet_start_time
        if time_since < 0.0:
            self.packet_start_time = self.getCurrentTime()

        return time_since