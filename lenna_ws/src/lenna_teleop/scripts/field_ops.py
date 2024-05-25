#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Communication Result
COMM_SUCCESS = 0            # tx or rx packet communication success
COMM_PORT_BUSY = -1000      # Port is busy (in use)
COMM_TX_FAIL = -1001        # Failed transmit instruction packet
COMM_RX_FAIL = -1002        # Failed get status packet
COMM_TX_ERROR = -2000       # Incorrect instruction packet
COMM_RX_WAITING = -3000     # Now recieving status packet
COMM_RX_TIMEOUT = -3001     # There is no status packet
COMM_RX_CORRUPT = -3002     # Incorrect status packet
COMM_NOT_AVAILABLE = -9000

# INSTRUCTION ADRESS TABLE
INST_MOTION_CONTROL = 0x01

def combine2Word(a, b):
    return (a & 0xFFFF) | (b & 0xFFFF) << 16

def combine2Byte(a, b):
    return (a & 0xFF) | ((b & 0xFF) << 8)

def lowWord(l):
    return l & 0xFFFF

def highWord(l):
    return (l >> 16) & 0xFFFF

def lowByte(w):
    return w & 0xFF

def highByte(w):
    return (w >> 8) & 0xFF

def getSigned(nums):
    signed = []
    for num in nums:
        if num & 0x8000:
            signed.append(num - 2**16)
        else:
            signed.append(num)
    return signed