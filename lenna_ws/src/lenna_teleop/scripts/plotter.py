#!/usr/bin/python3

import time
import keyboard

from serial_handler import *
from packet_handler import *
from lenna_mobile_robot import *

import numpy as np
import matplotlib.pyplot as plt

import pandas as pd

DEVICENAME = '/dev/ttyTHS1'
BAUDRATE = 115200

serial = SerialHandler(DEVICENAME, BAUDRATE)
serial.openPort()

packet = PacketHandler(serial)
lenna = LennaMobileRobot(packet)

def main ():
    flag = False
    vel_left = 0
    vel_right = 0

    end_flag = False

    t = [0]
    vel = [0]
    vel2 = [0]
    vel_ref = [0]

    t_ds = 0
    print("PRESS KEY 'k' ON YOUR KEYBOARD TO START!")
    while True:
        if keyboard.is_pressed('k'):
            if not flag:
                initial_time = time.time()
                end_flag = False
                flag = True
        elif keyboard.is_pressed('q'):
            break
        
        if flag:
            time_diff = time.time() - initial_time
            t_ds = time_diff

            if time_diff <= 2.5:
                vel_right = 50

            elif time_diff <= 5: 
                vel_right = 100
            
            elif time_diff <= 7.5:
                vel_right = 150

            elif time_diff <= 10:
                vel_right = 200

            elif time_diff > 10:
                vel_right = 0
                end_flag = True
                flag = False

        lenna.setMotorSpeed(vel_right, vel_right)
        enc, _, _ = lenna.getOdometry()
        enc = getSigned(enc)
        if enc[1] and enc[0]:
            t.append(t_ds)
            vel2.append(enc[0])
            vel.append(enc[1])
            vel_ref.append(vel_right)

        time.sleep(0.01)

    my_dict = {"time" : t[1::], "vel": vel[1::], "ref":vel_ref[1::]}
    my_df = pd.DataFrame(my_dict)
    my_df.to_csv("out_filter.csv",index = False)

    # plt.xticks(np.linspace(0,10,21))
    # plt.yticks(np.linspace(0,250,6))
    plt.ylim(0,250)
    plt.xlim(0,10)
    plt.plot(t, vel, t, vel_ref, t , vel2)
    plt.xlabel("Time")
    plt.ylabel("Motor Angular Velocity (RPM)")
    plt.title("Motor Speed Control Output")
    plt.savefig("motor2.png")
    plt.grid(linestyle = '--', linewidth = 0.5)
    plt.show()

if __name__ == '__main__':
    main()