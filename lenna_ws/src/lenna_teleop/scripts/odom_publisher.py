#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

import numpy as np

from serial_handler import *
from packet_handler import *
from field_ops import *
from lenna_mobile_robot import *

import time

DEVICENAME = '/dev/ttyTHS1'
BAUDRATE = 115200

serial = SerialHandler(DEVICENAME, BAUDRATE)
serial.openPort()

packet = PacketHandler(serial)
lenna = LennaMobileRobot(packet)

if __name__ == "__main__":
    rospy.init_node('node_imu', anonymous=False)
    #the difference between this and the reference code is that this is 
    #publishing undet IMU and /imu_data and the other one is publishing under
    #nav_msgs.Odometry
    imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)
    rate = rospy.Rate(100)

    imu_data = Imu()

    # roll = np.radians(-45)
    # roll_2 = np.radians(315)

    # orient = lenna.rpy2quat(roll, 0, 0)
    # orient_2 = lenna.rpy2quat(roll_2, 0, 0)
    # print(orient)
    # print(orient_2)

    # time.sleep(20)

    while not rospy.is_shutdown():
        odom, _, _ = lenna.getOdometry(100)
        odom = getSigned(odom) 

        imu_data.header.stamp = rospy.Time.now()
        imu_data.header.frame_id = 'imu'

        imu_data.linear_acceleration.x = odom[4] / 1000
        imu_data.linear_acceleration.y = odom[5] / 1000
        imu_data.linear_acceleration.z = odom[6] / 1000

        imu_data.angular_velocity.x = odom[7] / 1000
        imu_data.angular_velocity.y = odom[8] / 1000
        imu_data.angular_velocity.z = odom[9] / 1000

        # print(f"imu in degree: roll:{1} pitch:{2} yaw:{3}", odom[10], odom[11], odom[12])

        roll = np.radians(odom[10])
        pitch = np.radians(odom[11])
        yaw = np.radians(odom[12])

        orientation = lenna.rpy2quat(roll, pitch, yaw)

        imu_data.orientation.x = orientation[0]
        imu_data.orientation.y = orientation[1]
        imu_data.orientation.z = orientation[2]
        imu_data.orientation.w = orientation[3]
        
        imu_pub.publish(imu_data)
        # rospy.loginfo("All went well. Publishing topic /imu_data")
        rate.sleep()

