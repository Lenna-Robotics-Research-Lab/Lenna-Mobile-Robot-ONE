#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion

import numpy as np

from serial_handler import *
from packet_handler import *
from field_ops import *
from lenna_mobile_robot import *

DEVICENAME = '/dev/ttyTHS1'
BAUDRATE = 115200

serial = SerialHandler(DEVICENAME, BAUDRATE)
serial.openPort()

packet = PacketHandler(serial)
lenna = LennaMobileRobot(packet)

if __name__ == "__main__":
    rospy.init_node('node_odom_imu', anonymous=False)
    imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)
    rate = rospy.Rate(100)

    imu_data = Imu()

    while not rospy.is_shutdown():
        odom, _, _ = lenna.getOdometry(10)
        odom = getSigned(odom) 

        imu_data.header.stamp = rospy.Time.now()
        imu_data.header.frame_id = 'imu_sensor'

        imu_data.linear_acceleration.x = odom[3]/ 1000
        imu_data.linear_acceleration.y = odom[4]/ 1000
        imu_data.linear_acceleration.z = 0

        imu_data.angular_velocity.x = odom[5]/ 1000
        imu_data.angular_velocity.y = odom[6]/ 1000
        imu_data.angular_velocity.z = odom[7]/ 1000
        
        imu_pub.publish(imu_data)
        rospy.loginfo("All went well. Publishing topic /imu_data")
        rate.sleep()