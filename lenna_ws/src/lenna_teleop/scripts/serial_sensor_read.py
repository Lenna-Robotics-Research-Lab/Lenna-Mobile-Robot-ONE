#!/usr/bin/python3

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

from lenna_msgs.msg import OdomInformation

from serial_handler import *
from packet_handler import *
from field_ops import *
from lenna_mobile_robot import *

import numpy as np
import math
import time

DEVICENAME = '/dev/ttyTHS1'
BAUDRATE = 115200

serial = SerialHandler(DEVICENAME, BAUDRATE)
serial.openPort()

packet = PacketHandler(serial)
lenna = LennaMobileRobot(packet)

imu_data = Imu()

odom = Odometry()
odom_old = Odometry()

odomak = OdomInformation()

def update_odom(left_vel, right_vel, left_dist, right_dist, old_l, old_r):
    left_dist /= 100
    right_dist /= 100

    old_l /= 100
    old_r /= 100

    left_vel = (left_vel / 60) * (2*np.pi*lenna.wheel_radius)
    right_vel = (right_vel / 60) * (2*np.pi*lenna.wheel_radius)

    avg_dist = (left_dist + right_dist)/2
    avg_angle = 2 * math.atan((right_dist - left_dist))/lenna.wheel_distance

    d_l = left_dist - old_l
    d_r = right_dist - old_r
    
    d_avg_dist = (d_l + d_r)/2
    d_th = 2 * math.atan((d_l - d_r))/lenna.wheel_distance

    if (d_th > np.pi) :
        d_th -= 2*np.pi

    elif (d_th < -np.pi) :
        d_th += 2*np.pi

    if (avg_angle > np.pi) :
        avg_angle -= 2*np.pi

    elif (avg_angle < -np.pi) :
        avg_angle += 2*np.pi

    odom.pose.pose.position.x = odom_old.pose.pose.position.x + np.cos(d_th) * d_avg_dist
    odom.pose.pose.position.y = odom_old.pose.pose.position.y + np.sin(d_th) * d_avg_dist
    odom.pose.pose.orientation.z = avg_angle
    
    #   Prevent lockup from a single bad cycle
    if (np.isnan(odom.pose.pose.position.x) or np.isnan(odom.pose.pose.position.y)
        or np.isnan(odom.pose.pose.position.z)):
        odom.pose.pose.position.x = odom_old.pose.pose.position.x
        odom.pose.pose.position.y = odom_old.pose.pose.position.y
        odom.pose.pose.orientation.z = odom_old.pose.pose.orientation.z
 
    #    Make sure theta stays in the correct range
    if (odom.pose.pose.orientation.z > np.pi):
        odom.pose.pose.orientation.z -= 2 * np.pi
    elif (odom.pose.pose.orientation.z < -np.pi): 
        odom.pose.pose.orientation.z += 2 * np.pi

    
    #   // Compute the velocity
    odom.header.stamp = rospy.Time.now()
    odom.twist.twist.linear.x = (right_vel + left_vel) / 2
    odom.twist.twist.angular.z = 2 * (right_vel - left_vel) / lenna.wheel_distance
 
    #   // Save th e pose data for the next cycle
    odom_old.pose.pose.position.x = odom.pose.pose.position.x
    odom_old.pose.pose.position.y = odom.pose.pose.position.y
    odom_old.pose.pose.orientation.z = odom.pose.pose.orientation.z
    odom_old.header.stamp = odom.header.stamp

if __name__ == "__main__":
    rospy.init_node('node_serial_sens', anonymous=False)
    my_nav = rospy.Publisher('/odom_serial', OdomInformation, queue_size=10)

    rate = rospy.Rate(100)
    
    odom.pose.pose.position.x = 0 #initial x which I think is 0
    odom.pose.pose.position.y = 0
    odom.pose.pose.position.z = 0

    old_l = 0
    old_r = 0

    while not rospy.is_shutdown():
        al_odom, _, _ = lenna.getOdometry(10)
        al_odom = getSigned(al_odom)

        odom.header.frame_id = 'odom'
        odom.child_frame_id = "base_link"
        odom.header.stamp = rospy.Time.now()
        update_odom(al_odom[0], al_odom[1], al_odom[2] , al_odom[3], old_l, old_r)
        old_l, old_r = al_odom[2] ,al_odom[3]
        
        odom.pose.covariance = [   0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1    ]

        odom.twist.covariance = [ 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.01   ]

        imu_data.header.stamp = odom.header.stamp 
        imu_data.header.frame_id = 'imu'

        imu_data.orientation_covariance = [ 0.05, 0.0, 0.0,
                                            0.0, 0.05, 0.0,
                                            0.0, 0.0, 0.05 ]

        imu_data.angular_velocity_covariance = [ 0.02, 0.0, 0.0,
                                            0.0, 0.02, 0.0,
                                            0.0, 0.0, 0.02 ]

        imu_data.linear_acceleration_covariance = [ 0.03, 0.0, 0.0,
                                            0.0, 0.03, 0.0,
                                            0.0, 0.0, 0.03 ]

        imu_data.linear_acceleration.x = al_odom[4] / 1000
        imu_data.linear_acceleration.y = al_odom[5] / 1000
        imu_data.linear_acceleration.z = al_odom[6] / 1000

        imu_data.angular_velocity.x = al_odom[7] / 1000
        imu_data.angular_velocity.y = al_odom[8] / 1000
        imu_data.angular_velocity.z = al_odom[9] / 1000

        roll = np.radians(al_odom[10])
        pitch = np.radians(al_odom[11])
        yaw = np.radians(al_odom[12])

        orientation = lenna.rpy2quat(roll, pitch, yaw)

        imu_data.orientation.x = orientation[0]
        imu_data.orientation.y = orientation[1]
        imu_data.orientation.z = orientation[2]
        imu_data.orientation.w = orientation[3]
        
        odomak.imu_data = imu_data
        odomak.encoder_data = odom 

        my_nav.publish(odomak)
        rate.sleep()
