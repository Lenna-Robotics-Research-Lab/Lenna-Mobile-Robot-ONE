#!/usr/bin/python3

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

import numpy as np
import math

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

odom = Odometry()
odom_old = Odometry()


def update_odom(left_vel, right_vel, left_dist, right_dist, old_l, old_r):

    # mm to meter
    left_dist /= 1000
    right_dist /= 1000

    old_l /= 1000
    old_r /= 1000

    left_vel = (left_vel / 60) * (2*np.pi*lenna.wheel_radius)   # m/s
    right_vel = (right_vel / 60) * (2*np.pi*lenna.wheel_radius) # m/s

    avg_dist = (left_dist + right_dist)/2
    avg_angle = 2 * math.atan((right_dist - left_dist))/lenna.wheel_distance

    # mapping 0~2pi to -pi~pi
    if (avg_angle > np.pi) :
        avg_angle -= 2*np.pi

    elif (avg_angle < -np.pi) :
        avg_angle += 2*np.pi

    d_l = left_dist - old_l
    d_r = right_dist - old_r
    
    d_avg_dist = (d_l + d_r)/2
    d_th = 2 * math.atan((d_r - d_l))/lenna.wheel_distance

    if (d_th > np.pi) :
        d_th -= 2*np.pi

    elif (d_th < -np.pi) :
        d_th += 2*np.pi

    # this is changing rpy to quatrenion how? go find it yourself
    quatrenion = Quaternion()

    quatrenion.x = 0
    quatrenion.y = 0
    quatrenion.z = np.sin(avg_angle/2)
    quatrenion.w = np.cos(avg_angle/2)

    odom.pose.pose.position.x = odom_old.pose.pose.position.x + np.cos(d_th) * d_avg_dist
    odom.pose.pose.position.y = odom_old.pose.pose.position.y + np.sin(d_th) * d_avg_dist
    odom.pose.pose.orientation = quatrenion

    # odom.pose.pose.orientation.z = avg_angle

    
    #   Prevent lockup from a single bad cycle
    if (np.isnan(odom.pose.pose.position.x) or np.isnan(odom.pose.pose.position.y)
        or np.isnan(odom.pose.pose.position.z)):
        odom.pose.pose.position.x = odom_old.pose.pose.position.x
        odom.pose.pose.position.y = odom_old.pose.pose.position.y
        odom.pose.pose.orientation = odom_old.pose.pose.orientation
 
    #    Make sure theta stays in the correct range
    # if (odom.pose.pose.orientation.z > np.pi):
    #     odom.pose.pose.orientation.z -= 2 * np.pi
    # elif (odom.pose.pose.orientation.z < -np.pi): 
    #     odom.pose.pose.orientation.z += 2 * np.pi

    
    #  Compute the velocity
    odom.header.stamp = rospy.Time.now()
    odom.twist.twist.linear.x = (right_vel + left_vel) / 2
    odom.twist.twist.angular.z = 2 * (right_vel - left_vel) / lenna.wheel_distance
 
    # Save th e pose data for the next cycle
    odom_old.pose.pose.position.x = odom.pose.pose.position.x
    odom_old.pose.pose.position.y = odom.pose.pose.position.y
    odom_old.pose.pose.orientation = odom.pose.pose.orientation
    odom_old.header.stamp = odom.header.stamp
 
    #   Publish the odometry message
    #   odom_data_pub.publish(odomNew);


if __name__ == "__main__":
    rospy.init_node('node_odom', anonymous=False)
    my_nav = rospy.Publisher('/odom', Odometry, queue_size=10)

    rate = rospy.Rate(100)
    
    odom.pose.pose.position.x = 0 #initial x which I think is 0
    odom.pose.pose.position.y = 0
    odom.pose.pose.position.z = 0

    old_l = 0
    old_r = 0

    while not rospy.is_shutdown():
        enc, _, _ = lenna.getOdometry(100)
        enc = getSigned(enc)

        odom.header.frame_id = 'odom'
        odom.child_frame_id = "base_link"
        update_odom(enc[0], enc[1], enc[2], enc[3], old_l, old_r)
        old_l,old_r = enc[2] ,enc[3]

        odom.header.stamp = rospy.Time.now()
        
        my_nav.publish(odom)
        rate.sleep()

    #not sure
    #odom.pose.position.z = 0
    #odom.pose.orientation



#      q.setRPY(0, 0, odomNew.pose.pose.orientation.z);
 
#   nav_msgs::Odometry quatOdom;
#   quatOdom.header.stamp = odomNew.header.stamp;
#   quatOdom.header.frame_id = "odom";
#   quatOdom.child_frame_id = "base_link";
#   quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
#   quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
#   quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
#   quatOdom.pose.pose.orientation.x = q.x();
#   quatOdom.pose.pose.orientation.y = q.y();
#   quatOdom.pose.pose.orientation.z = q.z();
#   quatOdom.pose.pose.orientation.w = q.w();
#   quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
#   quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
#   quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
#   quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
#   quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
#   quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;
  

#  odomNew.header.frame_id = "odom";
# odomNew.pose.pose.position.z = 0;
# odomNew.pose.pose.orientation.x = 0;
# odomNew.pose.pose.orientation.y = 0;
# odomNew.twist.twist.linear.x = 0;
# odomNew.twist.twist.linear.y = 0;
# odomNew.twist.twist.linear.z = 0;
# odomNew.twist.twist.angular.x = 0;
# odomNew.twist.twist.angular.y = 0;
# odomNew.twist.twist.angular.z = 0;
# odomOld.pose.pose.position.x = initialX;
# odomOld.pose.pose.position.y = initialY;
# odomOld.pose.pose.orientation.z = initialTheta;

# // Launch ROS and create a node
# ros::init(argc, argv, "ekf_odom_pub");
# ros::NodeHandle node;