#!/usr/bin/python3

import rospy 

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

import numpy as np

# Topic callback function.
def tfListenerCallback(data):
    rospy.loginfo(' The contents of topic1: %s', data.data)

def tfListener():
    rospy.init_node('tf2http_listener_node' , anonymous = False)
    
    rospy.Subscriber('tf' , String, stringListenerCallback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    tfListener()