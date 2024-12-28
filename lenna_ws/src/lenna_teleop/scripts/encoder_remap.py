#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from lenna_msgs.msg import OdomInformation

def translationSubscriberCallback(data):
    odom = Odometry()
    odom = data.encoder_data

    translation_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    translation_pub.publish(odom)

def translationSubscriber():
    rospy.init_node('odom_remap_node', anonymous=True) 
    rospy.Subscriber('/odom_serial', OdomInformation, translationSubscriberCallback)

    rospy.spin()

if __name__ == '__main__':
    translationSubscriber()
