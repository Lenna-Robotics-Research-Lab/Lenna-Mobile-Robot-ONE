#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist

def translationSubscriberCallback(data):
    translation_pub = rospy.Publisher('/LENNA/diff_drive_controller/cmd_vel', Twist, queue_size=10)
    translation_pub.publish(data)
    rospy.loginfo('turtlesim command published!')

def translationSubscriber():
    rospy.init_node('remap_node', anonymous=True) 
    rospy.Subscriber('/cmd_vel', Twist, translationSubscriberCallback)
    rospy.spin()

if __name__ == '__main__':
    translationSubscriber()