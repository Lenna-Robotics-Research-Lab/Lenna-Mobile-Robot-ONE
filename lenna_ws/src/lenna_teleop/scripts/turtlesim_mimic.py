#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from lenna_msgs.msg import OdomInformation

def translationSubscriberCallback(data):
    vel = Twist()
    vel.linear.x = data.encoder_data.twist.twist.linear.x
    vel.angular.z = data.encoder_data.twist.twist.angular.z

    translation_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    translation_pub.publish(vel)

    rospy.loginfo('turtlesim command published!')

def translationSubscriber():
    rospy.init_node('translation_node', anonymous=True) 
    rospy.Subscriber('/odom_serial', OdomInformation, translationSubscriberCallback)
    rospy.spin()

if __name__ == '__main__':
    translationSubscriber()