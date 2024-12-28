#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Imu
from lenna_msgs.msg import OdomInformation

def translationSubscriberCallback(data):
    imu_data = Imu()
    imu_data = data.imu_data

    translation_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)
    translation_pub.publish(imu_data)

def translationSubscriber():
    rospy.init_node('imu_remap_node', anonymous=True) 
    rospy.Subscriber('/odom_serial', OdomInformation, translationSubscriberCallback)

    rospy.spin()

if __name__ == '__main__':
    translationSubscriber()
