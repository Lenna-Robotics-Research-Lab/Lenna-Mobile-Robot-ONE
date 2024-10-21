#!/usr/bin/python2.7

import rospy 
import tf 
import tf2_ros.transform_broadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster()

odom_trans = TransformStamped()

def transformationSubscriberCallback(data):

    odom_trans.header.stamp = data.header.stamp
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_link"
    odom_trans.transform.translation.x = data.pose.pose.position.x
    odom_trans.transform.translation.y = data.pose.pose.position.y
    odom_trans.transform.translation.z = 0
    odom_trans.transform.rotation = data.pose.pose.orientation

    broadcaster.sendTransform([odom_trans])

def transformationSubscriber():
    rospy.init_node('odom_transformer', anonymous=True) 
    rospy.Subscriber('/odom', Odometry, transformationSubscriberCallback)

    rospy.spin()

if __name__ == '__main__':
    transformationSubscriber()
