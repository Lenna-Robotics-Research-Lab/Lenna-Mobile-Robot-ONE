#!/usr/bin/python2.7

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class OdomTransformer:
    def __init__(self):
        # Initialize the ROS node and the broadcaster
        rospy.init_node('node_odom_transformer', anonymous=True)
        
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_trans = TransformStamped()

        self.parent_frame = rospy.get_param("~odom_tf_parent_frame", "odom")
        self.child_frame = rospy.get_param("~odom_tf_child_frame", "base_link")

        # Setup subscriber for Odometry data
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.transformation_subscriber_callback)

    def transformation_subscriber_callback(self, data):
        """Callback function to process the incoming Odometry data and broadcast the transform."""
        self.odom_trans.header.stamp = data.header.stamp
        self.odom_trans.header.frame_id = self.parent_frame
        self.odom_trans.child_frame_id = self.child_frame
        self.odom_trans.transform.translation.x = data.pose.pose.position.x
        self.odom_trans.transform.translation.y = data.pose.pose.position.y
        self.odom_trans.transform.translation.z = 0
        self.odom_trans.transform.rotation = data.pose.pose.orientation

        # Send the transform
        self.broadcaster.sendTransform(self.odom_trans)

    def spin(self):
        """Keep the node running and processing callbacks."""
        rospy.spin()

def main():
    # Create an instance of the OdomTransformer class and start the process
    transformer = OdomTransformer()
    transformer.spin()

if __name__ == '__main__':
    main()
