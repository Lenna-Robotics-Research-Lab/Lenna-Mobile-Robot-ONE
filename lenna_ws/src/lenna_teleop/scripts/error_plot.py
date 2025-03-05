#!/usr/bin/python3

import rospy
from tf2_msgs.msg import TFMessage 
from nav_msgs.msg import Path

from std_msgs.msg import Bool

import numpy as np
import pandas as pd


from lenna_mobile_robot import *

import matplotlib.pyplot as plt

path_dict = {
    "time" : [],
    "x" : [],
    "y" : [],
    "theta" : []  }

lenna_dict = {
    "time" : [],
    "x" : [],
    "y" : [],
    "theta" : []  }

lcl_dict = {
    "time" : [],
    "x" : [],
    "y" : []  }

gbl_dict = {
    "time" : [],
    "x" : [],
    "y" : []  }


def tfSubscriberCallback(data):
    global lenna_dict
    if data.transforms:  # ? Make sure there's at least one transform
        transform = data.transforms[0]  # Get the first transform (adjust as needed)
        if transform.header.frame_id == "odom":
            lenna_dict["time"].append(transform.header.stamp.to_sec())  # Convert to seconds
            lenna_dict["x"].append(transform.transform.translation.x)
            lenna_dict["y"].append(transform.transform.translation.y)
            lenna_dict["theta"].append(np.arccos(transform.transform.rotation.w)*2)

def pathSubscriberCallback(data):
    global path_dict 
    if data:
        for pose_stamped in data.poses:
            path_dict["x"].append(pose_stamped.pose.position.x)  
            path_dict["y"].append(pose_stamped.pose.position.y)  
            path_dict["time"].append(pose_stamped.header.stamp)
            path_dict["theta"].append(np.arccos(pose_stamped.pose.orientation.w)*2)
def lclpathSubscriberCallback(data):
    global lcl_dict 
    if data:
        for pose_stamped in data.poses:
            lcl_dict["x"].append(pose_stamped.pose.position.x)  
            lcl_dict["y"].append(pose_stamped.pose.position.y)  
            lcl_dict["time"].append(pose_stamped.header.stamp)

def gblpathSubscriberCallback(data):
    global gbl_dict 
    if data:
        for pose_stamped in data.poses:
            gbl_dict["x"].append(pose_stamped.pose.position.x)  
            gbl_dict["y"].append(pose_stamped.pose.position.y)  
            gbl_dict["time"].append(pose_stamped.header.stamp)


def save_path_to_csv():
    """Save collected path data to CSV files."""
    pd.DataFrame(path_dict).to_csv("saved_path_full_short.csv", index=False)
    pd.DataFrame(lenna_dict).to_csv("actual_path_full_short.csv", index=False)
    pd.DataFrame(lcl_dict).to_csv("lcl_path_full_short.csv", index=False)
    pd.DataFrame(gbl_dict).to_csv("gbl_path_full_short.csv", index=False)
    rospy.loginfo("Paths saved as saved_path.csv and actual_path.csv")

def main():
    rospy.init_node("path_saver", anonymous=False)

    # Set the rate (limit loop speed)
    rate = rospy.Rate(10)  # 10 Hz (adjust as needed)

    # Subscribe to necessary topics
    rospy.Subscriber('/tf', TFMessage, tfSubscriberCallback)
    rospy.Subscriber('/move_base/NavfnROS/plan', Path, pathSubscriberCallback)
    rospy.Subscriber('/move_base/LocalPlannerROS/local_plan', Path, lclpathSubscriberCallback)
    rospy.Subscriber('/move_base/LocalPlannerROS/local_plan', Path, gblpathSubscriberCallback)

    rospy.loginfo("Tracking paths... Press k to save.")

    while (not rospy.is_shutdown()):
        rate.sleep()  # ? Controls loop speed (prevents CPU overload)

    save_path_to_csv()

if __name__ == "__main__":
    main()


