#!/usr/bin/python2.7

import rospy
import tf2_ros
import geometry_msgs.msg

import Jetson.GPIO as GPIO

import requests
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(19, GPIO.OUT)

flag = True
falg_goal_reached = True

rospy.init_node('get_base_link_pose')

tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)

try:
    while not rospy.is_shutdown():
        try:
            if falg_goal_reached and flag:
                flag = False
                trans = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                data = {"x" : trans.transform.translation.x, "y": trans.transform.translation.y}
                requests.post("http://192.168.177.128/beacon/receive.php", data=data)
                GPIO.output(19, GPIO.HIGH)  # LED ON
                print("Transform: ", trans.transform.translation)
                time.sleep(5)
            else:
                GPIO.output(19, GPIO.LOW)  # LED ON
            # state = GPIO.input(21)
            # if state == GPIO.HIGH and flag == False:
            #     flag = True
            #     trans = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            #     print("Transform: ", trans.transform.translation)
            # elif state == GPIO.LOW and flag == True:
            #     flag = False
            #     print("GPIO LOW")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Transform not available")
            
except KeyboardInterrupt:
    rospy.logerr("Shutting Down ...")

finally: 
    GPIO.cleanup()  # Clean up GPIO settings