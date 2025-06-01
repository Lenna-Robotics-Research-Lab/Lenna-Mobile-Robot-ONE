import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import tf

import Jetson.GPIO as GPIO
import requests

class GoalPublisher:
    def __init__(self, csv_path):
        self.goals = self.load_goals(csv_path)
        self.current_index = 0
        self.goal_sent = False

        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.sleep(1)  # Ensure publisher is ready
        rospy.loginfo("GoalPublisher loaded %d goals.", len(self.goals))

    def load_goals(self, path):
        data = np.loadtxt(path, delimiter=',')
        if data.ndim == 1:
            data = np.expand_dims(data, axis=0)
        return data

    def publish_next_goal(self):
        if self.current_index >= len(self.goals):
            rospy.loginfo("All goals reached.")
            return

        x, y, theta = self.goals[self.current_index]
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()

        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        goal_msg.pose.orientation.x = quaternion[0]
        goal_msg.pose.orientation.y = quaternion[1]
        goal_msg.pose.orientation.z = quaternion[2]
        goal_msg.pose.orientation.w = quaternion[3]

        self.goal_pub.publish(goal_msg)
        rospy.loginfo("Published goal %d: x=%.2f, y=%.2f, theta=%.2f",
                      self.current_index + 1, x, y, theta)

        self.goal_sent = True


# Global instance
goal_publisher = None
DTX_PIN = 23
DRX_PIN = 21

def status_callback(msg):
    global goal_publisher
    global DTX_PIN, DRX_PIN

    if not goal_publisher or not goal_publisher.goal_sent:
        return

    for status in msg.status_list:
        if status.status == 3:  # SUCCEEDED
            rospy.loginfo("Goal %d reached.", goal_publisher.current_index + 1)
            goal_publisher.current_index += 1
            goal_publisher.goal_sent = False

            trans = tf_buffer.lookup_transform('map', 'base_esp32', rospy.Time(0), rospy.Duration(1.0))
            data = {"x" : trans.transform.translation.x, "y" : trans.transform.translation.y}
            requests.post("http://192.168.109.128/beacon/receive.php", data=data)
            GPIO.output(DTX_PIN, GPIO.HIGH)

            rospy.sleep(1)

            GPIO.output(DTX_PIN, GPIO.LOW)
            goal_publisher.publish_next_goal() # reconsider

            break


if __name__ == '__main__':
    try:
        rospy.init_node('sequential_goal_publisher', anonymous=True)
        csv_path = rospy.get_param('~csv_file', '/path/to/goals.csv')
        goal_publisher = GoalPublisher(csv_path)

        rospy.Subscriber('/move_base/status', GoalStatusArray, status_callback)

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        DTX_PIN = 23
        DRX_PIN = 21

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(DRX_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(DTX_PIN, GPIO.OUT)

        # Start with the first goal
        goal_publisher.publish_next_goal()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass