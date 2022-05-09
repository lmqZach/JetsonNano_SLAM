#!/usr/bin/env python

import rospy
import cv2
import apriltag
from std_msgs.msg import Float32MultiArray
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 


def pose_callback(msg):
    # TODO: estimate control actions 
    cmd_msg = Float32MultiArray()

    # refactor controller into here?

    ctrl_pub.publish(cmd_msg)


class SlamNode:
    def __init__(self):
        rospy.init_node('planner_node')

        self.control_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=2)
        rospy.Subscriber("/current_pose", Pose, pose_callback)

        rospy.spin()

