#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32


def yawCallback(yaw):
    yaw = yaw.data
    pub.publish( math.pow(DESIRED_YAW - yaw, 2))


# ---main---
rospy.init_node("squared")


DESIRED_YAW = -91.0

rospy.Subscriber("/model_car/yaw", Float32, yawCallback, queue_size=1)
pub = rospy.Publisher("/squared", Float32, queue_size=10)

rospy.spin()