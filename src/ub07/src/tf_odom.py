#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError
import tf
import math


class kalman_filter:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=1)
        self.gps_sub = rospy.Subscriber("/visual_gps/odom", Odometry, self.gpsCallback, queue_size=1)
        self.pos = None
        self.ori = None
        self.gps = None

    def odomCallback(self, data):
        if self.gps == None:
            return
        
        if self.pos == None:
            sg = self.getState(self.gps)
            so = self.getState(data)
            self.pos = sg + so
            print 'odom', so
            print 'gps', sg
            print 'pos', self.pos
        
        br = tf.TransformBroadcaster()
        br.sendTransform( (self.pos[0,0], self.pos[1,0], 0),
                      tf.transformations.quaternion_from_euler(math.pi, 0, self.pos[2,0]),
                      data.header.stamp,
                      "odom",
                      "map")

    def gpsCallback(self, data):
        self.gps = data

    def getState(self, odom):
        pos = odom.pose.pose.position
        ori = odom.pose.pose.orientation

        q = [ori.x, ori.y, ori.z, ori.w]
        yaw = math.acos(ori.w) * 2  * (ori.z / math.fabs(ori.z))

        x = pos.x
        y = pos.y
        th = yaw
        return np.matrix([x, y, th]).T

def main(args):
    rospy.init_node('kalman_filter', anonymous=True)
    ic = kalman_filter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
