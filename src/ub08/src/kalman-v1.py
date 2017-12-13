#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError
import tf
import math

k = 0.5

class kalman_filter:
  def __init__(self):

    self.gps_pub = rospy.Publisher("/kalman/odom", Odometry, queue_size = 100)

    self.bridge = CvBridge()
    self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomcallback, queue_size=1)
    self.gps_sub = rospy.Subscriber("/visual_gps/odom", Odometry, self.gpsCallback, queue_size=1)
    self.init = False

  def odomCallback(self,data):
      self.odom = data

  def gpsCallback(self, data):
      self.gps = data


def main(args):
  rospy.init_node('kalman-filter', anonymous=True)
  ic = kalman_filter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)

