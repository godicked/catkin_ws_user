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

k = 0.5


class kalman_filter:
	def __init__(self):

		self.gps_pub = rospy.Publisher("/kalman/odom", Odometry, queue_size=100)

		self.bridge = CvBridge()
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=1)
		self.gps_sub = rospy.Subscriber("/visual_gps/odom", Odometry, self.gpsCallback, queue_size=1)
		self.init = False
		self.gps = None
		self.odom = None

	def odomCallback(self, data):
		self.odom = data

	def gpsCallback(self, data):
			self.gps = data
			self.kalman()

	def kalman(self):
		if self.odom == None or self.gps == None:
			print 'One value is None'
			return
		gpsPos = self.gps.pose.pose.position
		gpsOrient = self.gps.pose.pose.orientation

		q = [gpsOrient.x, gpsOrient.y, gpsOrient.z, gpsOrient.w]
		(r, p, y) = tf.transformations.euler_from_quaternion(q)

		gpsX = gpsPos.x
		gpsY = gpsPos.y
		gpsYaw = y

		# print 'x, y, yaw', gpsX, gpsY, math.degrees(gpsYaw)

		odomPos = self.odom.pose.pose.position
		odomOrient = self.odom.pose.pose.orientation

		q = [odomOrient.x, odomOrient.y, odomOrient.z, odomOrient.w]
		(r, p, y) = tf.transformations.euler_from_quaternion(q)

		odomX = odomPos.x
		odomY = odomPos.y
		odomYaw = y

		x = k * gpsX + (1 - k) * odomX
		y = k * gpsY + (1 - k) * gpsY
		yaw = k * gpsYaw + (1 - k) * odomYaw

		# sending odometry  
		odom = Odometry()
		odom.header.frame_id  = 'odom'
		odom.header.seq = self.odom.header.seq
		odom.pose.pose.position.x = x
		odom.pose.pose.position.y = y

		odom.pose.pose.orientation = Quaternion( *tf.transformations.quaternion_from_euler(0, 0, yaw ) )

		self.gps_pub.publish( odom )


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
