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

def dtime(odom_last, odom_now):
	return (odom_now.header.stamp - odom_last.header.stamp).nsecs / 10.0**9

def getVelocity(odom_last, odom_now):

	# to seconds
	dt = dtime(odom_last, odom_now)

	print 'dt', dt

	p1 = odom_last.pose.pose.position
	p2 = odom_now.pose.pose.position

	t1 = odom_last.pose.pose.orientation
	t2 = odom_now.pose.pose.orientation

	(r, p, y1) = tf.transformations.euler_from_quaternion([t1.x, t1.y, t1.z, t1.w])
	(r, p, y2) = tf.transformations.euler_from_quaternion([t2.x, t2.y, t2.z, t2.w])

	dx = (p2.x - p1.x)
	dy = (p2.y - p2.y)
	dth = y2 - y1
	while(math.fabs(dth) > math.pi):
		if(dth > math.pi):
			dth -= math.pi * 2
		if(dth < -math.pi):
			dth += math.pi * 2

	v = math.sqrt(dx*dx + dy*dy) / dt
	vth = dth / dt

	return (v, -vth)


class kalman_filter:
	def __init__(self):

		self.gps_pub = rospy.Publisher("/kalman/odom", Odometry, queue_size=100)

		self.bridge = CvBridge()
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=1)
		self.gps_sub = rospy.Subscriber("/visual_gps/odom", Odometry, self.gpsCallback, queue_size=1)
		self.init = False
		self.gps = None
		self.odom = None
		self.lastGps = rospy.Time(0)
		self.vels = None
		self.time = None
		self.pos = None

	def odomCallback(self, data):
		if self.odom != None:
			self.vels = getVelocity(self.odom, data)
		self.odom = data

	def gpsCallback(self, data):
		if self.gps != None:
			dt = rospy.Time.now() - self.time
			self.gps = data
			self.kalman(dt.nsecs / 10.0**9)
		else:
			self.gps = data
		self.time = rospy.Time.now()

	def kalman(self, dt):
		if self.odom == None or self.gps == None or self.vels == None:
			print 'One value is None'
			return

		gpsPos = self.gps.pose.pose.position
		gpsOrient = self.gps.pose.pose.orientation

		q = [gpsOrient.x, gpsOrient.y, gpsOrient.z, gpsOrient.w]
		(r, p, y) = tf.transformations.euler_from_quaternion(q)

		gpsX = gpsPos.x
		gpsY = gpsPos.y
		gpsYaw = y

		# prediction
		(v, vth) = self.vels
		
		if self.pos == None:
			lx = gpsX
			ly = gpsY
			lth = gpsYaw
		else:
			(lx, ly, lth) = self.pos
			

		px = lx + v * math.cos(lth) * dt
		py = ly + v * math.sin(lth) * dt
		pth = lth + vth * dt


		x = k * gpsX + (1 - k) * px
		y = k * gpsY + (1 - k) * py
		yaw = k * gpsYaw + (1 - k) * pth

		self.pos = (x, y, yaw)

		# print "dt %f" %dt

		# sending odometry  
		odom = Odometry()
		odom.header.frame_id  = 'map'
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
