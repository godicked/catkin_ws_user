#!/usr/bin/env python
import rospy
import cv2
import sys
import numpy as np
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import tf
import math
import random

rospy.init_node('mc_filter', anonymous=True)

def buildMarker(id, pair):

	particle = pair[0]
	weight = pair[1]

	q = tf.transformations.quaternion_from_euler(0, 0, particle[2])
	marker = Marker()
	marker.header.frame_id = "map"
	marker.header.stamp = rospy.Time.now()
	marker.ns = "particle"
	marker.id = id
	marker.type = Marker.ARROW
	marker.action = Marker.MODIFY
	marker.pose.position.x = particle[0]
	marker.pose.position.y = particle[1]
	marker.pose.position.z = 1
	marker.pose.orientation.x = q[0]
	marker.pose.orientation.y = q[1]
	marker.pose.orientation.z = q[2]
	marker.pose.orientation.w = q[3]
	marker.scale.x = 0.2
	marker.scale.y = 0.1
	marker.scale.z = 0.1
	marker.color.a = 1.0
	marker.color.r = weight * 255.0
	marker.color.g = 0.0
	marker.color.b = 0.0

	return marker

def generateParticle(minX, maxX, minY, maxY):
	randomX = random.uniform(minX, maxX)
	randomY = random.uniform(minY, maxY)
	randomYaw = random.uniform(-math.pi, math.pi)
	return [randomX, randomY, randomYaw]

def generateParticleSet(minX, maxX, minY, maxY, size):
	particles = []

	for i in range(size):
		particles.append(generateParticle(minX, maxX, minY, maxY))
	return particles

def publishParticles(pub, particles):
	ma = MarkerArray()
	i = 0
	for p in particles:
		ma.markers.append(buildMarker(i, p))
		i += 1
	pub.publish(ma)

def compute_angle(particle, l1, l2):
	a = np.array(l1)
	b = np.array([particle[0], particle[1]])
	c = np.array(l2)

	ba = a - b
	bc = c - b

	cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
	angle = np.arccos(cosine_angle)
	if angle > math.pi:
		angle -= math.pi
	return angle

def compute_angles(particle, lamps):
	angles = []
	for i in range(len(lamps)-1):
		angles.append(compute_angle(particle, lamps[i], lamps[i+1]))
	return angles


def low_variance_resample(weighted_particles):
	particles = []
	M = len(weighted_particles)
	r = random.uniform(0.0, 1/(M+1))
	c = weighted_particles[0][1]
	i = 0
	for m in range(M):
		u = r + m * (1.0/(M+1))
		while u > c:
			i += 1
			c += weighted_particles[i][1]
		# add particle
		x = weighted_particles[i][0][0] + random.uniform(-0.3, 0.3)
		y = weighted_particles[i][0][1] + random.uniform(-0.3, 0.3)
		yaw = weighted_particles[i][0][2]
		particles.append([x, y, yaw])
	return particles 

def weightParticles(particles, lamps, expected_angles):
	weights = []
	total = 0
	deviation = 0.8
	for p in particles:
		w = 1
		for i in range(len(lamps)-1):
			angle = compute_angle(p, lamps[i], lamps[i+1])
			diff = expected_angles[i] - angle
			
			if(diff < -math.pi):
				 diff += math.pi
			if diff > math.pi:
				diff -= math.pi
			
			w *= math.exp(-(diff**2) / (deviation**2))
		weights.append([p, w])
		total += w
	for pair in weights:
		pair[1] /= total
	return weights

lamp_world = [
      [2.29, 1.14 ], #green lamp
      [3.55, 3.03], #red lamp
      [4.18, 1.77], #blue lamp
      [2.29, 2.40]] #purple lamp

particles = generateParticleSet(0, 6, 0, 5, 100)
mc_pub = rospy.Publisher('/mcmarkerarray', MarkerArray, queue_size=10)
last_odom = None

count = 0
def gpsCallback(odom):
	global last_odom, particles, count

	if last_odom != None:
		dx = odom.pose.pose.position.x - last_odom.pose.pose.position.x
		dy = odom.pose.pose.position.y - last_odom.pose.pose.position.y
		t1 = last_odom.pose.pose.orientation
		t2 = odom.pose.pose.orientation
		(r, p, y1) = tf.transformations.euler_from_quaternion([t1.x, t1.y, t1.z, t1.w])
		(r, p, y2) = tf.transformations.euler_from_quaternion([t2.x, t2.y, t2.z, t2.w])
		dYaw = y2 - y1


		dist = math.sqrt(dx*dx + dy*dy)

		# print dist

		# update particle position
		for p in particles:

			xnoise = random.uniform(-1.0, 1.0) / 5.0
			ynoise = random.uniform(-1.0, 1.0) / 5.0
			yawnoise = random.uniform(-math.pi, math.pi) / 5.0

			p[0] = p[0] + dist * math.cos(p[2])  + xnoise
			p[1] = p[1] + dist * math.sin(p[2])  + ynoise
			
			yaw = p[2]
			yaw += dYaw + yawnoise
			while yaw < -math.pi:
				yaw += math.pi
			while yaw > math.pi:
				yaw -= math.pi
			
			p[2] = yaw
			

	expected_angles = compute_angles([odom.pose.pose.position.x, odom.pose.pose.position.y], lamp_world)
	weighted_particles = weightParticles(particles, lamp_world, expected_angles)
	publishParticles(mc_pub, weighted_particles)
	particles = low_variance_resample(weighted_particles)
	last_odom = odom

#def weightParticle(particle, lamps, )

sub = rospy.Subscriber('/visual_gps/odom', Odometry, gpsCallback, queue_size=100)

rospy.spin()